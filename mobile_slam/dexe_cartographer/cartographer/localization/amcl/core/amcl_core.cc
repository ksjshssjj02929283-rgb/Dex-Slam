/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/localization/amcl/core/amcl_core.h"

#include <limits>
#include <memory>
#include <cmath>

#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

#include "absl/memory/memory.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/sync_point_cloud.h"
#include "cartographer/common/tic_toc.h"

namespace cartographer {
namespace localization {
namespace amcl {

AMCLCore::AMCLCore(
    const proto::AMCLCoreOptions& options)
    : options_(options)
{}

AMCLCore::~AMCLCore() {}

bool AMCLCore::SetInitialPose(const transform::Rigid2d& init_pose)
{
  if (grid_ == nullptr || particle_filter_ == nullptr ||
      amcl_cloud_ == nullptr || amcl_odom_ == nullptr)
  {
    LOG(WARNING) << "Map is not set!";
    return false;
  }

  amcl_hyp_t initial_pose_hyp;
  initial_pose_hyp.pf_pose_mean.v[0] = init_pose.translation().x();
  initial_pose_hyp.pf_pose_mean.v[1] = init_pose.translation().y();
  initial_pose_hyp.pf_pose_mean.v[2] = init_pose.rotation().angle();
  initial_pose_hyp.pf_pose_cov = pf_matrix_zero();

  initial_pose_hyp.pf_pose_cov.m[0][0] = options_.initial_cov_xx();
  initial_pose_hyp.pf_pose_cov.m[1][1] = options_.initial_cov_yy();
  initial_pose_hyp.pf_pose_cov.m[2][2] = options_.initial_cov_aa();

  assert(particle_filter_ == nullptr);
  pf_init(particle_filter_,
          initial_pose_hyp.pf_pose_mean,
          initial_pose_hyp.pf_pose_cov);

  is_initialized_ = true;

  return true;
}

void AMCLCore::SetGrid(const mapping::Grid2D* grid)
{
  grid_ = grid;
  amcl_cloud_ = std::make_unique<AMCLCloud>(options_.max_beams(), grid_);
  assert(amcl_cloud_);

  if (!is_initialized_) {
    InitAMCLModel();
    InitParticleFilter();
  }
}

bool AMCLCore::UpdateWithSensorData(
              const common::Time time,
              const sensor::PointCloud& point_cloud,
              const sensor::OdometryData& odometry_data)
{
  if (!is_initialized_) {
    LOG(WARNING) << "AMCLCore is not initialized.";
    return false;
  }

  if (grid_ == nullptr) {
    LOG(ERROR) << "Grid is not set.";
    return false;
  }

  if (last_odom_data_ == nullptr) {
    last_odom_data_ = std::make_shared<sensor::OdometryData>(odometry_data);
  } else {
    AMCLOdomData amcl_odom_data;
    const transform::Rigid2d odom_2d = transform::Project2D(odometry_data.pose);

    pf_vector_t pose;
    pose.v[0] = odom_2d.translation().x();
    pose.v[1] = odom_2d.translation().y();
    pose.v[2] = odom_2d.rotation().angle();

    transform::Rigid2d curr_odom_pose = transform::Project2D(odometry_data.pose);
    transform::Rigid2d last_odom_pose = transform::Project2D(last_odom_data_->pose);

    pf_vector_t delta = pf_vector_zero();
    delta.v[0] = curr_odom_pose.translation().x() - last_odom_pose.translation().x();
    delta.v[1] = curr_odom_pose.translation().y() - last_odom_pose.translation().y();
    delta.v[2] = angle_diff(curr_odom_pose.rotation().angle(), last_odom_pose.rotation().angle());

    amcl_odom_data.pose = pose;
    amcl_odom_data.delta = delta;
    
    // amcl_odom_data.time = odometry_data.time.ToSeconds();
    amcl_odom_->UpdateAction(particle_filter_, (AMCLSensorData*)&amcl_odom_data);

    last_odom_data_ = std::make_shared<sensor::OdometryData>(odometry_data);
  }

  AMCLCloudData amcl_cloud_data;
  // amcl_cloud_data.time = time.ToSeconds();
  amcl_cloud_data.cloud = point_cloud;
  amcl_cloud_data.sensor = amcl_cloud_.get();
  amcl_cloud_->UpdateSensor(particle_filter_, (AMCLSensorData*)&amcl_cloud_data);

  pf_update_resample(particle_filter_);

  // 对于每一个聚类，计算其均值和协方差
  // Read out the current hypotheses
  double max_weight = 0.0;
  int max_weight_hyp = -1;
  std::vector<amcl_hyp_t> hyps;
  hyps.resize(particle_filter_->sets[particle_filter_->current_set].cluster_count);
  for (int hyp_count = 0;
       hyp_count < particle_filter_->sets[particle_filter_->current_set].cluster_count;
       hyp_count++)
  {
    double weight;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    if (!pf_get_cluster_stats(particle_filter_, hyp_count, &weight, &pose_mean, &pose_cov)) {
      // ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
      break;
    }

    hyps[hyp_count].weight = weight;
    hyps[hyp_count].pf_pose_mean = pose_mean;
    hyps[hyp_count].pf_pose_cov = pose_cov;

    if (hyps[hyp_count].weight > max_weight) {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
  }

  // 权重最大的聚类，作为当前的位姿估计
  if (max_weight > 0.0) {
    // Copy in the covariance, converting from 3-D to 6-D
    Eigen::Vector2d trans(hyps[max_weight_hyp].pf_pose_mean.v[0],
                          hyps[max_weight_hyp].pf_pose_mean.v[1]);
    amcl_pose_ = std::make_shared<transform::Rigid2d>(
                        trans, hyps[max_weight_hyp].pf_pose_mean.v[2]);
    pf_sample_set_t* set = particle_filter_->sets + particle_filter_->current_set;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        covariance_(i, j) = set->cov.m[i][j];
      }
    }
  }

  return true;
}

std::vector<std::pair<int, int>> AMCLCore::free_space_indices_;
pf_vector_t AMCLCore::UniformPoseGenerator(void* arg)
{
  const mapping::Grid2D* grid = (mapping::Grid2D*)arg;
  unsigned int rand_index = drand48() * free_space_indices_.size();
  std::pair<int,int> free_point = free_space_indices_[rand_index];
  pf_vector_t p;
  Eigen::Array2i cell(free_point.first, free_point.second);
  const auto& map_limits = grid->limits();
  
  Eigen::Vector2f cell_center = map_limits.GetCellCenter(cell);
  p.v[0] = cell_center.x();
  p.v[1] = cell_center.y();
  p.v[2] = drand48() * 2 * M_PI - M_PI;

  return p;
}

void AMCLCore::InitParticleFilter()
{
  if (particle_filter_ != nullptr) {
    pf_free(particle_filter_);
  }

  // Index of free space
  free_space_indices_.resize(0);
  const auto& map_limits = grid_->limits();
  const auto& cell_limits = map_limits.cell_limits();
  for (int i = 0; i < cell_limits.num_x_cells; i++) {
    for (int j = 0; j < cell_limits.num_y_cells; j++) {
      Eigen::Array2i cell_index(i, j);
      double prob = mapping::CorrespondenceCostToProbability(
                              grid_->GetCorrespondenceCost(cell_index));
      if (prob < 0.3) {
        free_space_indices_.push_back(std::make_pair(i,j));
      }
    }
  }

  // Create the particle filter
  particle_filter_ = pf_alloc(options_.min_particles(),
                              options_.max_particles(),
                              options_.recovery_alpha_slow(),
                              options_.recovery_alpha_fast(),
                              (pf_init_model_fn_t)UniformPoseGenerator,
                              (void*)grid_);

  pf_set_selective_resampling(particle_filter_,
                              false);
}

void AMCLCore::InitAMCLModel()
{
  amcl_odom_ = std::make_unique<AMCLOdom>();
  assert(amcl_odom_);
  odom_model_t odom_model_type;

  if (options_.odom_model_type() == "diff") {
    odom_model_type = ODOM_MODEL_DIFF;
  } else if(options_.odom_model_type() == "omni") {
    odom_model_type = ODOM_MODEL_OMNI;
  } else if(options_.odom_model_type() == "diff-corrected") {
    odom_model_type = ODOM_MODEL_DIFF_CORRECTED;
  } else if(options_.odom_model_type() == "omni-corrected") {
    odom_model_type = ODOM_MODEL_OMNI_CORRECTED;
  } else {
    LOG(WARNING) << "Unknown odom model type \"" << options_.odom_model_type() << "\"; defaulting to diff model";
    odom_model_type = ODOM_MODEL_DIFF;
  }

  amcl_odom_->SetModel(odom_model_type,
                       options_.odom_alpha1(),
                       options_.odom_alpha2(),
                       options_.odom_alpha3(),
                       options_.odom_alpha4(),
                       options_.odom_alpha5());

  LOG(INFO) << "Done initializing Odom model.";


  LOG(INFO) << "Done initializing Cloud model.";
}

pf_sample_set_t* AMCLCore::GetParticleSet() const
{
  if (particle_filter_ == nullptr) {
    return nullptr;
  }
  return particle_filter_->sets + particle_filter_->current_set;
}

std::vector<transform::Rigid2d> AMCLCore::GetParticlePoses()
{
  pf_sample_set_t* set = particle_filter_->sets + particle_filter_->current_set;

  std::vector<transform::Rigid2d> poses;

  for(int i = 0;i < set->sample_count; i++) {
    Eigen::Vector2d trans(set->samples[i].pose.v[0],
                          set->samples[i].pose.v[1]);
    transform::Rigid2d pose(trans, set->samples[i].pose.v[2]);
    poses.push_back(pose);
  }
  return poses;
}



}  // namespace amcl
}  // namespace localization
}  // namespace cartographer
