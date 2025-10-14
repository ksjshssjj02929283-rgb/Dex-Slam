/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#include <ceres/cubic_interpolation.h>

#include "amcl/sensors/amcl_cloud.h"
#include "transform/rigid_transform.h"

// namespace cartographer {
// namespace localization {
namespace amcl {

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLCloud::AMCLCloud(const size_t max_beams, const cartographer::mapping::Grid2D* grid_map) : AMCLSensor()
{
  // time_ = 0.0;
  this->grid_map_ = grid_map;
  this->max_beams_ = max_beams;
  return;
}

AMCLCloud::~AMCLCloud()
{}


////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLCloud::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  pf_update_sensor(pf, (pf_sensor_model_fn_t) GridMapFieldModel, data);  

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose

double AMCLCloud::GridMapFieldModel(AMCLCloudData *data, pf_sample_set_t* set)
{
  AMCLCloud *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLCloud*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    Eigen::Vector2d trans(pose.v[0], pose.v[1]);
    cartographer::transform::Rigid2d p_pose(trans, pose.v[2]);
    cartographer::transform::Rigid3f p_pose_3d = cartographer::transform::Embed3D(p_pose).cast<float>();
    // Pre-compute a couple of things
    step = data->cloud_.size() / self->max_beams_;

    double prob_sum = 0.0;

    // Step size must be at least 1
    if(step < 1)
      step = 1;
    const GridArrayAdapter adapter(*(self->grid_map_));
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const cartographer::mapping::MapLimits& limits = self->grid_map_->limits();
    for (size_t i = 0; i < data->cloud_.size(); i += step) {
      const auto world = p_pose_3d * data->cloud_.points()[i];
      double prob;
      interpolator.Evaluate(
          (limits.max().x() - world.position.x()) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.max().y() - world.position.y()) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &prob);
      prob_sum += prob * prob * prob;
    }

    sample->weight *= prob_sum;
    total_weight += sample->weight;
  }

  return(total_weight);
}

}  // namespace amcl
// }  // namespace mapping
// }  // namespace cartographer
