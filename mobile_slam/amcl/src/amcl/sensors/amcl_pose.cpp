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

#include "amcl/sensors/amcl_pose.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLPose::AMCLPose() : AMCLSensor()
{
  this->time = 0.0;

  return;
}

AMCLPose::~AMCLPose()
{
}


////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLPose::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  pf_update_sensor(pf, (pf_sensor_model_fn_t) PoseUpdateModel, data);

  return true;
}


double AMCLPose::PoseUpdateModel(AMCLPoseData *data, pf_sample_set_t* set)
{
  double p;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  Eigen::Matrix3d cov_eigen;
  cov_eigen << data->cov.m[0][0], data->cov.m[0][1], data->cov.m[0][2],
               data->cov.m[1][0], data->cov.m[1][1], data->cov.m[1][2],
               data->cov.m[2][0], data->cov.m[2][1], data->cov.m[2][2];
  Eigen::Vector3d pose_eigen = Eigen::Map<Eigen::Vector3d>(data->pose.v);

  total_weight = 0.0;
  // Compute the sample weights
  for (int j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    Eigen::Vector3d particle_pose_eigen = Eigen::Map<Eigen::Vector3d>(pose.v);
    p = ComputeNormalPdf(particle_pose_eigen, pose_eigen, cov_eigen);
    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

double AMCLPose::ComputeNormalPdf(const Eigen::Vector3d& x,
                                  const Eigen::Vector3d& mean,
                                  const Eigen::Matrix3d& cov)
{
  double det_cov = cov.determinant();

  if (det_cov <= 0) {
    return 0.0;
  }
  Eigen::MatrixXd inv_cov = cov.inverse();

  Eigen::VectorXd diff = x - mean;
  diff.z() = std::fmod(x.z() - mean.z() + M_PI, 2 * M_PI) - M_PI;
  // std::cout << "diff: " << diff.transpose() << std::endl;
  double exponent = -0.5 * diff.transpose() * inv_cov * diff;
  // std::cout << "exponent: " << exponent << std::endl;
    // Calculate normalization constant
  // double coeff = std::pow(2 * M_PI, k / 2) * std::pow(det_cov, 0.5);

  return std::exp(exponent);
}