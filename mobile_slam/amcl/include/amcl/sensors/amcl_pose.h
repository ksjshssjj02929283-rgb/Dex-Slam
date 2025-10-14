/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
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
// Desc: LASER sensor model for AMCL
// Author: Andrew Howard
// Date: 17 Aug 2003
// CVS: $Id: amcl_laser.h 6443 2008-05-15 19:46:11Z gerkey $
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_POSE_H
#define AMCL_POSE_H

#include <iostream>
#include <cstring>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "amcl/sensors/amcl_sensor.h"
#include "../map/map.h"

namespace amcl
{


// Laser sensor data
class AMCLPoseData : public AMCLSensorData
{
  public:
    AMCLPoseData () {};
    virtual ~AMCLPoseData() {};

    AMCLPoseData(const AMCLPoseData& obj)
    : time(obj.time),
      frame_id(obj.frame_id),
      pose(obj.pose),
      cov(obj.cov)
    {
    }

    AMCLPoseData(const AMCLPoseData&& obj)
    : time(obj.time),
      frame_id(obj.frame_id),
      pose(obj.pose),
      cov(obj.cov)
    {
    }

    AMCLPoseData &operator=(const AMCLPoseData &obj)
    {
      if (this != &obj) {
        time = obj.time;
        frame_id = obj.frame_id;
        pose = obj.pose;
        cov = obj.cov;
      }
      return *this;
    }
    
  // Laser range data (range, bearing tuples)
  public: double time;
  public: std::string frame_id;
  public: pf_vector_t pose;
  public: pf_matrix_t cov;
};


// Laseretric sensor model
class AMCLPose : public AMCLSensor
{
  // Default constructor
  public: AMCLPose();

  public: virtual ~AMCLPose(); 

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  // Set the laser's pose after construction

  // Determine the probability for the given pose
  private: static double PoseUpdateModel(AMCLPoseData *data, 
                                         pf_sample_set_t* set);

  private: static double ComputeNormalPdf(const Eigen::Vector3d& x,
                                          const Eigen::Vector3d& mean,
                                          const Eigen::Matrix3d& cov);
  // Current data timestamp
  private: double time;

  // The laser map
  private: map_t *map;

  // Laser offset relative to robot
  private: pf_vector_t laser_pose;
};


}

#endif
