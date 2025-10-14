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

#ifndef AMCL_CLOUD_H_
#define AMCL_CLOUD_H_

#include <iostream>
#include <cstring>
#include <string>
#include "amcl/sensors/amcl_sensor.h"
#include "amcl/map/map.h"
#include "common/time.h"
#include "scan_match/grid_2d.h"
#include "sensor/point_cloud.h"
#include "transform/rigid_transform.h"

// namespace cartographer {
// namespace localization {
namespace amcl {

// Laser sensor data
class AMCLCloudData : public AMCLSensorData
{
public:
  AMCLCloudData () {};
  virtual ~AMCLCloudData() {};

  AMCLCloudData(const AMCLCloudData& obj)
  : time_(obj.time_),
    frame_id_(obj.frame_id_),
    cloud_(obj.cloud_)
  {}

  AMCLCloudData(const AMCLCloudData&& obj)
  : time_(obj.time_),
    frame_id_(obj.frame_id_),
    cloud_(obj.cloud_)
  {}

  AMCLCloudData &operator=(const AMCLCloudData &obj)
  {
    if (this != &obj) {
      time_ = obj.time_;
      frame_id_ = obj.frame_id_;
      cloud_ = obj.cloud_;
    }
    return *this;
  }
  

public:
  cartographer::common::Time time_;
  std::string frame_id_;
  cartographer::sensor::PointCloud cloud_;
};


// Laseretric sensor model
class AMCLCloud : public AMCLSensor
{
public:
  // Default constructor
  AMCLCloud(const size_t max_beams,
            const cartographer::mapping::Grid2D* grid_map);

  virtual ~AMCLCloud(); 

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  static double GridMapFieldModel(AMCLCloudData *data, 
                                  pf_sample_set_t* set);

private:
  // Current data timestamp
  cartographer::common::Time time_;
  // Current map
  const cartographer::mapping::Grid2D* grid_map_;

  size_t max_beams_ = 50;

  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const cartographer::mapping::Grid2D& grid) : grid_(grid) {}

    void GetValue(const int row, const int column, double* const value) const {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = cartographer::mapping::kMinProbability;
      } else {
        double cost = static_cast<double>(grid_.GetCorrespondenceCost(
                          Eigen::Array2i(column - kPadding, row - kPadding)));
        double problity = cartographer::mapping::CorrespondenceCostToProbability(cost);
        *value = problity;
      }
    }

    int NumRows() const {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    const cartographer::mapping::Grid2D& grid_;
  };
};

}  // namespace amcl
// }  // namespace mapping
// }  // namespace cartographer

#endif  // AMCL_CLOUD_H_
