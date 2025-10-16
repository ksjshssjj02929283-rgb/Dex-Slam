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
#include "scan_match/probability_grid.h"

#include <limits>

#include "absl/memory/memory.h"
#include "boost/filesystem.hpp"
#include "glog/logging.h"
#include "opencv2/opencv.hpp"

#include "scan_match/probability_values.h"
#include "scan_match/submaps.h"

namespace cartographer {
namespace mapping {

ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) {}

ProbabilityGrid::ProbabilityGrid(const proto::Grid2D& proto,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(proto, conversion_tables), conversion_tables_(conversion_tables) {
  CHECK(proto.has_probability_grid_2d());
}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  mutable_known_cells_box()->extend(cell_index.matrix());
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index);
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
  if (*cell >= kUpdateMarker) {
    return false;
  }
  mutable_update_indices()->push_back(flat_index);
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}

GridType ProbabilityGrid::GetGridType() const {
  return GridType::PROBABILITY_GRID;
}

// Returns the probability of the cell with 'cell_index'.
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!limits().Contains(cell_index)) return kMinProbability;
  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}

proto::Grid2D ProbabilityGrid::ToProto() const {
  proto::Grid2D result;
  result = Grid2D::ToProto();
  result.mutable_probability_grid_2d();
  return result;
}

std::unique_ptr<Grid2D> ProbabilityGrid::ComputeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = limits().resolution();
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  std::unique_ptr<ProbabilityGrid> cropped_grid =
      absl::make_unique<ProbabilityGrid>(
          MapLimits(resolution, max, cell_limits), conversion_tables_);
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) continue;
    cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
  }

  return std::unique_ptr<Grid2D>(cropped_grid.release());
}

/**
 * @brief 根据已有的grid2d和conversion_tables_计算出一个新的grid2d，并记录在texture中
 * 
 * @param texture 需要操作的texture
 * @param local_pose 
 * @return true 
 * @return false 
 */
// bool ProbabilityGrid::DrawToSubmapTexture(
//     proto::SubmapQuery::Response::SubmapTexture* const texture,
//     transform::Rigid3d local_pose) const {
//   Eigen::Array2i offset;
//   CellLimits cell_limits;
//   // 1. 计算submap原点和边界
//   ComputeCroppedLimits(&offset, &cell_limits);

//   // 2. 遍历submap内的所有cell
//   std::string cells;
//   // XYIndexRangeIterator，从cell_limits转换成xy的index
//   for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
//     // 2.1 如果不是已知的cell，则初始化数值和alpha为0（alpha为增益系数，为自己设置）
//     if (!IsKnown(xy_index + offset)) {
//       cells.push_back(0 /* unknown log odds value */);
//       cells.push_back(0 /* alpha */);
//       continue;
//     }
//     // We would like to add 'delta' but this is not possible using a value and
//     // alpha. We use premultiplied alpha, so when 'delta' is positive we can
//     // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
//     // zero, and use 'alpha' to subtract. This is only correct when the pixel
//     // is currently white, so walls will look too gray. This should be hard to
//     // detect visually for the user, though.
//     // 我们希望添加 'delta'，但是使用值和 alpha 是不可能的。
//     // 我们使用预乘 alpha，因此当 'delta' 为正数时，我们可以通过将 alpha 设置为零来相加。
//     // 如果 'delta' 为负数，我们将值设置为零，并使用 alpha 进行减法。
//     // 这仅在像素当前为白色时才正确，因此墙壁看起来会有些灰色。
//     // 不过，对于用户来说，这在视觉上应该很难察觉到。

//     // 2.2 计算当前cell的占用概率的增量delta
//     const int delta =
//         128 - ProbabilityToLogOddsInteger(GetProbability(xy_index + offset));
//     // 2.3 计算当前cell的占用值和alpha
//     const uint8 alpha = delta > 0 ? 0 : -delta;
//     const uint8 value = delta > 0 ? delta : 0;
//     cells.push_back(value);
//     cells.push_back((value || alpha) ? alpha : 1);
//   }

//   // 3. 压缩地图
//   common::FastGzipString(cells, texture->mutable_cells());
//   // 4. 设置地图的其他属性
//   texture->set_width(cell_limits.num_x_cells);
//   texture->set_height(cell_limits.num_y_cells);
//   const double resolution = limits().resolution();
//   texture->set_resolution(resolution);
//   const double max_x = limits().max().x() - resolution * offset.y();
//   const double max_y = limits().max().y() - resolution * offset.x();
//   // 5. 设置地图位姿（TODO:这个位姿的含义是什么，以及计算逻辑是什么）
//   *texture->mutable_slice_pose() = transform::ToProto(
//       local_pose.inverse() *
//       transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));

//   return true;
// }

bool ProbabilityGrid::DrawAsImage(
  const std::string& folder,
  const std::string& filename) const
{
  if (!boost::filesystem::is_directory(folder)) {
    if (!boost::filesystem::create_directory(folder)) {
      LOG(ERROR) << "DrawAsImage: " << folder << " is not a directory.";
      return false;
    }
  }

  Eigen::Array2i offset;
  CellLimits cell_limits;
  // 1. 计算submap原点和边界
  ComputeCroppedLimits(&offset, &cell_limits);
  // 初始化为 127（灰色），表示未知区域
  cv::Mat img(cell_limits.num_y_cells,
              cell_limits.num_x_cells,
              CV_8UC1, cv::Scalar(127));
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) {
      // 未知区域显示为灰色（127）
      img.at<uchar>(xy_index.y(), xy_index.x()) = 127;
      continue;
    }
    const int value = correspondence_cost_cells()[ToFlatIndex(xy_index)];
    img.at<uchar>(xy_index.y(), xy_index.x()) = static_cast<uchar>(ValueToProbability(value) * 255);
  }
  cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);

  cv::imwrite(folder + filename, img);
  return true;
}

}  // namespace mapping
}  // namespace cartographer
