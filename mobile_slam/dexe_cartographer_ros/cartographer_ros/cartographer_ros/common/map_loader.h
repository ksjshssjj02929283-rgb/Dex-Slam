#ifndef CARTOGRAPHER_ROS_COMMON_MAP_LOADER_H_
#define CARTOGRAPHER_ROS_COMMON_MAP_LOADER_H_

#include <libgen.h>
#include <fstream>
#include "yaml-cpp/yaml.h"

#include "cartographer/localization/2d/grid_map_2d.h"
#include "cartographer/localization/2d/cloud_map_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/localization/2d/map_data.h"
#include "cartographer/sensor/line_data.h"

template <typename T>
void operator>>(const YAML::Node& node, T& i) {
  i = node.as<T>();
}

namespace cartographer_ros {

namespace carto = ::cartographer;

class MapLoader {
 public:
  MapLoader() = default;
  ~MapLoader() = default;

  bool LoadMapDataInfo(
    const std::string& map_file,
    carto::localization::MapDataInfo& map_data_info);
  bool LoadGridMap(
    const carto::localization::MapDataInfo& map_data_info,
    std::shared_ptr<carto::localization::GridMap2D>& grid_map);
  bool LoadLaneLineMap(
    const std::string& map_file,
    std::shared_ptr<carto::sensor::LineMapData>& lane_line_map);
  bool LoadCloudMap(
    const std::string& map_file,
    std::shared_ptr<carto::localization::CloudMap2D>& cloud_map);

 private:
  /*
   bool LoadMap(const std::string map_file,
               nav_msgs::GetMap::Response& map_resp);*/
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_COMMON_MAP_LOADER_H_
