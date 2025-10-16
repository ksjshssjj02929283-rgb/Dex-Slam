#include <boost/filesystem.hpp>
#include "glog/logging.h"
#include "jsoncpp/json/json.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"

#include "cartographer_ros/common/map_loader.h"


namespace cartographer_ros
{

bool MapLoader::LoadMapDataInfo(const std::string& map_file, 
                 carto::localization::MapDataInfo& map_data_info)
{
  LOG(INFO) << "map file path: " << map_file;

  if (!boost::filesystem::exists(map_file)) {
    LOG(ERROR) << "map file not exist: " << map_file;
    return false;
  }

  std::string fname = map_file;
  // MapMode mode = TRINARY;
  std::ifstream fin(map_file.c_str());
  if (fin.fail()) {
    LOG(ERROR) << "Map_server could not open " << map_file.c_str();
    return false;
  }

  YAML::Node doc = YAML::Load(fin);

  if (!doc["resolution"].IsDefined()) {
    LOG(ERROR) << "The map does not contain a resolution tag or it is invalid.";
    return false;
  }
  try {
    doc["resolution"] >> map_data_info.resolution;
  } catch (YAML::InvalidScalar& e) {
    LOG(ERROR) << "The map does not contain a resolution tag or it is invalid.";
    return false;
  }

  try {
    doc["negate"] >> map_data_info.negate;
  } catch (YAML::InvalidScalar& e) {
    LOG(ERROR) << "The map does not contain a negate tag or it is invalid.";
    return false;
  }
  try {
    doc["occupied_thresh"] >> map_data_info.occ_th;
  } catch (YAML::InvalidScalar& e) {
    LOG(ERROR) << "The map does not contain an occupied_thresh tag or it is invalid.";
    return false;
  }
  try {
    doc["free_thresh"] >> map_data_info.free_th;
  } catch (YAML::InvalidScalar& e) {
    LOG(ERROR)<<"The map does not contain a free_thresh tag or it is invalid.";
    return false;
  }
  try {
    doc["origin"][0] >> map_data_info.origin[0];
    doc["origin"][1] >> map_data_info.origin[1];
    doc["origin"][2] >> map_data_info.origin[2];
  } catch (YAML::InvalidScalar& e) {
    LOG(ERROR)<<"The map does not contain an origin tag or it is invalid.";
    return false;
  }

  if (!doc["grid_map"].IsDefined()) {
    LOG(ERROR) << "The map does not contain a grid_map tag or it is invalid.";
    return false;
  }
  try {
    doc["grid_map"] >> map_data_info.grid_filename;

    if(map_data_info.grid_filename.size() == 0)
    {
      LOG(ERROR) << "The grid_map tag cannot be an empty string.";
      return false;
    }

    boost::filesystem::path mapfpath(map_data_info.grid_filename);
    if (!mapfpath.is_absolute())
    {
      boost::filesystem::path dir(map_file);
      dir = dir.parent_path();
      mapfpath = dir / mapfpath;
      map_data_info.grid_filename = mapfpath.string();
    }
  } catch (YAML::InvalidScalar& e) {
    LOG(ERROR) << "The map does not contain an line_map tag or it is invalid.";
    return false;
  }

  if (!doc["line_map"].IsDefined()) {
    LOG(ERROR) << "The map does not contain a line_map tag or it is invalid.";
    return false;
  }
  try {
    doc["line_map"] >> map_data_info.line_filename;

    if(map_data_info.line_filename.size() == 0)
    {
      LOG(ERROR) << "The line_map tag cannot be an empty string.";
      return false;
    }

    boost::filesystem::path mapfpath(map_data_info.line_filename);
    if (!mapfpath.is_absolute())
    {
      boost::filesystem::path dir(map_file);
      dir = dir.parent_path();
      mapfpath = dir / mapfpath;
      map_data_info.line_filename = mapfpath.string();
    }
  } catch (YAML::InvalidScalar& e) {
    LOG(ERROR) << "The map does not contain a valid file.";
    return false;
  }

  if (!doc["cloud_map"].IsDefined()) {
    LOG(ERROR) << "The map does not contain a cloud_map tag or it is invalid.";
    return false;
  }
  try {
    doc["cloud_map"] >> map_data_info.cloud_filename;

    if(map_data_info.cloud_filename.size() == 0)
    {
      LOG(ERROR) << "The cloud_map tag cannot be an empty string.";
      return false;
    }

    boost::filesystem::path mapfpath(map_data_info.cloud_filename);
    if (!mapfpath.is_absolute())
    {
      boost::filesystem::path dir(map_file);
      dir = dir.parent_path();
      mapfpath = dir / mapfpath;
      map_data_info.cloud_filename = mapfpath.string();
    }
  } catch (YAML::InvalidScalar& e) {
    LOG(ERROR) << "The map does not contain a valid file.";
    return false;
  }

  if (!doc["highlight_cloud_map"].IsDefined()) {
    LOG(ERROR) << "The map does not contain a highlight_cloud_map tag or it is invalid.";
  } else {
    try {
      doc["highlight_cloud_map"] >> map_data_info.highlight_cloud_filename;

      if(map_data_info.cloud_filename.size() == 0)
      {
        LOG(ERROR) << "The highlight_cloud_map tag cannot be an empty string.";
        return false;
      }

      boost::filesystem::path mapfpath(map_data_info.highlight_cloud_filename);
      if (!mapfpath.is_absolute())
      {
        boost::filesystem::path dir(map_file);
        dir = dir.parent_path();
        mapfpath = dir / mapfpath;
        map_data_info.cloud_filename = mapfpath.string();
      }
    } catch (YAML::InvalidScalar& e) {
      LOG(ERROR) << "The map does not contain a valid file.";
      return false;
    }
  }

  return true;
}

bool MapLoader::LoadGridMap(
          const carto::localization::MapDataInfo& map_data_info,
          std::shared_ptr<carto::localization::GridMap2D>& grid_map)
{
  const std::string& grid_filename = map_data_info.grid_filename;
  LOG(INFO) << "grid map file path: " << grid_filename;

  if (!boost::filesystem::exists(grid_filename)) {
    LOG(ERROR) << "map file not exist: " << grid_filename;
    return false;
  }

  cv::Mat img = cv::imread(grid_filename, cv::IMREAD_GRAYSCALE);
  std::vector<uint8_t> data(img.begin<uint8_t>(), img.end<uint8_t>());
  
  int num_x_cells = img.rows; 
  int num_y_cells = img.cols; 
  double origin_x = map_data_info.origin[0];
  double origin_y = map_data_info.origin[1];
  double scale = map_data_info.resolution;

  Eigen::Vector2d max = Eigen::Vector2d(origin_x + num_y_cells * scale,
                                        origin_y + num_x_cells * scale);
  carto::mapping::CellLimits cell_limits(num_x_cells, num_y_cells);
  carto::mapping::MapLimits map_limits(scale, max, cell_limits);

  Eigen::Vector2f origin_eigen(origin_x, origin_y);
  grid_map.reset();
  grid_map = std::make_shared<carto::localization::GridMap2D>(map_limits);

  for (int x = 0; x < num_x_cells; x++) {
    for (int y = 0; y < num_y_cells; y++) {
      int xx = num_y_cells - y - 1;
      int yy = x;

      // 图片的像素值和栅格地图的值是相反的
      // 图片 0: 黑色， 255: 白色
      // 然后 kFreeCorrespondenceValue: 与占用概率prob也是相反的
      uchar value = data[xx + yy * num_y_cells];
      if (value == 126|| value == 127 || value == 128) {
        // 未知区域使用 kUnknownCorrespondenceValue (值为0，表示未知)
        grid_map->mutable_correspondence_cost_cells()->at(
          grid_map->ToFlatIndex(Eigen::Array2i{x, y})) = carto::mapping::kUnknownCorrespondenceValue;
      } else {
        // 图片灰度值转换为占用概率: 0(黑色)=障碍物, 255(白色)=自由空间
        double occ_prob = value / 255.0;
        if (occ_prob < 0.3) {
          // 暗色区域 (障碍物): 高占用概率 -> 低correspondence cost
          grid_map->mutable_correspondence_cost_cells()->at(
            grid_map->ToFlatIndex(Eigen::Array2i{x, y})) = carto::mapping::ProbabilityToValue(0.0);
        } else if (occ_prob < 0.7) {
          // 中间灰度区域: 按实际占用概率赋值
          grid_map->mutable_correspondence_cost_cells()->at(
            grid_map->ToFlatIndex(Eigen::Array2i{x, y})) = carto::mapping::ProbabilityToValue(occ_prob);
        } else {
          // 亮色区域 (自由空间): 低占用概率 -> 高correspondence cost
          grid_map->mutable_correspondence_cost_cells()->at(
            grid_map->ToFlatIndex(Eigen::Array2i{x, y})) = carto::mapping::ProbabilityToValue(1.0);
        }
      }
    }
  }
  grid_map->mutable_known_cells_box()->extend(
    Eigen::Vector2i(
      num_x_cells,
      num_y_cells).matrix());
  grid_map->mutable_known_cells_box()->extend(
    Eigen::Array2i(0, 0).matrix());
  // std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/tmp";
  // grid_map->DrawAsImage(home_dir + "/.dexe/data/", "loc_map.png");
  return true;
}


bool MapLoader::LoadCloudMap(const std::string& map_file,
std::shared_ptr<carto::localization::CloudMap2D>& cloud_map)
{
  LOG(INFO) << "cloud map_file: " << map_file;
  if (!boost::filesystem::exists(map_file)) {
    LOG(ERROR) << "Cloud map does not exist : " << map_file;
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(map_file, *cloud_pcl);
  
  carto::sensor::PointCloud point_cloud;
  for (const auto& p: cloud_pcl->points) {
    carto::sensor::PointCloud::PointType point;
    point.position.x() = p.x;
    point.position.y() = p.y;
    point.position.z() = 0.0;
    point_cloud.push_back(point);
  }
  cloud_map.reset(new carto::localization::CloudMap2D());
  cloud_map->AddCloud(point_cloud);

  return true;
}


bool MapLoader::LoadLaneLineMap(const std::string& map_file, 
      std::shared_ptr<carto::sensor::LineMapData>& lane_line_map)
{
  LOG(INFO) << "laneLine map_file: " << map_file;
  std::ifstream is(map_file.c_str(), std::ios::binary);
  if (!is.is_open()) {
    LOG(ERROR) << "cant open lane_line_map: "<<map_file;
    return false;
  }

  lane_line_map.reset(new carto::sensor::LineMapData());

  Json::Reader reader;
  Json::Value root;
  reader.parse(is, root);

  if (root.empty()) {
    LOG(ERROR) << "Failed to parse file: " << map_file;
    return false;
  }

  try {
    carto::sensor::LaneDataWithNormal tmp_data;
    for (const auto& node: root) {
      tmp_data.data.line_data.begin[0] = node["begin"]["x"].asDouble();
      tmp_data.data.line_data.begin[1] = node["begin"]["y"].asDouble();
      tmp_data.data.line_data.end[0] = node["end"]["x"].asDouble();
      tmp_data.data.line_data.end[1] = node["end"]["y"].asDouble();
      LOG(WARNING) << "begin: x: " << tmp_data.data.line_data.begin[0]
                   << ", y: "<< tmp_data.data.line_data.begin[1]
                   << ", end: x: " << tmp_data.data.line_data.end[0]
                   << ", y: " << tmp_data.data.line_data.end[1];
      lane_line_map->push_back(tmp_data);
    }
  } catch (const std::exception &e) {
    std::cout << "laneLineMapLoad Failed to parse JSON data: " << e.what() << std::endl;
    return false;
  }
  return true;
}

}  // namespace cartographer_ros
