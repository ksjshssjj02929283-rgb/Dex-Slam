/*
 * Copyright 2017 The Cartographer Authors
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
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cartographer_ros/mapping/ros_map.h"

#include "absl/strings/str_cat.h"

namespace cartographer_ros {

void WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer,
              double occ_thresh, double free_thresh)

{
  const std::string header =
      absl::StrCat("P5\n# Cartographer map; ", resolution, " m/pixel\n",
                   image.width(), " ", image.height(), "\n255\n");
  uchar occ_thresh_val = 255 * occ_thresh;
  uchar free_thresh_val = 255 * free_thresh;

  file_writer->Write(header.data(), header.size());
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      const char color = image.GetPixel(x, y)[0];
      u_char* uc_value = (u_char*)malloc(sizeof(u_char));
      memcpy(uc_value, &color, sizeof(char));
      if (*uc_value > occ_thresh_val) {
        *uc_value = 255;
      } else if (*uc_value < free_thresh_val) {
        *uc_value = 0;
      }
      char* c_value = (char*)malloc(sizeof(char));
      memcpy(c_value, uc_value, sizeof(u_char));
      file_writer->Write(c_value, 1);
    }
  }
}

void WriteYaml(const double resolution, const Eigen::Vector2d& origin,      
               const std::string& map_file_name,
               ::cartographer::io::FileWriter* file_writer,
               double occ_thresh, double free_thresh)
{  
  const std::string pgm_filename = map_file_name + ".png";
  const std::string line_map_filename = map_file_name + "_line_map.json";
  const std::string cloud_map_filename = map_file_name + ".pcd";
  const std::string highlight_cloud_map_filename = map_file_name + "_highlight.pcd";

  // Magic constants taken directly from ros map_saver code:
  // https://github.com/ros-planning/navigation/blob/ac41d2480c4cf1602daf39a6e9629142731d92b0/map_server/src/map_saver.cpp#L114
  const std::string output = absl::StrCat(
      "image: ", pgm_filename, "\n",
      "line_map: ", line_map_filename, "\n",
      "cloud_map: ", cloud_map_filename, "\n",
      "highlight_cloud_map: ", highlight_cloud_map_filename, "\n",
      "resolution: ", resolution, "\n",
      "origin: [", origin.x(), ", ", origin.y(),
      ", 0.0]\nnegate: 0\noccupied_thresh: ", occ_thresh, "\nfree_thresh: ", free_thresh, "\n");
  file_writer->Write(output.data(), output.size());
}

void WritePng(const ::cartographer::io::Image& image, 
              const std::string file_name,
              double occ_thresh, double free_thresh)
{
  cv::imwrite(file_name, GeneratePngMat(image, occ_thresh, free_thresh));
}

cv::Mat GeneratePngMat(const ::cartographer::io::Image& image,
                       double occ_thresh, double free_thresh)
{
  cv::Mat map_mat(image.height(), image.width(), CV_8UC1);
  uchar occ_thresh_val = 255 * occ_thresh;
  uchar free_thresh_val = 255 * free_thresh;
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      uchar value = image.GetPixel(x, y)[0];
      if (value > occ_thresh_val) {
        value = 255;
      } else if (value < free_thresh_val) {
        value = 0;
      }
      map_mat.at<uchar>(y, x) = value;
    }
  }
  return map_mat;
}

void WriteNavigationMap(const std::string pgm_file_name,
                       const std::string output_file_name,
                       const std::string& filter_mode,
                       bool save_debug_map,
                       const std::string& map_dir)
{
  cv::Mat original = cv::imread(pgm_file_name, cv::IMREAD_GRAYSCALE);
  if (original.empty()) {
    LOG(ERROR) << "Failed to read PNG file: " << pgm_file_name;
    return;
  }
  WriteNavigationMap(original, output_file_name, filter_mode, save_debug_map, map_dir);
}

void WriteNavigationMap(const cv::Mat& original,
                       const std::string output_file_name,
                       const std::string& filter_mode,
                       bool save_debug_map,
                       const std::string& map_dir)
{
  LOG(INFO) << "Starting WriteNavigationMap with noise removal for " << output_file_name;
  LOG(INFO) << "Input mat dimensions: " << original.cols << "x" << original.rows;

  // 创建debug_map目录（在地图目录下）
  std::string debug_dir = map_dir + "/debug_map";
  if (save_debug_map) {
    std::string mkdir_cmd = "mkdir -p " + debug_dir;
    int ret = system(mkdir_cmd.c_str());
    if (ret != 0) {
      LOG(WARNING) << "Failed to create debug directory: " << debug_dir;
    } else {
      LOG(INFO) << "Created debug directory: " << debug_dir;
    }
  }
  
  // 获取滤波模式参数
  bool use_area_filter = true;
  int min_noise_area = 10;

  if (filter_mode == "normal") {
    use_area_filter = true;
    min_noise_area = 10;
    LOG(INFO) << "Using normal filter mode.";
  } else if (filter_mode == "high") {
    use_area_filter = true;
    min_noise_area = 50;
    LOG(INFO) << "Using high filter mode.";
  } else if (filter_mode == "super") {
    use_area_filter = true;
    min_noise_area = 100;
    LOG(INFO) << "Using super filter mode.";
  } else {
    LOG(WARNING) << "Unknown filter mode: " << filter_mode << ", using default (normal)";
    use_area_filter = true;
    min_noise_area = 10;
  }
  
  // 固定参数（与Python版本debug.py完全一致）
  const int white_thresh = 250;
  const int gray_thresh = 250;
  const int black_thresh = 160;
  const int dilate_radius = 3;
  
  // 步骤2: 对PNG图像进行噪声处理
  cv::Mat result = original.clone();
  
  // 生成三类区域mask（与debug.py完全一致）
  cv::Mat white_mask = (original >= white_thresh) * 255;  // 转为255显示
  cv::Mat black_mask = (original <= black_thresh) * 255;  // 转为255显示
  cv::Mat gray_mask = ((original > black_thresh) & (original < gray_thresh)) * 255;  // 转为255显示

  // 保存mask图像（仅在debug模式）
  if (save_debug_map) {
    cv::imwrite(debug_dir + "/01_white_mask.png", white_mask );  // 转为255显示
    cv::imwrite(debug_dir + "/02_black_mask.png", black_mask);
    cv::imwrite(debug_dir + "/03_gray_mask.png", gray_mask);
  }
  
  // 对黑色区域做连通域分析
  cv::Mat labels, stats, centroids;
  int num_labels = cv::connectedComponentsWithStats(black_mask, labels, stats, centroids, 8);
  
  LOG(INFO) << "Found " << (num_labels - 1) << " connected components in black regions";
  
  // 保存连通域标签可视化（仅在debug模式）
  if (save_debug_map) {
    cv::Mat labels_vis;
    cv::normalize(labels, labels_vis, 0, 255, cv::NORM_MINMAX);
    labels_vis.convertTo(labels_vis, CV_8U);
    cv::applyColorMap(labels_vis, labels_vis, cv::COLORMAP_JET);
    cv::imwrite(debug_dir + "/04_connected_components.png", labels_vis);
  }
  
  // 构造膨胀核
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, 
      cv::Size(2 * dilate_radius + 1, 2 * dilate_radius + 1));
  
  int removed_count = 0;
  int kept_by_gray = 0;
  int kept_by_size = 0;
  
  // 创建中间处理结果可视化图像（用于debug）
  cv::Mat removed_mask = cv::Mat::zeros(original.size(), CV_8U);
  cv::Mat edge_mask = cv::Mat::zeros(original.size(), CV_8U);  // 05_edge_mask: 灰色邻居区域
  
  // 处理每个连通域（逻辑与debug.py完全一致）
  for (int i = 1; i < num_labels; ++i) {
    cv::Mat component_mask = (labels == i) * 255;
    
    // 条件①：面积约束（仅use_area_filter=True时生效）
    if (use_area_filter) {
      int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area >= min_noise_area) {
        kept_by_size++;
        continue;
      }
    }
    
    // 条件②：灰色邻居约束
    cv::Mat dilated;
    cv::dilate(component_mask, dilated, kernel, cv::Point(-1, -1), 1);
    
    cv::Mat neighbor_mask;
    cv::bitwise_and(dilated, ~component_mask, neighbor_mask);
    
    cv::Mat gray_neighbors;
    cv::bitwise_and(neighbor_mask, gray_mask, gray_neighbors);
    
    bool has_gray_neighbor = cv::countNonZero(gray_neighbors) > 0;
    
    if (has_gray_neighbor) {
      kept_by_gray++;
      // 记录边缘区域（有灰色邻居的区域）
      edge_mask.setTo(255, component_mask);
    } else {
      // 判定为噪点，清除（改为白色255）
      result.setTo(255, component_mask);
      removed_mask.setTo(255, component_mask);
      removed_count++;
    }
  }
  
  // 保存debug图片（仅在save_debug_map=true时）
  if (save_debug_map) {
    // 05_edge_mask.png - 显示灰色邻居区域（边缘检测）
    cv::imwrite(debug_dir + "/05_edge_mask.png", edge_mask);
    
    // 06_annotated.png - 标注图：原图 + 不同颜色的标记
    cv::Mat annotated;
    cv::cvtColor(original, annotated, cv::COLOR_GRAY2BGR);  // 转为彩色图
    
    // 蓝色标记：被移除的噪点
    cv::Mat removed_bgr = cv::Mat::zeros(original.size(), CV_8UC3);
    removed_bgr.setTo(cv::Scalar(255, 0, 0), removed_mask);  // 蓝色 (BGR)
    cv::addWeighted(annotated, 1.0, removed_bgr, 1.0, 0, annotated);
    
    // 绿色标记：边缘区域（灰色邻居）
    cv::Mat edge_bgr = cv::Mat::zeros(original.size(), CV_8UC3);
    edge_bgr.setTo(cv::Scalar(0, 255, 0), edge_mask);  // 绿色 (BGR)
    cv::addWeighted(annotated, 1.0, edge_bgr, 1.0, 0, annotated);
    
    cv::imwrite(debug_dir + "/06_annotated.png", annotated);
  }
  
  LOG(INFO) << "Noise removal statistics:";
  LOG(INFO) << "  Filter mode: " << filter_mode;
  LOG(INFO) << "  Removed components (noise): " << removed_count;
  LOG(INFO) << "  Kept components (has gray neighbors): " << kept_by_gray;
  
  // 步骤3: 保存为PNG文件
  if (!cv::imwrite(output_file_name, result)) {
    LOG(ERROR) << "Failed to write navigation map to " << output_file_name;
    return;
  }
  
  // 保存最终结果和对比图（仅在debug模式）
  if (save_debug_map) {
    // 07_final_result.png - 最终处理后的结果
    cv::imwrite(debug_dir + "/07_final_result.png", result);
    
    // 08_comparison.png - 左右对比：原始 vs 最终结果
    cv::Mat comparison;
    cv::hconcat(original, result, comparison);
    cv::imwrite(debug_dir + "/08_comparison.png", comparison);
  }
  
  LOG(INFO) << "Successfully saved navigation map to " << output_file_name;
}

}  // namespace cartographer_ros
