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
  const std::string pgm_filename = map_file_name + ".pgm";
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
  // 边缘增强策略，会将非边缘区域变为黑色，需要增加逻辑
  // cv::Ptr<cv::ximgproc::FastLineDetector> lsd =
  //   cv::ximgproc::createFastLineDetector(
  //     25, M_PI_4, 128, 178, 3, true);

  // std::vector<cv::Vec4f> lines_std;
  //   lsd->detect(map_mat, lines_std);
  // std::cout << "lines_std.size() = " << lines_std.size() << std::endl;

  // for (auto lines : lines_std) {
  //   std::cout << lines.val[0] << " " << lines.val[1] << " " << lines.val[2] << " " << lines.val[3] << std::endl;
  // }
  // lsd->drawSegments(map_mat, lines_std);

  cv::imwrite(file_name, map_mat);

  // cv::Mat blurred_image;  
  // cv::GaussianBlur(map_mat, blurred_image, cv::Size(5, 5), 0);  

  // // 定义锐化卷积核  
  // cv::Mat sharpening_kernel = (cv::Mat_<float>(3, 3) <<   
  //     0, -1, 0,  
  //     -1, 5, -1,  
  //     0, -1, 0);  

  // // 锐化处理  
  // cv::Mat sharpened_image;  
  // cv::filter2D(blurred_image, sharpened_image, -1, sharpening_kernel); 

  // cv::imwrite(file_name, sharpened_image);
}

}  // namespace cartographer_ros
