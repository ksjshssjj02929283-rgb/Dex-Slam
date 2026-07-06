//
// Created by xiang on 25-3-18.
//
// 离线建图主程序 - 参考 Cartographer 的 mapping offline node 设计
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <filesystem>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "common/map_layout.h"
#include "core/system/slam.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

DEFINE_string(input_bag, "", "输入 ROS2 bag 文件路径");
DEFINE_string(config, "./config/default.yaml", "配置文件路径");
DEFINE_string(map_folder, "", "地图存储根目录（空 = 默认 ~/.dexe/data/maps）");
DEFINE_string(map_name, "offline_map", "地图名称");

/// 运行离线建图，支持 Pangolin 可视化
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据包，使用 --input_bag <bag_path>";
        return -1;
    }

    using namespace lightning;

    // 验证并创建地图目录（底图统一保存到 <map_folder>/<map_name>/slam/）
    FLAGS_map_folder = ResolveMapsRoot(FLAGS_map_folder);
    const std::string map_dir = MapSlamDir(FLAGS_map_folder, FLAGS_map_name) + "/";
    try {
        std::filesystem::create_directories(map_dir);
        LOG(INFO) << "Map directory: " << map_dir;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to create map directory: " << e.what();
        return -1;
    }

    RosbagIO rosbag(FLAGS_input_bag);

    SlamSystem::Options options;
    options.online_mode_ = false;

    SlamSystem slam(options);

    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    slam.StartSLAM(FLAGS_map_name);

    // 启动 spin 线程，使 TF / RViz 发布正常工作
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(slam.GetNode());
    std::thread spin_thread([&executor]() { executor->spin(); });

    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");

    rosbag
        /// IMU 的处理
        .AddImuHandle(imu_topic,
                      [&slam](IMUPtr imu) {
                          slam.ProcessIMU(imu);
                          return true;
                      })

        /// lidar 的处理
        .AddPointCloud2Handle(lidar_topic,
                              [&slam](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                  slam.ProcessLidar(msg);
                                  return true;
                              })
        /// livox 的处理
        .AddLivoxCloudHandle("/livox/lidar",
                             [&slam](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
                                 slam.ProcessLidar(cloud);
                                 return true;
                             })
        .Go();

    slam.SaveMap(map_dir);
    Timer::PrintAll();

    executor->cancel();
    spin_thread.join();
    rclcpp::shutdown();

    LOG(INFO) << "Offline SLAM completed. Map saved to: " << map_dir;

    return 0;
}