//
// Created by xiang on 25-3-18.
//
// 离线定位主程序 - 通过 LocSystem 统一管理 ROS 发布 / TF / RViz
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <filesystem>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "common/map_layout.h"
#include "core/system/loc_system.h"
#include "io/yaml_io.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>

DEFINE_string(input_bag, "", "输入 ROS2 bag 文件路径");
DEFINE_string(config, "./config/default.yaml", "配置文件路径");
DEFINE_string(map_folder, "", "地图根目录（空 = 默认 ~/.dexe/data/maps）");
DEFINE_string(map_name, "offline_map", "地图名称");

/// 运行离线定位，针对已有地图进行轨迹跟踪测试
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据包，使用 --input_bag <bag_path>";
        rclcpp::shutdown();
        return -1;
    }

    // 验证地图是否存在（优先 slam/ 分层布局，回退旧扁平布局）
    FLAGS_map_folder = lightning::ResolveMapsRoot(FLAGS_map_folder);
    const std::string map_dir = lightning::ResolveMapDir(FLAGS_map_folder, FLAGS_map_name) + "/";
    const std::string map_file = map_dir + "global.pcd";
    if (!std::filesystem::exists(map_file)) {
        LOG(ERROR) << "Map file not found: " << map_file;
        LOG(ERROR) << "Please ensure map exists at: " << map_dir;
        rclcpp::shutdown();
        return -1;
    }

    using namespace lightning;

    LocSystem::Options options;
    options.online_mode_ = false;
    auto yaml_cfg = YAML::LoadFile(FLAGS_config);
    if (yaml_cfg["system"] && yaml_cfg["system"]["pub_tf"]) {
        options.pub_tf_ = yaml_cfg["system"]["pub_tf"].as<bool>();
    }
    if (yaml_cfg["system"] && yaml_cfg["system"]["with_rviz"]) {
        options.with_rviz_ = yaml_cfg["system"]["with_rviz"].as<bool>();
    }
    LocSystem loc_system(options);

    if (!loc_system.Init(FLAGS_config, map_dir)) {
        LOG(ERROR) << "Failed to initialize localization";
        rclcpp::shutdown();
        return -1;
    }

    loc_system.StartLocalization();

    // 启动 spin 线程，使 TF / RViz 发布正常工作
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(loc_system.GetNode());
    std::thread spin_thread([&executor]() { executor->spin(); });

    YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");

    RosbagIO rosbag(FLAGS_input_bag);
    rosbag
        .AddImuHandle(imu_topic,
                      [&loc_system](IMUPtr imu) {
                          loc_system.ProcessIMU(imu);
                          usleep(1000);
                          return true;
                      })
        .AddPointCloud2Handle(lidar_topic,
                              [&loc_system](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  loc_system.ProcessLidar(cloud);
                                  usleep(1000);
                                  return true;
                              })
        .AddLivoxCloudHandle("/livox/lidar",
                             [&loc_system](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
                                 loc_system.ProcessLidar(cloud);
                                 usleep(1000);
                                 return true;
                             })
        .Go();

    Timer::PrintAll();

    executor->cancel();
    spin_thread.join();
    rclcpp::shutdown();

    LOG(INFO) << "Offline localization completed. Map: " << FLAGS_map_name;

    return 0;
}