//
// Created by xiang on 25-3-18.
//
// 在线建图系统主程序 - 提供 ROS2 服务管理建图生命周期
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <filesystem>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_interfaces/srv/get_map_list.hpp"
#include "nav_interfaces/srv/get_string.hpp"
#include "nav_interfaces/srv/set_map_name.hpp"

#include "core/system/slam.h"
#include "utils/timer.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(map_folder, "./data", "地图存储根目录");

namespace lightning {
namespace {

/**
 * 在线建图管理节点
 *
 * 服务：
 *   ~/set_map_name      - 设置地图名（建图前必须先调用）
 *   ~/start_mapping     - 开始建图
 *   ~/finish_mapping    - 完成建图并保存地图
 *   ~/cancel_mapping    - 取消建图（不保存）
 *   ~/save_map          - 保存地图（等同 finish_mapping）
 *   ~/get_mapping_status - 查询建图状态（"mapping" / "idle"）
 *   ~/get_map_list       - 获取已有地图列表
 *
 * 话题：
 *   mapping_status (std_msgs/Int16)  1=建图中, -1=空闲
 */
class SlamMappingNode : public rclcpp::Node {
   public:
    explicit SlamMappingNode() : rclcpp::Node("mapping_3d_manager") {
        using namespace std::chrono_literals;

        mapping_status_pub_ = create_publisher<std_msgs::msg::Int16>("mapping_status", 10);

        set_map_name_srv_ = create_service<nav_interfaces::srv::SetMapName>(
            "~/set_map_name",
            std::bind(&SlamMappingNode::SetMapNameCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        start_mapping_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/start_mapping",
            std::bind(&SlamMappingNode::StartMappingCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        finish_mapping_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/finish_mapping",
            std::bind(&SlamMappingNode::FinishMappingCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        cancel_mapping_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/cancel_mapping",
            std::bind(&SlamMappingNode::CancelMappingCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        save_map_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/save_map",
            std::bind(&SlamMappingNode::SaveMapCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        get_mapping_status_srv_ = create_service<nav_interfaces::srv::GetString>(
            "~/get_mapping_status",
            std::bind(&SlamMappingNode::GetMappingStatusCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        get_map_list_srv_ = create_service<nav_interfaces::srv::GetMapList>(
            "~/get_map_list",
            std::bind(&SlamMappingNode::GetMapListCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        status_timer_ = create_wall_timer(200ms, [this]() { PublishStatus(); });

        LOG(INFO) << "SlamMappingNode initialized. map_folder=" << FLAGS_map_folder;
    }

    ~SlamMappingNode() {
        if (is_mapping_) {
            StopSlam(false);
        }
    }

   private:
    // ---- Service Callbacks ----

    void SetMapNameCallback(const std::shared_ptr<nav_interfaces::srv::SetMapName::Request> req,
                            std::shared_ptr<nav_interfaces::srv::SetMapName::Response> res) {
        if (is_mapping_) {
            LOG(ERROR) << "Cannot change map name while mapping is running.";
            res->success = false;
            return;
        }
        map_name_ = req->map_name;
        if (!EnsureDirectoryExists(FLAGS_map_folder) ||
            !EnsureDirectoryExists(FLAGS_map_folder + "/" + map_name_)) {
            res->success = false;
            return;
        }
        RCLCPP_INFO(get_logger(), "Map name set: %s", map_name_.c_str());
        res->success = true;
    }

    void StartMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (is_mapping_) {
            res->success = false;
            res->message = "Already mapping";
            return;
        }
        if (map_name_.empty()) {
            res->success = false;
            res->message = "Map name not set. Call ~/set_map_name first.";
            return;
        }

        SlamSystem::Options opts;
        opts.online_mode_ = true;
        slam_ = std::make_unique<SlamSystem>(opts);

        if (!slam_->Init(FLAGS_config)) {
            slam_.reset();
            res->success = false;
            res->message = "Failed to initialize SlamSystem";
            return;
        }

        slam_->StartSLAM(map_name_);

        // 在独立线程中启动 SlamSystem 的传感器处理 executor
        slam_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        slam_executor_->add_node(slam_->GetNode());
        slam_thread_ = std::thread([this]() { slam_executor_->spin(); });

        is_mapping_ = true;
        LOG(INFO) << "SLAM started for map: " << map_name_;
        res->success = true;
        res->message = "OK";
    }

    void FinishMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (!is_mapping_) {
            if (!map_name_.empty()) {
                map_name_.clear();
                res->success = true;
                res->message = "Map name cleared, ready for new mapping";
            } else {
                res->success = true;
                res->message = "Already idle";
            }
            return;
        }
        const std::string save_path = FLAGS_map_folder + "/" + map_name_ + "/";
        StopSlam(true, save_path);
        res->success = true;
        res->message = "OK";
    }

    void CancelMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (!is_mapping_) {
            map_name_.clear();
            res->success = true;
            res->message = "Already idle";
            return;
        }
        StopSlam(false);
        res->success = true;
        res->message = "Mapping cancelled";
    }

    void SaveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (!is_mapping_) {
            res->success = false;
            res->message = "Not mapping";
            return;
        }
        const std::string save_path = FLAGS_map_folder + "/" + map_name_ + "/";
        StopSlam(true, save_path);
        res->success = true;
        res->message = "OK";
    }

    void GetMappingStatusCallback(
        const std::shared_ptr<nav_interfaces::srv::GetString::Request> /*req*/,
        std::shared_ptr<nav_interfaces::srv::GetString::Response> res) {
        res->data = is_mapping_ ? "mapping" : "idle";
        res->success = true;
    }

    void GetMapListCallback(
        const std::shared_ptr<nav_interfaces::srv::GetMapList::Request> /*req*/,
        std::shared_ptr<nav_interfaces::srv::GetMapList::Response> res) {
        const std::string& folder = FLAGS_map_folder;
        if (folder.empty() || !std::filesystem::exists(folder)) {
            res->map_list = {};
            res->success = true;
            return;
        }
        std::vector<std::string> maps;
        try {
            for (const auto& entry : std::filesystem::directory_iterator(folder)) {
                if (!entry.is_directory()) continue;
                // 含 global.pcd 或 map.pgm 视为有效 lightning 地图
                if (std::filesystem::exists(entry.path() / "global.pcd") ||
                    std::filesystem::exists(entry.path() / "map.pgm")) {
                    maps.push_back(entry.path().filename().string());
                }
            }
        } catch (const std::filesystem::filesystem_error& e) {
            LOG(WARNING) << "GetMapList filesystem error: " << e.what();
        }
        res->map_list = maps;
        res->success = true;
    }

    // ---- Helpers ----

    bool EnsureDirectoryExists(const std::string& path) {
        try {
            if (path.empty()) return false;
            if (std::filesystem::exists(path)) return true;
            return std::filesystem::create_directories(path);
        } catch (const std::exception& e) {
            LOG(ERROR) << "Filesystem error for '" << path << "': " << e.what();
            return false;
        }
    }

    /// 停止 SlamSystem：先停执行器（避免并发），再选择性保存地图
    void StopSlam(bool save, const std::string& save_path = "") {
        if (slam_executor_) {
            slam_executor_->cancel();
        }
        if (slam_thread_.joinable()) {
            slam_thread_.join();
        }
        slam_executor_.reset();

        if (save && slam_) {
            slam_->SaveMap(save_path);
        }

        slam_.reset();
        map_name_.clear();
        is_mapping_ = false;
    }

    void PublishStatus() {
        auto msg = std_msgs::msg::Int16();
        msg.data = is_mapping_ ? 1 : -1;
        mapping_status_pub_->publish(msg);
    }

    // ---- State ----
    std::string map_name_;
    bool is_mapping_{false};

    std::unique_ptr<SlamSystem> slam_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> slam_executor_;
    std::thread slam_thread_;

    // ROS2 通信
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mapping_status_pub_;

    rclcpp::Service<nav_interfaces::srv::SetMapName>::SharedPtr set_map_name_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mapping_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_mapping_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_mapping_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    rclcpp::Service<nav_interfaces::srv::GetString>::SharedPtr get_mapping_status_srv_;
    rclcpp::Service<nav_interfaces::srv::GetMapList>::SharedPtr get_map_list_srv_;
};

}  // namespace
}  // namespace lightning

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::AllowCommandLineReparsing();
    google::ParseCommandLineFlags(&argc, &argv, true);

    auto node = std::make_shared<lightning::SlamMappingNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    lightning::Timer::PrintAll();
    rclcpp::shutdown();

    LOG(INFO) << "done";
    return 0;
}
