//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>

#include "common/map_layout.h"
#include "core/system/loc_system.h"
#include "nav_interfaces/srv/get_status.hpp"
#include "nav_interfaces/srv/get_string.hpp"
#include "nav_interfaces/srv/set_map_name.hpp"
#include "nav_interfaces/srv/set_pose.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <yaml-cpp/yaml.h>

DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(map_folder, "", "地图根目录（空 = 默认 ~/.dexe/data/maps）");
DEFINE_string(map_name, "", "地图名称");

namespace lightning {
namespace {

class LocalizationManagerNode : public rclcpp::Node {
   public:
    explicit LocalizationManagerNode() : rclcpp::Node("localization_3d_manager") {
        using namespace std::chrono_literals;

        localization_status_pub_ = create_publisher<std_msgs::msg::Int16>("localization_status", 10);

        set_map_name_srv_ = create_service<nav_interfaces::srv::SetMapName>(
            "~/set_map_name",
            std::bind(&LocalizationManagerNode::HandleSetMapName, this, std::placeholders::_1, std::placeholders::_2));

        set_init_pose_srv_ = create_service<nav_interfaces::srv::SetPose>(
            "~/set_init_pose",
            std::bind(&LocalizationManagerNode::HandleSetInitPose, this, std::placeholders::_1, std::placeholders::_2));

        start_localization_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/start_localization", std::bind(&LocalizationManagerNode::HandleStartLocalization, this,
                                              std::placeholders::_1, std::placeholders::_2));

        stop_localization_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/stop_localization", std::bind(&LocalizationManagerNode::HandleStopLocalization, this,
                                             std::placeholders::_1, std::placeholders::_2));

        reset_localization_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/reset_localization", std::bind(&LocalizationManagerNode::HandleResetLocalization, this,
                                              std::placeholders::_1, std::placeholders::_2));

        set_map_from_manager_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/set_map_from_manager", std::bind(&LocalizationManagerNode::HandleSetMapFromManager, this,
                                                std::placeholders::_1, std::placeholders::_2));

        get_localization_status_srv_ = create_service<nav_interfaces::srv::GetStatus>(
            "~/get_localization_status", std::bind(&LocalizationManagerNode::HandleGetLocalizationStatus, this,
                                                   std::placeholders::_1, std::placeholders::_2));

        get_map_folder_srv_ = create_service<nav_interfaces::srv::GetString>(
            "~/get_map_folder", std::bind(&LocalizationManagerNode::HandleGetMapFolder, this, std::placeholders::_1,
                                          std::placeholders::_2));

        get_map_name_srv_ = create_service<nav_interfaces::srv::GetString>(
            "~/get_map_name",
            std::bind(&LocalizationManagerNode::HandleGetMapName, this, std::placeholders::_1, std::placeholders::_2));

        status_timer_ = create_wall_timer(200ms, std::bind(&LocalizationManagerNode::PublishStatus, this));

        RCLCPP_INFO(get_logger(), "LocalizationManagerNode initialized");
    }

    ~LocalizationManagerNode() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        StopLocalizationInternal();
    }

   private:
    enum class LocalizationState { STOPPED, STARTING, RUNNING, STOPPING, ERROR };

    std::string StateToString(LocalizationState s) const {
        switch (s) {
            case LocalizationState::STOPPED:
                return "stopped";
            case LocalizationState::STARTING:
                return "starting";
            case LocalizationState::RUNNING:
                return "running";
            case LocalizationState::STOPPING:
                return "stopping";
            case LocalizationState::ERROR:
                return "error";
        }
        return "unknown";
    }

    bool GetMapInfoFromMapManager(std::string& map_folder, std::string& map_name) {
        try {
            auto temp_node = std::make_shared<rclcpp::Node>("lightning_loc_map_manager_client");
            auto param_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node, "map_manager");
            if (!param_client->wait_for_service(std::chrono::seconds(5))) {
                RCLCPP_ERROR(get_logger(), "map_manager parameter service unavailable");
                return false;
            }

            auto params = param_client->get_parameters({"map_folder", "map_name"});
            if (params.size() != 2) {
                RCLCPP_ERROR(get_logger(), "map_manager returns invalid parameter set");
                return false;
            }
            if (params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
                params[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                RCLCPP_ERROR(get_logger(), "map_manager map_folder/map_name type is invalid");
                return false;
            }

            map_folder = params[0].as_string();
            map_name = params[1].as_string();
            return !map_folder.empty() && !map_name.empty();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "GetMapInfoFromMapManager exception: %s", e.what());
            return false;
        }
    }

    bool ValidateMapResources() const {
        if (FLAGS_map_folder.empty() || FLAGS_map_name.empty()) {
            return false;
        }
        // ResolveMapDir 优先 slam/ 分层布局，回退旧扁平布局
        const std::filesystem::path map_dir = ResolveMapDir(FLAGS_map_folder, FLAGS_map_name);
        return std::filesystem::exists(map_dir / "global.pcd");
    }

    bool StartLocalizationInternal() {
        if (state_ == LocalizationState::RUNNING) {
            return true;
        }

        state_ = LocalizationState::STARTING;

        if (!ValidateMapResources()) {
            RCLCPP_ERROR(get_logger(), "Invalid map resources: %s/%s", FLAGS_map_folder.c_str(),
                         FLAGS_map_name.c_str());
            state_ = LocalizationState::ERROR;
            return false;
        }

        const std::string map_path = ResolveMapDir(FLAGS_map_folder, FLAGS_map_name) + "/";

        LocSystem::Options opt;
        auto yaml = YAML::LoadFile(FLAGS_config);
        if (yaml["system"] && yaml["system"]["pub_tf"]) {
            opt.pub_tf_ = yaml["system"]["pub_tf"].as<bool>();
        }
        if (yaml["system"] && yaml["system"]["with_rviz"]) {
            opt.with_rviz_ = yaml["system"]["with_rviz"].as<bool>();
        }
        loc_system_ = std::make_unique<LocSystem>(opt);
        if (!loc_system_->Init(FLAGS_config, map_path)) {
            loc_system_.reset();
            state_ = LocalizationState::ERROR;
            return false;
        }

        if (init_pose_set_) {
            loc_system_->SetInitPose(init_pose_);
        } else {
            loc_system_->SetInitPose(SE3());
        }
        loc_system_->StartLocalization();

        loc_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        loc_executor_->add_node(loc_system_->GetNode());
        loc_thread_ = std::thread([this]() { loc_executor_->spin(); });

        state_ = LocalizationState::RUNNING;
        return true;
    }

    void StopLocalizationInternal() {
        if (state_ == LocalizationState::STOPPED) {
            return;
        }

        state_ = LocalizationState::STOPPING;

        if (loc_executor_) {
            loc_executor_->cancel();
        }
        if (loc_thread_.joinable()) {
            loc_thread_.join();
        }
        loc_executor_.reset();

        if (loc_system_) {
            loc_system_->StopLocalization();
        }
        loc_system_.reset();

        state_ = LocalizationState::STOPPED;
    }

    void HandleSetMapName(const std::shared_ptr<nav_interfaces::srv::SetMapName::Request> req,
                          std::shared_ptr<nav_interfaces::srv::SetMapName::Response> res) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (state_ == LocalizationState::RUNNING || state_ == LocalizationState::STARTING) {
            res->success = false;
            return;
        }

        FLAGS_map_name = req->map_name;
        res->success = !FLAGS_map_name.empty();
    }

    void HandleSetInitPose(const std::shared_ptr<nav_interfaces::srv::SetPose::Request> req,
                           std::shared_ptr<nav_interfaces::srv::SetPose::Response> /*res*/) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        const auto& p = req->pose.pose.pose.position;
        const auto& q_msg = req->pose.pose.pose.orientation;
        Quatd q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        if (q.norm() < 1e-6) {
            q = Quatd::Identity();
        } else {
            q.normalize();
        }

        init_pose_ = SE3(q, Vec3d(p.x, p.y, p.z));
        init_pose_set_ = true;

        if (loc_system_ && state_ == LocalizationState::RUNNING) {
            loc_system_->SetInitPose(init_pose_);
        }
    }

    void HandleStartLocalization(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const bool ok = StartLocalizationInternal();
        res->success = ok;
        res->message = ok ? "Localization started" : "Failed to start localization";
    }

    void HandleStopLocalization(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        StopLocalizationInternal();
        res->success = true;
        res->message = "Localization stopped";
    }

    void HandleResetLocalization(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        StopLocalizationInternal();
        const bool ok = StartLocalizationInternal();
        res->success = ok;
        res->message = ok ? "Localization reset" : "Failed to reset localization";
    }

    void HandleSetMapFromManager(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        std::string folder;
        std::string name;
        if (!GetMapInfoFromMapManager(folder, name)) {
            res->success = false;
            res->message = "Failed to get map from map_manager";
            return;
        }

        const bool was_running = (state_ == LocalizationState::RUNNING);
        if (was_running) {
            StopLocalizationInternal();
        }

        FLAGS_map_folder = folder;
        FLAGS_map_name = name;

        bool ok = true;
        if (was_running) {
            ok = StartLocalizationInternal();
        }

        res->success = ok;
        res->message = ok ? "Map updated from map_manager" : "Map updated but restart localization failed";
    }

    void HandleGetLocalizationStatus(const std::shared_ptr<nav_interfaces::srv::GetStatus::Request> /*req*/,
                                     std::shared_ptr<nav_interfaces::srv::GetStatus::Response> res) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        res->success = true;
        res->name = "localization";
        res->status = StateToString(state_);
    }

    void HandleGetMapFolder(const std::shared_ptr<nav_interfaces::srv::GetString::Request> /*req*/,
                            std::shared_ptr<nav_interfaces::srv::GetString::Response> res) {
        res->success = true;
        res->data = FLAGS_map_folder;
    }

    void HandleGetMapName(const std::shared_ptr<nav_interfaces::srv::GetString::Request> /*req*/,
                          std::shared_ptr<nav_interfaces::srv::GetString::Response> res) {
        res->success = true;
        res->data = FLAGS_map_name;
    }

    void PublishStatus() {
        std_msgs::msg::Int16 msg;
        switch (state_) {
            case LocalizationState::RUNNING:
                msg.data = 1;
                break;
            case LocalizationState::ERROR:
                msg.data = 0;
                break;
            default:
                msg.data = -1;
                break;
        }
        localization_status_pub_->publish(msg);
    }

   private:
    LocalizationState state_{LocalizationState::STOPPED};
    std::mutex state_mutex_;

    std::unique_ptr<LocSystem> loc_system_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> loc_executor_;
    std::thread loc_thread_;

    bool init_pose_set_{false};
    SE3 init_pose_ = SE3();

    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr localization_status_pub_;

    rclcpp::Service<nav_interfaces::srv::SetMapName>::SharedPtr set_map_name_srv_;
    rclcpp::Service<nav_interfaces::srv::SetPose>::SharedPtr set_init_pose_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_localization_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_localization_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_localization_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_map_from_manager_srv_;
    rclcpp::Service<nav_interfaces::srv::GetStatus>::SharedPtr get_localization_status_srv_;
    rclcpp::Service<nav_interfaces::srv::GetString>::SharedPtr get_map_folder_srv_;
    rclcpp::Service<nav_interfaces::srv::GetString>::SharedPtr get_map_name_srv_;
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
    FLAGS_map_folder = lightning::ResolveMapsRoot(FLAGS_map_folder);

    auto node = std::make_shared<lightning::LocalizationManagerNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}