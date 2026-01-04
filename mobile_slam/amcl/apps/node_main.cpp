#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "nav_interfaces/srv/get_string.hpp"

#include "amcl_node.hpp"

std::shared_ptr<amcl::AmclNode> amcl_node_ptr;

bool is_activating = false;

bool StartFilter()
{
  // ROS2 service client
  auto client = amcl_node_ptr->create_client<std_srvs::srv::Trigger>("/robot_localization/start_filter");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(amcl_node_ptr->get_logger(), "Service %s not available", client->get_service_name());
    return false;
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(amcl_node_ptr, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(amcl_node_ptr->get_logger(), "%s call success", client->get_service_name());
    return true;
  } else {
    RCLCPP_ERROR(amcl_node_ptr->get_logger(), "%s call failed", client->get_service_name());
    return false;
  }

  return false;
}

bool StopFilter()
{
  auto client = amcl_node_ptr->create_client<std_srvs::srv::Trigger>("/robot_localization/stop_filter");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(amcl_node_ptr->get_logger(), "Service %s not available", client->get_service_name());
    return false;
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(amcl_node_ptr, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(amcl_node_ptr->get_logger(), "%s call success", client->get_service_name());
    return true;
  } else {
    RCLCPP_ERROR(amcl_node_ptr->get_logger(), "%s call failed", client->get_service_name());
    return false;
  }

  return false;
}

bool ResetFilter()
{
  auto client = amcl_node_ptr->create_client<std_srvs::srv::Trigger>("/robot_localization/reset_filter");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(amcl_node_ptr->get_logger(), "Service %s not available", client->get_service_name());
    return false;
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(amcl_node_ptr, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(amcl_node_ptr->get_logger(), "%s call success", client->get_service_name());
    return true;
  } else {
    RCLCPP_ERROR(amcl_node_ptr->get_logger(), "%s call failed", client->get_service_name());
    return false;
  }

  return false;
}

bool StartLocalizationCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;  // unused parameter
  if (is_activating) {
    RCLCPP_WARN(rclcpp::get_logger("amcl"), "Localization is already activated");
    res->success = true;
    return true;
  }

  StartFilter();
  amcl_node_ptr.reset(new amcl::AmclNode());

  res->success = (amcl_node_ptr != nullptr);
  is_activating = true;
  if (!res->success) {
    RCLCPP_ERROR(rclcpp::get_logger("amcl"), "Start localization failed");
  }
  return res->success;
}

bool StopLocalizationCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;  // unused parameter
  if (!is_activating) {
    RCLCPP_WARN(rclcpp::get_logger("amcl"), "Localization is already deactivated");
    res->success = true;
    return true;
  }

  amcl_node_ptr.reset();
  StopFilter();

  is_activating = false;

  res->success = true;
  return res->success;
}

bool ResetLocalizationCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;  // unused parameter
  RCLCPP_INFO(rclcpp::get_logger("amcl"), "Reset localization");
  is_activating = false;
  amcl_node_ptr.reset();
  ResetFilter();
  amcl_node_ptr.reset(new amcl::AmclNode());
  is_activating = true;
  res->success = (amcl_node_ptr != nullptr);
  if (!res->success) {
    RCLCPP_ERROR(rclcpp::get_logger("amcl"), "Start localization failed");
  }
  return res->success;
}

bool GetActivationStatusCallback(const std::shared_ptr<nav_interfaces::srv::GetString::Request> req,
                                 std::shared_ptr<nav_interfaces::srv::GetString::Response> res)
{
  (void)req;  // unused parameter
  res->data = is_activating ? "activated" : "deactivated";
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = rclcpp::Node::make_shared("amcl_lifecycle");

  auto start_localization_srv = node->create_service<std_srvs::srv::Trigger>(
    "~/start_localization", StartLocalizationCallback);
  auto stop_localization_srv = node->create_service<std_srvs::srv::Trigger>(
    "~/stop_localization", StopLocalizationCallback);
  auto reset_localization_srv = node->create_service<std_srvs::srv::Trigger>(
    "~/reset_localization", ResetLocalizationCallback);
  auto get_activation_status_srv = node->create_service<nav_interfaces::srv::GetString>(
    "~/get_activation_status", GetActivationStatusCallback);

  amcl_node_ptr.reset(new amcl::AmclNode());
  is_activating = true;

  // Create a multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(amcl_node_ptr);
  executor.spin();

  amcl_node_ptr.reset();
  rclcpp::shutdown();

  return 0;
}
