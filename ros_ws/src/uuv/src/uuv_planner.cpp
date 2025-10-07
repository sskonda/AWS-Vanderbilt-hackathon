#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <array>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "custom_msg/msg/event.hpp"

using namespace std::chrono_literals;

// enum
enum class MaintenanceStatus { WORKING, DUE_FOR_MAINTENANCE, FAILING };
enum class PipelineStatus { GOOD, WARNING, CRITICAL_FAILURE };
enum class Role { BATTERY_TANKER, PATROL, MASTER, SLAVE };

// sub
class SubNode : public rclcpp::Node
{
public:
  explicit SubNode(const std::string &sub_id)
  : Node(sub_id + "_node"),
    sub_id_(sub_id)
  {
    // init
    patrol_route_id_ = "route1";
    route_start_ = {0.0, 0.0, -20.0};
    route_end_   = {200.0, 100.0, -20.0};
    route_estimated_time_ = 3600.0; // 1 hour route

    maintenance_status_ = MaintenanceStatus::WORKING;
    pipeline_status_ = PipelineStatus::GOOD;
    current_role_ = Role::PATROL;
    battery_ = 100.0;

    last_surfaced_ = nowSec();
    next_scheduled_surface_ = last_surfaced_ + 600.0; // every 10 min
    assigned_grace_period_ = 120.0; // 2 min
    local_tile_id_ = "tile5";

    // ros pubs/ subs
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + sub_id_ + "/pose", 10);

    event_sub_ = create_subscription<custom_msg::msg::Event>(
      "/events/event_1", 10,
      std::bind(&SubNode::onEvent, this, std::placeholders::_1));

    // Timer for periodic updates
    timer_ = create_wall_timer(500ms, std::bind(&SubNode::update_state, this));

    RCLCPP_INFO(get_logger(),
      "[%s] Node initialized | Route: %s | Tile: %s | Role: %s",
      sub_id_.c_str(), patrol_route_id_.c_str(), local_tile_id_.c_str(),
      roleToStr(current_role_).c_str());
  }

private:
  // variables 
  std::string sub_id_;
  std::string patrol_route_id_;
  std::array<double, 3> route_start_;
  std::array<double, 3> route_end_;
  double route_estimated_time_;
  double battery_;
  MaintenanceStatus maintenance_status_;
  PipelineStatus pipeline_status_;
  Role current_role_;
  double last_surfaced_;
  double next_scheduled_surface_;
  double assigned_grace_period_;
  std::string local_tile_id_;

  // ros
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<custom_msg::msg::Event>::SharedPtr event_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static double nowSec()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static std::string roleToStr(Role r)
  {
    switch (r) {
      case Role::BATTERY_TANKER: return "BATTERY_TANKER";
      case Role::PATROL: return "PATROL";
      case Role::MASTER: return "MASTER";
      case Role::SLAVE: return "SLAVE";
    }
    return "UNKNOWN";
  }

  static std::string maintToStr(MaintenanceStatus s)
  {
    switch (s) {
      case MaintenanceStatus::WORKING: return "WORKING";
      case MaintenanceStatus::DUE_FOR_MAINTENANCE: return "DUE_FOR_MAINTENANCE";
      case MaintenanceStatus::FAILING: return "FAILING";
    }
    return "UNKNOWN";
  }

  static std::string pipeToStr(PipelineStatus s)
  {
    switch (s) {
      case PipelineStatus::GOOD: return "GOOD";
      case PipelineStatus::WARNING: return "WARNING";
      case PipelineStatus::CRITICAL_FAILURE: return "CRITICAL_FAILURE";
    }
    return "UNKNOWN";
  }

  // status
  void update_state()
  {
    double elapsed = fmod(nowSec() - last_surfaced_, route_estimated_time_);
    double t = elapsed / route_estimated_time_;

    // Calculate linear interpolation along route
    double x = route_start_[0] + t * (route_end_[0] - route_start_[0]);
    double y = route_start_[1] + t * (route_end_[1] - route_start_[1]);
    double z = route_start_[2];

    // pose stamped which will be output
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = sub_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    pose_pub_->publish(pose);

    // battery 
    battery_ -= 0.05; // slow drain
    if (battery_ < 30.0 && maintenance_status_ == MaintenanceStatus::WORKING)
      maintenance_status_ = MaintenanceStatus::DUE_FOR_MAINTENANCE;
    if (battery_ < 10.0)
      pipeline_status_ = PipelineStatus::WARNING;
    if (battery_ <= 0.0)
    {
      pipeline_status_ = PipelineStatus::CRITICAL_FAILURE;
      battery_ = 0.0;
    }

    // whether surface 
    if (nowSec() >= next_scheduled_surface_)
    {
      last_surfaced_ = nowSec();
      next_scheduled_surface_ = last_surfaced_ + 600.0;
      battery_ = std::min(100.0, battery_ + 25.0);
      RCLCPP_INFO(get_logger(), "[%s] Surfaced | Battery %.1f%% | Maintenance=%s",
        sub_id_.c_str(), battery_, maintToStr(maintenance_status_).c_str());
    }

    if ((rand() % 1000) == 1)
    {
      maintenance_status_ = MaintenanceStatus::FAILING;
      RCLCPP_WARN(get_logger(), "[%s] Random subsystem failure detected!", sub_id_.c_str());
    }

    RCLCPP_DEBUG(get_logger(),
      "[%s] Pose(%.1f,%.1f,%.1f) | Batt=%.1f%% | Maint=%s | Pipeline=%s",
      sub_id_.c_str(), x, y, z, battery_,
      maintToStr(maintenance_status_).c_str(), pipeToStr(pipeline_status_).c_str());
  }

  void onEvent(const custom_msg::msg::Event &evt)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] Received Event: type=%s | msg=%s | from frame=%s | pos=(%.1f,%.1f,%.1f)",
      sub_id_.c_str(),
      evt.event_type.c_str(),
      evt.message.c_str(),
      evt.header.frame_id.c_str(),
      evt.pose.position.x,
      evt.pose.position.y,
      evt.pose.position.z);

    if (evt.event_type == "PROXIMITY_HIT") {
      RCLCPP_WARN(get_logger(), "[%s] Avoiding region due to event proximity!", sub_id_.c_str());
      route_start_[0] += 5.0; // simple adjustment
      route_end_[0] += 5.0;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubNode>("sub1");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
