#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "custom_msg/msg/event.hpp"

using namespace std::chrono_literals;

enum class MaintenanceStatus { WORKING, DUE_FOR_MAINTENANCE, FAILING };
enum class PipelineStatus { GOOD, WARNING, CRITICAL_FAILURE };
enum class Role { PATROL, MID_TIER };
enum class BehaviorState { IDLE, PATROL, SURFACE, CHARGE, FAILSAFE };

class SubNode : public rclcpp::Node
{
public:
  explicit SubNode(const std::string &sub_id)
  : Node(sub_id + "_node"), sub_id_(sub_id)
  {
    this->declare_parameter<std::string>("role", "PATROL");
    this->declare_parameter<std::vector<std::string>>("team_ids", {"subA","subB","subC"});

    std::string role_str = this->get_parameter("role").as_string();
    team_ids_ = this->get_parameter("team_ids").as_string_array();

    if (role_str == "PATROL") current_role_ = Role::PATROL;
    else if (role_str == "MID_TIER") current_role_ = Role::MID_TIER;
    else RCLCPP_WARN(get_logger(), "Unknown role '%s', defaulting to PATROL", role_str.c_str());

    patrol_route_id_ = "route1";
    route_start_ = {0.0, 0.0, -20.0};
    route_end_   = {200.0, 100.0, -20.0};
    route_estimated_time_ = 360.0;

    maintenance_status_ = MaintenanceStatus::WORKING;
    pipeline_status_ = PipelineStatus::GOOD;
    behavior_state_ = BehaviorState::IDLE;

    battery_ = 100.0;
    last_surfaced_ = nowSec();
    next_scheduled_surface_ = last_surfaced_ + 180.0;
    assigned_grace_period_ = 40.0;
    local_tile_id_ = "tile5";

    last_rotation_time_ = nowSec();
    rotation_interval_s_ = 360.0;  // 20 min (surfacing + buffer)

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + sub_id_ + "/pose", 10);
    event_pub_ = create_publisher<custom_msg::msg::Event>(
      "/events/status", 10);
    event_sub_ = create_subscription<custom_msg::msg::Event>(
      "/events/event_1", 10, std::bind(&SubNode::onEvent, this, std::placeholders::_1));


    // main loop timer 
    timer_ = create_wall_timer(500ms, std::bind(&SubNode::tick, this));

    RCLCPP_INFO(get_logger(),
      "[%s] Node started | Role=%s | Route=%s | Tile=%s",
      sub_id_.c_str(), roleToStr(current_role_).c_str(),
      patrol_route_id_.c_str(), local_tile_id_.c_str());
  }

private:
// variable 
  std::string sub_id_;
  std::string patrol_route_id_;
  std::array<double, 3> route_start_;
  std::array<double, 3> route_end_;
  double route_estimated_time_;
  double battery_;
  MaintenanceStatus maintenance_status_;
  PipelineStatus pipeline_status_;
  Role current_role_;
  BehaviorState behavior_state_;
  double last_surfaced_;
  double next_scheduled_surface_;
  double assigned_grace_period_;
  std::string local_tile_id_;
  double last_rotation_time_;
  double rotation_interval_s_;
  std::vector<std::string> team_ids_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<custom_msg::msg::Event>::SharedPtr event_pub_;
  rclcpp::Subscription<custom_msg::msg::Event>::SharedPtr event_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static double nowSec()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  // helpers 
  static std::string roleToStr(Role r)
  {
    switch (r) {
      case Role::PATROL: return "PATROL";
      case Role::MID_TIER: return "MID_TIER";
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

  // main loop, what happens every tick
  void tick()
  {
    switch (current_role_) {
      case Role::PATROL:   patrolBehavior(); break;
      case Role::MID_TIER: midTierBehavior(); break;
    }
  }
 // patrol and mid tier behavior 
  void patrolBehavior()
  {
    switch (behavior_state_) {
      case BehaviorState::IDLE:
        if (battery_ > 20.0)
          behavior_state_ = BehaviorState::PATROL;
        break;

      case BehaviorState::PATROL:
        if (battery_ <= 10.0) behavior_state_ = BehaviorState::CHARGE;
        else if (nowSec() >= next_scheduled_surface_)
          behavior_state_ = BehaviorState::SURFACE;
        break;

      case BehaviorState::SURFACE:
        performSurfaceAndRecharge(10.0);
        behavior_state_ = BehaviorState::PATROL;
        break;

      case BehaviorState::CHARGE:
        performSurfaceAndRecharge(50.0);
        if (battery_ >= 80.0) behavior_state_ = BehaviorState::PATROL;
        break;

      case BehaviorState::FAILSAFE:
        if (battery_ > 30.0) behavior_state_ = BehaviorState::IDLE;
        break;
    }

    publishPatrolPose(-20.0);  // lower altitude
    applyResourceModel();
    publishStatusToMidTier();
  }

  void midTierBehavior()
  {
    switch (behavior_state_) {
      case BehaviorState::IDLE:
        behavior_state_ = BehaviorState::PATROL;
        break;

      case BehaviorState::PATROL:
        publishPatrolPose(-10.0);  // higher altitude
        if (nowSec() >= next_scheduled_surface_)
          behavior_state_ = BehaviorState::SURFACE;
        break;

      case BehaviorState::SURFACE:
        performSurfaceAndRecharge(35.0);  // solar recharge
        behavior_state_ = BehaviorState::PATROL;
        break;

      case BehaviorState::CHARGE:
        performSurfaceAndRecharge(50.0);
        if (battery_ >= 90.0)
          behavior_state_ = BehaviorState::PATROL;
        break;

      case BehaviorState::FAILSAFE:
        if (battery_ > 30.0)
          behavior_state_ = BehaviorState::PATROL;
        break;
    }

    applyResourceModel();
  }

  // publish the pose of the patrols
  void publishPatrolPose(double z_depth)
  {
    const double elapsed = std::fmod(nowSec() - last_surfaced_, route_estimated_time_);
    const double t = elapsed / route_estimated_time_;

    const double x = route_start_[0] + t * (route_end_[0] - route_start_[0]);
    const double y = route_start_[1] + t * (route_end_[1] - route_start_[1]);
    const double z = z_depth;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = sub_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    pose_pub_->publish(pose);
  }


  void applyResourceModel() // whether it needs to maintanance or charge 
  {
    if (behavior_state_ != BehaviorState::CHARGE)
      battery_ = std::max(0.0, battery_ - 0.05);

    if (battery_ < 30.0 && maintenance_status_ == MaintenanceStatus::WORKING)
      maintenance_status_ = MaintenanceStatus::DUE_FOR_MAINTENANCE;
    if (battery_ < 10.0)
      pipeline_status_ = PipelineStatus::WARNING;
    if (battery_ <= 0.0) {
      pipeline_status_ = PipelineStatus::CRITICAL_FAILURE;
      behavior_state_  = BehaviorState::FAILSAFE;
    }
  }

  // mid tier surface, supposed to transmit and then recharge 
  void performSurfaceAndRecharge(double recharge_amount)
  {
    last_surfaced_ = nowSec();
    next_scheduled_surface_ = last_surfaced_ + 600.0;
    battery_ = std::min(100.0, battery_ + recharge_amount);

    RCLCPP_INFO(get_logger(),
      "[%s] SURFACED | battery=%.1f%% | maint=%s | role=%s",
      sub_id_.c_str(),
      battery_,
      maintToStr(maintenance_status_).c_str(),
      roleToStr(current_role_).c_str());

    maybeRotateRole();
  }

  // rotate roles 
  void maybeRotateRole()
  {
    double now_t = nowSec();
    if (now_t - last_rotation_time_ < rotation_interval_s_)
      return;

    last_rotation_time_ = now_t;

    int idx = std::distance(team_ids_.begin(),
                std::find(team_ids_.begin(), team_ids_.end(), sub_id_));
    int next_idx = (idx + 1) % team_ids_.size();

    // round robin 
    if (current_role_ == Role::PATROL && idx == 0)
      current_role_ = Role::MID_TIER;
    else if (current_role_ == Role::MID_TIER)
      current_role_ = Role::PATROL;

    RCLCPP_INFO(get_logger(),
      "[%s] Role rotation -> %s (next rotation target: %s)",
      sub_id_.c_str(),
      roleToStr(current_role_).c_str(),
      team_ids_[next_idx].c_str());
  }

  void publishStatusToMidTier()
  {
    custom_msg::msg::Event evt;
    evt.header.stamp = now();
    evt.header.frame_id = sub_id_;
    evt.event_type = "STATUS_UPDATE";
    evt.message = "Battery=" + std::to_string(battery_) +
                  ", Maint=" + maintToStr(maintenance_status_) +
                  ", Pipeline=" + pipeToStr(pipeline_status_);
    evt.pose.position.x = 0.0;
    evt.pose.position.y = 0.0;
    evt.pose.position.z = 0.0;

    event_pub_->publish(evt);
  }

  void onEvent(const custom_msg::msg::Event &evt)
  { // use custom event message 
    RCLCPP_INFO(get_logger(),
      "[%s] Event: %s | msg=%s | frame=%s",
      sub_id_.c_str(),
      evt.event_type.c_str(),
      evt.message.c_str(),
      evt.header.frame_id.c_str());

    if (evt.event_type == "PROXIMITY_HIT") {
      route_start_[0] += 5.0;
      route_end_[0] += 5.0;
      RCLCPP_WARN(get_logger(), "[%s] Adjusted course to avoid event region", sub_id_.c_str());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubNode>("subA");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
