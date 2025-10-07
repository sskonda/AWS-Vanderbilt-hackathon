#include <rclcpp/rclcpp.hpp>
#include <ros_ign_interfaces/msg/entity.hpp>

// class SubmarinePoseFilter : public rclcpp::Node
// {
// public:
//   SubmarinePoseFilter()
//   : Node("submarine_pose_filter")
//   {
//     sub_ = this->create_subscription<ros_ign_interfaces::msg::EntityPose>(
//       "/world/buoyancy/pose/info", 10,
//       std::bind(&SubmarinePoseFilter::pose_callback, this, std::placeholders::_1));
//   }

// private:
//   void pose_callback(const ros_ign_interfaces::msg::EntityPose::SharedPtr msg)
//   {
//     for (size_t i = 0; i < msg->name.size(); ++i)
//     {
//       if (msg->name[i] == "submarine")
//       {
//         const auto & pose = msg->pose[i];
//         RCLCPP_INFO(this->get_logger(),
//           "Submarine pose: position=(%.2f, %.2f, %.2f) orientation=(%.2f, %.2f, %.2f, %.2f)",
//           pose.position.x, pose.position.y, pose.position.z,
//           pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
//       }
//     }
//   }

//   rclcpp::Subscription<ros_ign_interfaces::msg::EntityPose>::SharedPtr sub_;
// };

int main(int argc, char ** argv)
{
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<SubmarinePoseFilter>());
  // rclcpp::shutdown();
  return 0;
}
