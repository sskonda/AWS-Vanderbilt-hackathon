#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class SubmarinePoseFilter : public rclcpp::Node
{
public:
  SubmarinePoseFilter()
  : Node("submarine_pose_filter")
  {
    sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/world/buoyancy/dynamic_pose/info", 10,
      std::bind(&SubmarinePoseFilter::pose_callback, this, std::placeholders::_1)
    );
    
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/submarines/pose", 10);
  }

private:
  void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped filtered_msg;
    for (const auto & transform : msg->transforms) {
      // Log message
      std::string name = transform.child_frame_id;
      if (name.substr(0, 3) == "sub") {
        RCLCPP_INFO(this->get_logger(), "Received transform of %s", name.c_str());
        filtered_msg.header = transform.header;
        filtered_msg.header.frame_id = transform.child_frame_id;
        filtered_msg.pose.position.x = transform.transform.translation.x;
        filtered_msg.pose.position.y = transform.transform.translation.y;
        filtered_msg.pose.position.z = transform.transform.translation.z;
        filtered_msg.pose.orientation = transform.transform.rotation;

        pub_->publish(filtered_msg);
      }
    }
  }

  std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> sub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubmarinePoseFilter>());
  rclcpp::shutdown();
  return 0;
}
