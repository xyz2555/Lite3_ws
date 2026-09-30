#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

class Lite3BasePoseBridge : public rclcpp::Node
{
public:
  Lite3BasePoseBridge()
  : Node("lite3_base_pose_bridge")
  {
    publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lite3/base_pose", 20);

    const std::string topic =
      "/world/flat/dynamic_pose/info";

    const bool subscribed =
      gz_node_.Subscribe(
        topic,
        &Lite3BasePoseBridge::poseCallback,
        this);

    if (!subscribed) {
      throw std::runtime_error(
        "Failed to subscribe to " + topic);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed to %s",
      topic.c_str());
  }

private:
  void poseCallback(const gz::msgs::Pose_V &msg)
  {
    for (const auto &pose : msg.pose()) {

      if (pose.name() != "lite3") {
        continue;
      }

      geometry_msgs::msg::PoseStamped out;

      /*
       * Use Gazebo simulation timestamp directly.
       */
      if (msg.has_header() && msg.header().has_stamp()) {
        out.header.stamp.sec =
          msg.header().stamp().sec();

        out.header.stamp.nanosec =
          static_cast<uint32_t>(
            msg.header().stamp().nsec());
      } else {
        out.header.stamp =
          this->get_clock()->now();
      }

      out.header.frame_id = "world";

      out.pose.position.x = pose.position().x();
      out.pose.position.y = pose.position().y();
      out.pose.position.z = pose.position().z();

      out.pose.orientation.x =
        pose.orientation().x();

      out.pose.orientation.y =
        pose.orientation().y();

      out.pose.orientation.z =
        pose.orientation().z();

      out.pose.orientation.w =
        pose.orientation().w();

      publisher_->publish(out);

      return;
    }
  }

  gz::transport::Node gz_node_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped
  >::SharedPtr publisher_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<Lite3BasePoseBridge>();

    rclcpp::spin(node);
  }
  catch (const std::exception &e) {
    std::cerr
      << "Fatal error: "
      << e.what()
      << std::endl;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}