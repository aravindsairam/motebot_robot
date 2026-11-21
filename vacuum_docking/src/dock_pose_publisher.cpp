// Turns the AprilTag detector's sighting of the dock into the pose the docking server expects.
//
//   in:   /tf                   camera_optical_frame -> dock_tag, sent by apriltag_ros while the tag is seen
//   out:  /detected_dock_pose   geometry_msgs/PoseStamped in the camera frame, stamped with the image time
//
// The detector describes the tag with z pointing out of it and y pointing down. The docking
// server wants a pose it can read a heading from. So the pose published here sits at the tag
// but is turned: its x axis points INTO the dock (the way the robot faces when docked) and its
// z axis points up. The docking server then puts the robot a set distance in front of it.
//
// Nothing is published while the tag is not seen; the docking server treats silence as "lost".

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace vacuum_docking
{

class DockPosePublisher : public rclcpp::Node
{
public:
  DockPosePublisher()
  : Node("dock_pose_publisher")
  {
    tag_frame_ = declare_parameter("tag_frame", std::string("dock_tag"));

    // The dock frame's axes written in the tag's frame: x = into the tag (-z of the tag),
    // y = the tag's x, z = up (-y of the tag).
    tf2::Matrix3x3(
      0.0, 1.0, 0.0,
      0.0, 0.0, -1.0,
      -1.0, 0.0, 0.0).getRotation(tag_to_dock_);

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", rclcpp::QoS(1));
    sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", rclcpp::QoS(100), [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for (const auto & t : msg->transforms) {
          if (t.child_frame_id == tag_frame_) {
            publish(t);
          }
        }
      });
    RCLCPP_INFO(get_logger(), "publishing the dock pose from sightings of '%s'", tag_frame_.c_str());
  }

private:
  void publish(const geometry_msgs::msg::TransformStamped & tag)
  {
    const auto & r = tag.transform.rotation;
    const tf2::Quaternion camera_to_tag(r.x, r.y, r.z, r.w);
    const tf2::Quaternion q = (camera_to_tag * tag_to_dock_).normalized();

    geometry_msgs::msg::PoseStamped pose;
    pose.header = tag.header;  // the camera frame and the time the image was taken
    pose.pose.position.x = tag.transform.translation.x;
    pose.pose.position.y = tag.transform.translation.y;
    pose.pose.position.z = tag.transform.translation.z;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pub_->publish(pose);
  }

  std::string tag_frame_;
  tf2::Quaternion tag_to_dock_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
};

}  // namespace vacuum_docking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vacuum_docking::DockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
