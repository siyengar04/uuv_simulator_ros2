// message_to_tf.cpp
// Converted from ROS1 to ROS2 by example.
// License: Apache-2.0

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

class MessageToTFNode : public rclcpp::Node
{
public:
  MessageToTFNode()
  : Node("message_to_tf")
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("odometry_topic", "");
    this->declare_parameter<std::string>("pose_topic", "");
    this->declare_parameter<std::string>("imu_topic", "");
    this->declare_parameter<std::string>("topic", "");
    this->declare_parameter<std::string>("frame_id", "");
    this->declare_parameter<std::string>("footprint_frame_id", "base_footprint");
    this->declare_parameter<std::string>("position_frame_id", "");
    this->declare_parameter<std::string>("stabilized_frame_id", "base_stabilized");
    this->declare_parameter<std::string>("child_frame_id", "");
    this->declare_parameter<bool>("publish_roll_pitch", true);
    this->declare_parameter<bool>("publish_pose", true);
    this->declare_parameter<std::string>("publish_pose_topic", "pose");
    this->declare_parameter<bool>("publish_euler", true);
    this->declare_parameter<std::string>("publish_euler_topic", "euler");

    this->get_parameter("odometry_topic", odometry_topic_);
    this->get_parameter("pose_topic", pose_topic_);
    this->get_parameter("imu_topic", imu_topic_);
    this->get_parameter("topic", topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("footprint_frame_id", footprint_frame_id_);
    this->get_parameter("position_frame_id", position_frame_id_);
    this->get_parameter("stabilized_frame_id", stabilized_frame_id_);
    this->get_parameter("child_frame_id", child_frame_id_);
    this->get_parameter("publish_roll_pitch", publish_roll_pitch_);
    this->get_parameter("publish_pose", publish_pose_);
    this->get_parameter("publish_pose_topic", publish_pose_topic_);
    this->get_parameter("publish_euler", publish_euler_);
    this->get_parameter("publish_euler_topic", publish_euler_topic_);

    // Create the TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create publishers if needed
    if (publish_pose_) {
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(publish_pose_topic_, 10);
    }
    if (publish_euler_) {
      euler_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(publish_euler_topic_, 10);
    }

    // Create subscriptions based on provided topics.
    int subscribers = 0;
    if (!odometry_topic_.empty()) {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, 10,
        std::bind(&MessageToTFNode::odomCallback, this, _1));
      subscribers++;
    }
    if (!pose_topic_.empty()) {
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10,
        std::bind(&MessageToTFNode::poseCallback, this, _1));
      subscribers++;
    }
    if (!imu_topic_.empty()) {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&MessageToTFNode::imuCallback, this, _1));
      subscribers++;
    }
    // Note: The generic "topic" callback (as in ROS1's ShapeShifter) is not implemented in this conversion.

    if (subscribers == 0) {
      RCLCPP_FATAL(this->get_logger(),
        "No subscription topics were set. Please set at least one of odometry_topic, pose_topic, or imu_topic.");
      rclcpp::shutdown();
    }
    if (subscribers > 1) {
      RCLCPP_FATAL(this->get_logger(),
        "More than one of the parameters odometry_topic, pose_topic, and imu_topic are set. "
        "Please specify exactly one of them or adjust your configuration.");
      rclcpp::shutdown();
    }
  }

private:
  // Helper function to add a transform to a vector
  void addTransform(std::vector<geometry_msgs::msg::TransformStamped> &transforms,
                    const geometry_msgs::msg::TransformStamped &transform)
  {
    transforms.push_back(transform);
  }
  bool publish_pose_;
  std::string publish_pose_topic_;
  bool publish_euler_;
  std::string publish_euler_topic_;
  // This function replicates the logic from ROS1: create and send intermediate transforms.
  void sendTransform(const geometry_msgs::msg::Pose &pose,
                     const std_msgs::msg::Header &header,
                     const std::string &child_frame_override = "")
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = header.stamp;
    transform.header.frame_id = !frame_id_.empty() ? frame_id_ : header.frame_id;

    std::string effective_child_frame = child_frame_override;
    if (!child_frame_id_.empty()) {
      effective_child_frame = child_frame_id_;
    }
    if (effective_child_frame.empty()) {
      effective_child_frame = "base_link";
    }

    // Convert orientation to tf2::Quaternion
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // Position vector
    double px = pose.position.x;
    double py = pose.position.y;
    double pz = pose.position.z;

    // Optionally create intermediate transforms (position, footprint, stabilized)
    // Position intermediate transform
    if (!position_frame_id_.empty() && effective_child_frame != position_frame_id_) {
      geometry_msgs::msg::TransformStamped t_pos;
      t_pos.header = transform.header;
      t_pos.header.stamp = header.stamp;
      t_pos.child_frame_id = position_frame_id_;
      t_pos.transform.translation.x = px;
      t_pos.transform.translation.y = py;
      t_pos.transform.translation.z = pz;
      t_pos.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
      addTransform(transforms, t_pos);
    }

    // Footprint intermediate transform (x, y, yaw)
    if (!footprint_frame_id_.empty() && effective_child_frame != footprint_frame_id_) {
      geometry_msgs::msg::TransformStamped t_fp;
      t_fp.header = transform.header;
      t_fp.child_frame_id = footprint_frame_id_;
      t_fp.transform.translation.x = px;
      t_fp.transform.translation.y = py;
      t_fp.transform.translation.z = 0.0;
      tf2::Quaternion q_yaw;
      q_yaw.setRPY(0.0, 0.0, yaw);
      t_fp.transform.rotation = tf2::toMsg(q_yaw);
      addTransform(transforms, t_fp);

      // Reset yaw and position for next transform
      yaw = 0.0;
      px = 0.0;
      py = 0.0;
      transform.header.frame_id = footprint_frame_id_;
    }

    // Stabilized intermediate transform (z)
    if (!stabilized_frame_id_.empty() && effective_child_frame != stabilized_frame_id_) {
      geometry_msgs::msg::TransformStamped t_stab;
      t_stab.header = transform.header;
      t_stab.child_frame_id = stabilized_frame_id_;
      t_stab.transform.translation.x = 0.0;
      t_stab.transform.translation.y = 0.0;
      t_stab.transform.translation.z = pz;
      tf2::Quaternion q_identity;
      q_identity.setRPY(0, 0, 0);
      t_stab.transform.rotation = tf2::toMsg(q_identity);
      addTransform(transforms, t_stab);

      // Reset z for final transform
      pz = 0.0;
      transform.header.frame_id = stabilized_frame_id_;
    }

    // Final transform: base_link with roll and pitch if requested
    if (publish_roll_pitch_) {
      geometry_msgs::msg::TransformStamped t_final;
      t_final.header = transform.header;
      t_final.child_frame_id = effective_child_frame;
      t_final.transform.translation.x = px;
      t_final.transform.translation.y = py;
      t_final.transform.translation.z = pz;
      tf2::Quaternion q_final;
      q_final.setRPY(roll, pitch, yaw);
      t_final.transform.rotation = tf2::toMsg(q_final);
      addTransform(transforms, t_final);
    }

    // Broadcast all transforms
    tf_broadcaster_->sendTransform(transforms);

    // Publish pose message if enabled
    if (pose_publisher_) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = header;
      pose_msg.pose = pose;
      pose_publisher_->publish(pose_msg);
    }

    // Publish euler angles if enabled
    if (euler_publisher_) {
      geometry_msgs::msg::Vector3Stamped euler_msg;
      euler_msg.header = header;
      euler_msg.vector.x = roll;
      euler_msg.vector.y = pitch;
      euler_msg.vector.z = yaw;
      euler_publisher_->publish(euler_msg);
    }
  }

  // Callback for Odometry messages
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    sendTransform(msg->pose.pose, msg->header, msg->child_frame_id);
  }

  // Callback for PoseStamped messages
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    sendTransform(msg->pose, msg->header);
  }

  // Callback for Imu messages
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // For IMU, we compute roll and pitch only.
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // Create a new pose using only orientation (position zero)
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    tf2::Quaternion q_rp;
    q_rp.setRPY(roll, pitch, 0.0);
    pose.orientation = tf2::toMsg(q_rp);

    sendTransform(pose, msg->header);
  }

  // Member variables
  std::string odometry_topic_, pose_topic_, imu_topic_, topic_;
  std::string frame_id_, footprint_frame_id_, position_frame_id_, stabilized_frame_id_, child_frame_id_;
  bool publish_roll_pitch_;

  // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publish_pose_;
  // rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_publisher_;


  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MessageToTFNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
