#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class MessageToTfNode : public rclcpp::Node
{
public:
  MessageToTfNode()
      : Node("message_to_tf")
  {
    // Declare parameters
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
    this->declare_parameter<bool>("publish_euler", true);

    // Get parameters
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    pose_topic_ = this->get_parameter("pose_topic").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    topic_ = this->get_parameter("topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    footprint_frame_id_ = this->get_parameter("footprint_frame_id").as_string();
    position_frame_id_ = this->get_parameter("position_frame_id").as_string();
    stabilized_frame_id_ = this->get_parameter("stabilized_frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();
    publish_roll_pitch_ = this->get_parameter("publish_roll_pitch").as_bool();

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create subscribers based on parameters
    if (!odometry_topic_.empty())
    {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          odometry_topic_, 10, std::bind(&MessageToTfNode::odomCallback, this, std::placeholders::_1));
    }
    if (!pose_topic_.empty())
    {
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          pose_topic_, 10, std::bind(&MessageToTfNode::poseCallback, this, std::placeholders::_1));
    }
    if (!imu_topic_.empty())
    {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
          imu_topic_, 10, std::bind(&MessageToTfNode::imuCallback, this, std::placeholders::_1));
    }
    if (!topic_.empty())
    {
      multi_sub_ = this->create_subscription<topic_tools::msg::ShapeShifter>(
          topic_, 10, std::bind(&MessageToTfNode::multiCallback, this, std::placeholders::_1));
    }

    // Create publishers
    if (this->get_parameter("publish_pose").as_bool())
    {
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    }
    if (this->get_parameter("publish_euler").as_bool())
    {
      euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("euler", 10);
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    sendTransform(msg->pose.pose, msg->header, msg->child_frame_id);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    sendTransform(msg->pose, msg->header);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    tf2::Quaternion orientation;
    tf2::fromMsg(msg->orientation, orientation);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    tf2::Quaternion rollpitch = tf2::Quaternion();
    rollpitch.setRPY(roll, pitch, 0.0);

    // base_link transform (roll, pitch)
    if (publish_roll_pitch_)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = msg->header.stamp;
      tf.header.frame_id = stabilized_frame_id_;
      tf.child_frame_id = child_frame_id_.empty() ? "base_link" : child_frame_id_;
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = tf2::toMsg(rollpitch);
      transforms.push_back(tf);
    }

    if (!transforms.empty())
    {
      tf_broadcaster_->sendTransform(transforms);
    }

    // Publish pose message
    if (pose_pub_)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = msg->header.stamp;
      pose_stamped.header.frame_id = stabilized_frame_id_;
      pose_stamped.pose.orientation = tf2::toMsg(rollpitch);
      pose_pub_->publish(pose_stamped);
    }
  }

  void multiCallback(const topic_tools::msg::ShapeShifter::SharedPtr msg)
  {
    if (msg->get_type() == "nav_msgs/msg/Odometry")
    {
      auto odom = msg->instantiate<nav_msgs::msg::Odometry>();
      odomCallback(odom);
    }
    else if (msg->get_type() == "geometry_msgs/msg/PoseStamped")
    {
      auto pose = msg->instantiate<geometry_msgs::msg::PoseStamped>();
      poseCallback(pose);
    }
    else if (msg->get_type() == "sensor_msgs/msg/Imu")
    {
      auto imu = msg->instantiate<sensor_msgs::msg::Imu>();
      imuCallback(imu);
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "message_to_tf received a %s message. Supported message types: nav_msgs/Odometry geometry_msgs/PoseStamped sensor_msgs/Imu",
          msg->get_type().c_str());
    }
  }

  void sendTransform(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header, const std::string &child_frame_id = "")
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = header.stamp;
    tf.header.frame_id = frame_id_.empty() ? header.frame_id : frame_id_;
    tf.child_frame_id = child_frame_id_.empty() ? child_frame_id : child_frame_id_;

    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    double yaw, pitch, roll;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);

    // Position intermediate transform (x, y, z)
    if (!position_frame_id_.empty() && child_frame_id != position_frame_id_)
    {
      tf.child_frame_id = position_frame_id_;
      tf.transform.translation.x = position.x();
      tf.transform.translation.y = position.y();
      tf.transform.translation.z = position.z();
      tf.transform.rotation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      transforms.push_back(tf);
    }

    // Footprint intermediate transform (x, y, yaw)
    if (!footprint_frame_id_.empty() && child_frame_id != footprint_frame_id_)
    {
      tf.child_frame_id = footprint_frame_id_;
      tf.transform.translation.x = position.x();
      tf.transform.translation.y = position.y();
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, yaw));
      transforms.push_back(tf);

      yaw = 0.0;
      position.setX(0.0);
      position.setY(0.0);
      tf.header.frame_id = footprint_frame_id_;
    }

    // Stabilized intermediate transform (z)
    if (!stabilized_frame_id_.empty() && child_frame_id != stabilized_frame_id_)
    {
      tf.child_frame_id = stabilized_frame_id_;
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = position.z();
      tf.transform.rotation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      transforms.push_back(tf);

      position.setZ(0.0);
      tf.header.frame_id = stabilized_frame_id_;
    }

    // Base_link transform (roll, pitch)
    if (publish_roll_pitch_)
    {
      tf.child_frame_id = child_frame_id_.empty() ? "base_link" : child_frame_id_;
      tf.transform.translation.x = position.x();
      tf.transform.translation.y = position.y();
      tf.transform.translation.z = position.z();
      tf.transform.rotation = tf2::toMsg(tf2::Quaternion(roll, pitch, yaw));
      transforms.push_back(tf);
    }

    tf_broadcaster_->sendTransform(transforms);

    // Publish pose message
    if (pose_pub_)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose = pose;
      pose_stamped.header = header;
      pose_pub_->publish(pose_stamped);
    }

    // Publish Euler angles
    if (euler_pub_)
    {
      geometry_msgs::msg::Vector3Stamped euler_stamped;
      euler_stamped.vector.x = roll;
      euler_stamped.vector.y = pitch;
      euler_stamped.vector.z = yaw;
      euler_stamped.header = header;
      euler_pub_->publish(euler_stamped);
    }
  }

  // Parameters
  std::string odometry_topic_;
  std::string pose_topic_;
  std::string imu_topic_;
  std::string topic_;
  std::string frame_id_;
  std::string footprint_frame_id_;
  std::string position_frame_id_;
  std::string stabilized_frame_id_;
  std::string child_frame_id_;
  bool publish_roll_pitch_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<topic_tools::msg::ShapeShifter>::SharedPtr multi_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;

  // TF Broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessageToTfNode>());
  rclcpp::shutdown();
  return 0;
}