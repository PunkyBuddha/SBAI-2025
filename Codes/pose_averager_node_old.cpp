#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <vector>
#include <cmath>
#include <string>

class PoseAveragerNode {
public:
  PoseAveragerNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
    private_nh.param<std::string>("subscribe_topic", subscribe_topic_, "/natnet_client_node/B1/pose");
    private_nh.param<std::string>("publish_topic", publish_topic_, "/natnet_client_node/B1/pose_averaged");
    private_nh.param<int>("window_size", window_size_, 6);

    sub_ = nh.subscribe(subscribe_topic_, 1000, &PoseAveragerNode::poseCallback, this);
    pub_ = nh.advertise<geometry_msgs::PoseStamped>(publish_topic_, 10);
    vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>(publish_topic_ + "_velocity", 10);
    acc_pub_ = nh.advertise<geometry_msgs::AccelStamped>(publish_topic_ + "_acceleration", 10);

    ROS_INFO_STREAM("Subscribed to topic: " << subscribe_topic_);
    ROS_INFO_STREAM("Publishing averaged pose on: " << publish_topic_ 
                    << " with a moving average window size of " << window_size_);
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher vel_pub_;
  ros::Publisher acc_pub_;
  std::string subscribe_topic_;
  std::string publish_topic_;
  int window_size_;
  std::vector<geometry_msgs::PoseStamped> buffer_;

  bool first_avg_ = true;
  geometry_msgs::PoseStamped last_avg_pose_;

  bool first_velocity_ = true;
  geometry_msgs::TwistStamped last_velocity_;

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    buffer_.push_back(*msg);
    if (buffer_.size() > static_cast<size_t>(window_size_)) {
      buffer_.erase(buffer_.begin());
    }

    geometry_msgs::PoseStamped avg_pose;
    avg_pose.header.stamp = msg->header.stamp;
    avg_pose.header.frame_id = msg->header.frame_id;

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    double sum_qx = 0.0, sum_qy = 0.0, sum_qz = 0.0, sum_qw = 0.0;
    
    for (const auto &p : buffer_) {
      sum_x += p.pose.position.x;
      sum_y += p.pose.position.y;
      sum_z += p.pose.position.z;
      sum_qx += p.pose.orientation.x;
      sum_qy += p.pose.orientation.y;
      sum_qz += p.pose.orientation.z;
      sum_qw += p.pose.orientation.w;
    }
    
    size_t n = buffer_.size();
    avg_pose.pose.position.x = sum_x / n;
    avg_pose.pose.position.y = sum_y / n;
    avg_pose.pose.position.z = sum_z / n;

    double avg_qx = sum_qx / n;
    double avg_qy = sum_qy / n;
    double avg_qz = sum_qz / n;
    double avg_qw = sum_qw / n;
    
    double norm = std::sqrt(avg_qx * avg_qx + avg_qy * avg_qy + avg_qz * avg_qz + avg_qw * avg_qw);
    if (norm > 0.0) {
      avg_pose.pose.orientation.x = avg_qx / norm;
      avg_pose.pose.orientation.y = avg_qy / norm;
      avg_pose.pose.orientation.z = avg_qz / norm;
      avg_pose.pose.orientation.w = avg_qw / norm;
    } else {
      avg_pose.pose.orientation.x = 0.0;
      avg_pose.pose.orientation.y = 0.0;
      avg_pose.pose.orientation.z = 0.0;
      avg_pose.pose.orientation.w = 1.0;
    }

    pub_.publish(avg_pose);

    geometry_msgs::TwistStamped current_velocity;
    if (!first_avg_) {
      double dt = (avg_pose.header.stamp - last_avg_pose_.header.stamp).toSec();
      if (dt > 0.0) {
        current_velocity.header.stamp = ros::Time::now();
        current_velocity.header.frame_id = avg_pose.header.frame_id;
        current_velocity.twist.linear.x = (avg_pose.pose.position.x - last_avg_pose_.pose.position.x) / dt;
        current_velocity.twist.linear.y = (avg_pose.pose.position.y - last_avg_pose_.pose.position.y) / dt;
        current_velocity.twist.linear.z = (avg_pose.pose.position.z - last_avg_pose_.pose.position.z) / dt;
        vel_pub_.publish(current_velocity);
      }
    } else {
      first_avg_ = false;
    }
    
    if (!first_velocity_ && !current_velocity.header.stamp.isZero()) {
      double dt_acc = (current_velocity.header.stamp - last_velocity_.header.stamp).toSec();
      if (dt_acc > 0.0) {
        geometry_msgs::AccelStamped accel;
        accel.header.stamp = ros::Time::now();
        accel.header.frame_id = avg_pose.header.frame_id;
        accel.accel.linear.x = (current_velocity.twist.linear.x - last_velocity_.twist.linear.x) / dt_acc;
        accel.accel.linear.y = (current_velocity.twist.linear.y - last_velocity_.twist.linear.y) / dt_acc;
        accel.accel.linear.z = (current_velocity.twist.linear.z - last_velocity_.twist.linear.z) / dt_acc;
        acc_pub_.publish(accel);
      }
    } else {
      first_velocity_ = false;
    }
    
    last_avg_pose_ = avg_pose;
    last_velocity_ = current_velocity;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_averager_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  PoseAveragerNode node(nh, private_nh);
  ros::spin();
  return 0;
}