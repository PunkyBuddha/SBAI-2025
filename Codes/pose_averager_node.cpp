#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>

class PoseAveragerNode {
public:
  PoseAveragerNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
    private_nh.param<std::string>("subscribe_topic", subscribe_topic_, "/natnet_client_node/B1/pose");
    private_nh.param<std::string>("publish_topic", publish_topic_, "/natnet_client_node/B1/pose_averaged");

    sub_ = nh.subscribe(subscribe_topic_, 1000, &PoseAveragerNode::poseCallback, this);
    pub_ = nh.advertise<geometry_msgs::PoseStamped>(publish_topic_, 10);
    vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>(publish_topic_ + "_velocity", 10);
    acc_pub_ = nh.advertise<geometry_msgs::AccelStamped>(publish_topic_ + "_acceleration", 10);

    x_est_ = 0.0;
    y_est_ = 0.0;
    z_est_ = 0.0;

    x_vel_ = 0.0;
    y_vel_ = 0.0;
    z_vel_ = 0.0;

    x_acc_ = 0.0;
    y_acc_ = 0.0;
    z_acc_ = 0.0;

    alpha_pos_ = 0.5;
    alpha_vel_ = alpha_pos_ / 2;
    alpha_acc_ = alpha_pos_ / 8;
    last_time_ = -1;
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher vel_pub_;
  ros::Publisher acc_pub_;
  double x_est_;
  double y_est_;
  double z_est_;

  double x_vel_;
  double y_vel_;
  double z_vel_;

  double x_acc_;
  double y_acc_;
  double z_acc_;

  double alpha_pos_;
  double alpha_vel_;
  double alpha_acc_;

  double dt_;
  double last_time_;
  std::string subscribe_topic_;
  std::string publish_topic_;

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = msg->header.frame_id;
    pose.header.stamp = msg->header.stamp;

    double time = msg->header.stamp.toSec();

    if (last_time_ == -1) {
      last_time_ = time;
      return;
    }

    dt_ = time - last_time_;
    // std::cout << "dt: " << dt_ << std::endl;
    last_time_ = time;

    // std::cout << "time: " << time << std::endl;

    x_est_ = x_est_ + alpha_pos_ * (msg->pose.position.x - x_est_) + dt_*x_vel_ + 0.5*dt_*dt_*x_acc_; 
    y_est_ = y_est_ + alpha_pos_ * (msg->pose.position.y - y_est_) + dt_*y_vel_ + 0.5*dt_*dt_*y_acc_;
    z_est_ = z_est_ + alpha_pos_ * (msg->pose.position.z - z_est_) + dt_*z_vel_ + 0.5*dt_*dt_*z_acc_;

    pose.pose.position.x = x_est_;
    pose.pose.position.y = y_est_;
    pose.pose.position.z = z_est_;

    pub_.publish(pose);

    x_vel_ = x_vel_ + alpha_vel_ * ((msg->pose.position.x - x_est_) / dt_);
    y_vel_ = y_vel_ + alpha_vel_ * ((msg->pose.position.y - y_est_) / dt_);
    z_vel_ = z_vel_ + alpha_vel_ * ((msg->pose.position.z - z_est_) / dt_);

    geometry_msgs::TwistStamped vel;
    vel.header.frame_id = msg->header.frame_id;
    vel.header.stamp = msg->header.stamp;
    vel.twist.linear.x = x_vel_;
    vel.twist.linear.y = y_vel_;
    vel.twist.linear.z = z_vel_;

    vel_pub_.publish(vel);

    x_acc_ = x_acc_ + alpha_acc_ * ((msg->pose.position.x - x_est_) / (0.5 * dt_*dt_));
    y_acc_ = y_acc_ + alpha_acc_ * ((msg->pose.position.y - y_est_) / (0.5 * dt_*dt_));
    z_acc_ = z_acc_ + alpha_acc_ * ((msg->pose.position.z - z_est_) / (0.5 * dt_*dt_));

    geometry_msgs::AccelStamped acc;
    acc.header.frame_id = msg->header.frame_id;
    acc.header.stamp = msg->header.stamp;
    acc.accel.linear.x = x_acc_;
    acc.accel.linear.y = y_acc_;
    acc.accel.linear.z = z_acc_;

    acc_pub_.publish(acc);
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