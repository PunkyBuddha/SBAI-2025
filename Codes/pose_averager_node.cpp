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

    sx_ = 0.0;
    sy_ = 0.0;
    sz_ = 0.0;

    qx_ = 0.0;
    qy_ = 0.0;
    qz_ = 0.0;

    x_est_ = 0.0;
    y_est_ = 0.0;
    z_est_ = 0.0;

    x_vel_ = 0.0;
    y_vel_ = 0.0;
    z_vel_ = 0.0;

    x_acc_ = 0.0;
    y_acc_ = 0.0;
    z_acc_ = 0.0;

    wn_ = 50;
    zeta_ = 1;

    Ts_ = 1/50;
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher vel_pub_;
  ros::Publisher acc_pub_;
  double sx_;
  double sy_;
  double sz_;

  double qx_;
  double qy_;
  double qz_;

  double x_est_;
  double y_est_;
  double z_est_;

  double x_vel_;
  double y_vel_;
  double z_vel_;

  double x_acc_;
  double y_acc_;
  double z_acc_;

  double wn_;
  double zeta_;

  double Ts_;

  double A[2][2] = {{0, 1}, {-(wn_*wn_), -2*zeta_*wn_}};
  double B[2][1] = {0, wn_*wn_};
  double C[3][2] = {{1, 0}, {0, 1}, {-(wn_*wn_), -2*zeta_*wn_}};
  double D[3][1] = {0, 0, wn_*wn_};
  int I[2][2] = {{1, 0}, {0, 1}}; 

  double Ad = I + A*Ts_;
  double Bd = B*Ts_;

  std::string subscribe_topic_;
  std::string publish_topic_;

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = msg->header.frame_id;
    pose.header.stamp = msg->header.stamp;

    double time = msg->header.stamp.toSec();

    sx_ = C*qx_+ D*msg->pose.position.x;
    sy_ = C*qy_+ D*msg->pose.position.y;
    sz_ = C*qz_+ D*msg->pose.position.z; 

    qx_ = Ad*qx_ + Bd*msg->pose.position.x;
    qy_ = Ad*qy_ + Bd*msg->pose.position.y;
    qz_ = Ad*qz_ + Bd*msg->pose.position.z;

    x_est_ = sx_(0); 
    y_est_ = sy_(0);
    z_est_ = sz_(0);

    pose.pose.position.x = x_est_;
    pose.pose.position.y = y_est_;
    pose.pose.position.z = z_est_;

    pub_.publish(pose);

    x_vel_ = sx_(1);
    y_vel_ = sy_(1);
    z_vel_ = sz_(1);

    geometry_msgs::TwistStamped vel;
    vel.header.frame_id = msg->header.frame_id;
    vel.header.stamp = msg->header.stamp;
    vel.twist.linear.x = x_vel_;
    vel.twist.linear.y = y_vel_;
    vel.twist.linear.z = z_vel_;

    vel_pub_.publish(vel);

    x_acc_ = sx_(2);
    y_acc_ = sy_(2);
    z_acc_ = sz_(2);

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