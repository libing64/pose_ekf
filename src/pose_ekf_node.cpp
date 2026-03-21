#include <algorithm>
#include <deque>
#include <iostream>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Range.h>

#include "pose_ekf.h"
#include "rtklib.h"

using namespace std;
using namespace Eigen;

pose_ekf pose;
ros::Publisher pose_pub;
ros::Publisher gps_pose_pub;

ros::Publisher pub_delta_pos;
ros::Publisher pub_delta_vel;
ros::Publisher pub_delta_theta_mag;
ros::Publisher pub_delta_theta_acc;

double ref_time = 0;
bool is_ref_time_init = false;

nav_msgs::Path path;
ros::Publisher pub_path;
ros::Publisher pub_odom;

nav_msgs::Path gps_path;
ros::Publisher pub_gps_path;

Vector3d mag_raw;
bool is_mag_init = false;

void publish_pose(pose_ekf _pose) {
  if (_pose.is_init_done) {
    geometry_msgs::PoseStamped pose;
    Quaterniond q = _pose.q;
    Vector3d p = _pose.pos;

    pose.header.stamp = ros::Time(_pose.timestamp);
    pose.header.frame_id = "world";
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.position.x = p(0);
    pose.pose.position.y = p(1);
    pose.pose.position.z = p(2);
    pose_pub.publish(pose);

    // cout << "publish pose: " << endl;
    //_pose.print_state();

    path.poses.push_back(pose);
    pub_path.publish(path);
  }
}

void publish_odom(pose_ekf pose) {
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "world";
  odometry.header.stamp = ros::Time(pose.timestamp);

  odometry.child_frame_id = "base_link";
  odometry.pose.pose.position.x = pose.pos(0);
  odometry.pose.pose.position.y = pose.pos(1);
  odometry.pose.pose.position.z = pose.pos(2);
  odometry.pose.pose.orientation.x = pose.q.x();
  odometry.pose.pose.orientation.y = pose.q.y();
  odometry.pose.pose.orientation.z = pose.q.z();
  odometry.pose.pose.orientation.w = pose.q.w();
  odometry.twist.twist.linear.x = pose.vel(0);
  odometry.twist.twist.linear.y = pose.vel(1);
  odometry.twist.twist.linear.z = pose.vel(2);
  odometry.twist.twist.angular.x = 0;  // pose.omega(0);
  odometry.twist.twist.angular.y = 0;  // pose.omega(1);
  odometry.twist.twist.angular.z = 0;  // pose.omega(2);
  pub_odom.publish(odometry);
}

void publish_gps_pos(Vector3d pos, double time) {
  geometry_msgs::PoseStamped pose;
  Quaterniond q = Quaterniond::Identity();
  Vector3d p = pos;

  pose.header.stamp = ros::Time(time);
  pose.header.frame_id = "world";
  pose.pose.orientation.w = q.w();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.position.x = p(0);
  pose.pose.position.y = p(1);
  pose.pose.position.z = p(2);
  gps_pose_pub.publish(pose);
  // cout << "publish gps pose: " << p.transpose() << endl;

  gps_path.poses.push_back(pose);
  pub_gps_path.publish(gps_path);
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  if (is_ref_time_init == false) {
    ref_time = msg->header.stamp.toSec();
    is_ref_time_init = true;
  }

  static double time_prev = 0.0;
  double time = msg->header.stamp.toSec();
  static int cnt = 0;
  cnt++;

  Vector3d gyro_raw, acc_raw;
  gyro_raw(0) = msg->angular_velocity.x;
  gyro_raw(1) = msg->angular_velocity.y;
  gyro_raw(2) = msg->angular_velocity.z;
  acc_raw(0) = msg->linear_acceleration.x;
  acc_raw(1) = msg->linear_acceleration.y;
  acc_raw(2) = msg->linear_acceleration.z;

  if (pose.is_atti_init_done == false || pose.is_init_done == false) {
    if (is_mag_init) {
      pose.atti_init(acc_raw, mag_raw);
    }
  } else {
    double dt = time - time_prev;
    pose.predict(gyro_raw, acc_raw, dt);
  }

  pose.set_timestatmp(time);
  if (cnt % 100 == 0) {
    cout << "omega: " << gyro_raw.transpose() << endl;
    cout << "acc: " << acc_raw.transpose() << endl;
    cout << "dt: " << time - time_prev << endl;
  }

  time_prev = time;
}

void magCallback(const sensor_msgs::MagneticFieldConstPtr& msg) {
  if (is_ref_time_init == false) {
    ref_time = msg->header.stamp.toSec();
    is_ref_time_init = true;
  }
  double time = msg->header.stamp.toSec() - ref_time;
  // double t = msg->header.stamp.toSec();
  // mag_q.push_back(make_pair(t, *msg));
  mag_raw(0) = msg->magnetic_field.x;
  mag_raw(1) = msg->magnetic_field.y;
  mag_raw(2) = msg->magnetic_field.z;
  is_mag_init = true;

  cout << "mag raw: " << mag_raw.transpose() << endl;
  if (pose.is_atti_init_done && pose.is_init_done) {
    pose.update_magnetic(mag_raw);
    cout << "mag delta: " << time << "s " << pose.delta_theta_mag.transpose()
         << endl;
  }

  geometry_msgs::Vector3Stamped v3_msg;
  v3_msg.header.stamp = ros::Time(time);
  v3_msg.vector.x = pose.delta_theta_mag(0);
  v3_msg.vector.y = pose.delta_theta_mag(1);
  v3_msg.vector.z = pose.delta_theta_mag(2);
  pub_delta_theta_mag.publish(v3_msg);
}

void mag_v3_Callback(const geometry_msgs::Vector3StampedConstPtr& msg) {
  if (is_ref_time_init == false) {
    ref_time = msg->header.stamp.toSec();
    is_ref_time_init = true;
  }
  double time = msg->header.stamp.toSec() - ref_time;
  mag_raw(0) = msg->vector.x;
  mag_raw(1) = msg->vector.y;
  mag_raw(2) = msg->vector.z;
  is_mag_init = true;

  cout << "mag raw: " << mag_raw.transpose() << endl;
  if (pose.is_atti_init_done && pose.is_init_done) {
    pose.update_magnetic(mag_raw);
    cout << "mag delta: " << time << "s " << pose.delta_theta_mag.transpose()
         << endl;
  }

  geometry_msgs::Vector3Stamped v3_msg;
  v3_msg.header.stamp = ros::Time(time);
  v3_msg.vector.x = pose.delta_theta_mag(0);
  v3_msg.vector.y = pose.delta_theta_mag(1);
  v3_msg.vector.z = pose.delta_theta_mag(2);
  pub_delta_theta_mag.publish(v3_msg);
}

void sonarCallback(const sensor_msgs::RangeConstPtr& msg) {
  // double t = msg->header.stamp.toSec();
  // sonar_height_q.push_back(make_pair(t, *msg));
  // cout << "sonar; ";
}

void fixCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
  if (is_ref_time_init == false) {
    ref_time = msg->header.stamp.toSec();
    is_ref_time_init = true;
  }
  static double time_prev = 0;
  static Vector3d pos_ned_prev = Vector3d::Zero();

  double time = msg->header.stamp.toSec() - ref_time;

  static bool ref_pos_init = false;
  static double ecef_ref[3] = {0};
  double lla[3], enu[3];
  lla[0] = msg->latitude / 180.0 * M_PI;
  lla[1] = msg->longitude / 180.0 * M_PI;
  lla[2] = msg->altitude;
  cout << "lla: " << lla[0] << "  " << lla[1] << "  " << lla[2] << endl;

  if (ref_pos_init == false) {
    pos2ecef(lla, ecef_ref);
    ref_pos_init = true;
  }

  double ecef[3] = {0};
  pos2ecef(lla, ecef);
  double r[3] = {0};
  r[0] = ecef[0] - ecef_ref[0];
  r[1] = ecef[1] - ecef_ref[1];
  r[2] = ecef[2] - ecef_ref[2];

  // cout << "r: " << r[0] << "  " << r[1] << "  " << r[2] << endl;
  ecef2enu(lla, r, enu);
  // cout << "enu: " << enu[0] << "  " << enu[1] << "  " << enu[2] << endl;

  Vector3d pos_ned;
  pos_ned(0) = enu[1];
  pos_ned(1) = enu[0];
  pos_ned(2) = -enu[2];

  if (pose.is_init_done == false) {
    Vector3d vel = Vector3d::Zero();
    pose.pose_init(pos_ned, vel);
  } else {
    pose.update_gps_pos(pos_ned);
    cout << "delta pos: " << time << "s  " << pose.delta_pos.transpose()
         << endl;
  }

  publish_gps_pos(pos_ned, time);

  geometry_msgs::Vector3Stamped v3_msg;
  v3_msg.header.stamp = ros::Time(time);
  v3_msg.vector.x = pose.delta_pos(0);
  v3_msg.vector.y = pose.delta_pos(1);
  v3_msg.vector.z = pose.delta_pos(2);
  pub_delta_pos.publish(v3_msg);

  Vector3d vel_dif = (pos_ned - pos_ned_prev) / (time - time_prev);
  cout << "gps vel_dif: " << vel_dif.transpose() << endl;
  time_prev = time;
  pos_ned_prev = pos_ned;
}

// geometry_msgs/TwistStamped
void fixVelocityCallback(const geometry_msgs::TwistStampedConstPtr& msg) {
  if (is_ref_time_init == false) {
    ref_time = msg->header.stamp.toSec();
    is_ref_time_init = true;
  }
  double time = msg->header.stamp.toSec();
  // double t = msg->header.stamp.toSec();
  // fix_velocity_q.push_back(make_pair(t, *msg));

  double enu[3] = {0};
  enu[0] = msg->twist.linear.x;
  enu[1] = msg->twist.linear.y;
  enu[2] = msg->twist.linear.z;

  Vector3d vel_ned;  // enu to ned
  vel_ned(0) = enu[1];
  vel_ned(1) = enu[0];
  vel_ned(2) = -enu[2];

  cout << "gps vel: " << vel_ned.transpose() << endl;
  if (pose.is_init_done == true && pose.is_atti_init_done == true) {
    pose.update_gps_vel(vel_ned);
    cout << "delta vel: " << time << "s  " << pose.delta_vel.transpose()
         << endl;
  }

  geometry_msgs::Vector3Stamped v3_msg;
  v3_msg.header.stamp = ros::Time(time);
  v3_msg.vector.x = pose.delta_vel(0);
  v3_msg.vector.y = pose.delta_vel(1);
  v3_msg.vector.z = pose.delta_vel(2);
  pub_delta_vel.publish(v3_msg);
}

// geometry_msgs/TwistStamped
void fixVelocity_v3_Callback(const geometry_msgs::Vector3StampedConstPtr& msg) {
  if (is_ref_time_init == false) {
    ref_time = msg->header.stamp.toSec();
    is_ref_time_init = true;
  }
  double time = msg->header.stamp.toSec();
  // double t = msg->header.stamp.toSec();
  // fix_velocity_q.push_back(make_pair(t, *msg));

  Vector3d vel_ned;  // enu to ned
  vel_ned(0) = msg->vector.y;
  vel_ned(1) = msg->vector.x;
  vel_ned(2) = -msg->vector.z;

  cout << "gps vel: " << vel_ned.transpose() << endl;
  if (pose.is_init_done == true && pose.is_atti_init_done == true) {
    pose.update_gps_vel(vel_ned);
    cout << "delta vel: " << time << "s  " << pose.delta_vel.transpose()
         << endl;
  }

  geometry_msgs::Vector3Stamped v3_msg;
  v3_msg.header.stamp = ros::Time(time);
  v3_msg.vector.x = pose.delta_vel(0);
  v3_msg.vector.y = pose.delta_vel(1);
  v3_msg.vector.z = pose.delta_vel(2);
  pub_delta_vel.publish(v3_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_estimator");
  ros::NodeHandle n("~");

  pub_path = n.advertise<nav_msgs::Path>("path", 10);
  pub_gps_path = n.advertise<nav_msgs::Path>("gps_path", 10);
  pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 10);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/est_pose", 10);
  gps_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/gps_pose", 10);

  pub_delta_pos = n.advertise<geometry_msgs::Vector3Stamped>("/delta_pos", 10);
  pub_delta_vel = n.advertise<geometry_msgs::Vector3Stamped>("/delta_vel", 10);
  pub_delta_theta_mag =
      n.advertise<geometry_msgs::Vector3Stamped>("/delta_theta_mag", 10);
  pub_delta_theta_acc =
      n.advertise<geometry_msgs::Vector3Stamped>("/delta_theta_acc", 10);

  path.header.frame_id = "world";
  gps_path.header.frame_id = "world";

  ros::Subscriber sub_imu = n.subscribe("imu", 100, imuCallback);
  ros::Subscriber sub_mag = n.subscribe("magnetic_field", 100, magCallback);
  ros::Subscriber sub_mag_v3 =
      n.subscribe("magnetic_field_v3", 100, mag_v3_Callback);
  ros::Subscriber sub_fix = n.subscribe("fix", 100, fixCallback);
  ros::Subscriber sub_sonar = n.subscribe("sonar_height", 100, sonarCallback);
  ros::Subscriber sub_fix_velocity =
      n.subscribe("fix_velocity", 100, fixVelocityCallback);
  ros::Subscriber sub_fix_velocity_v3 =
      n.subscribe("fix_velocity_v3", 100, fixVelocity_v3_Callback);

  double imu_freq = 100;
  pose.set_imu_freq(imu_freq);

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    ros::spinOnce();

    publish_pose(pose);
    publish_odom(pose);

    loop_rate.sleep();
  }
  return 0;
}
