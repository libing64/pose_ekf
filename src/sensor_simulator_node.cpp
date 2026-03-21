#include <algorithm>
#include <deque>
#include <iostream>

#include <Eigen/Eigen>
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

#include "math_utils.h"
#include "noise_simulator.h"
#include "pose_ekf.h"
#include "rtklib.h"
#include "sensor_config.h"
#include "sensor_simulator.h"
#include "tf/transform_broadcaster.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_simulator");
  ros::NodeHandle n("~");

  double max_acc = 1.0;
  double max_vel = 1.0;
  double max_omega = 40.0;
  n.param("max_acc", max_acc, 1.0);

  ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("/imu", 1000);
  ros::Publisher pub_mag =
      n.advertise<geometry_msgs::Vector3Stamped>("/magnetic", 1000);
  ros::Publisher pub_fix = n.advertise<sensor_msgs::NavSatFix>("/fix", 1000);
  ros::Publisher pub_fix_velocity =
      n.advertise<geometry_msgs::Vector3Stamped>("/fix_velocity", 1000);
  ros::Publisher pub_odometry =
      n.advertise<nav_msgs::Odometry>("/odometry", 1000);
  ros::Publisher pub_pose =
      n.advertise<geometry_msgs::PoseStamped>("/pose", 1000);

  double freq = IMU_FREQ;
  double total_time = 500;
  Vector3d pos, vel, linear_acc;
  Vector3d omega;
  Quaterniond q;

  sensor_simulator simulator;
  simulator.set_max_acc(max_acc);

  // Vector3d gyro_bias(0.02, 0.01, -0.02);
  // Vector3d acc_bias(-0.01, 0.01, 0.02);
  Vector3d acc_bias(0, 0, 0);
  Vector3d gyro_bias(0, 0, 0);
  Vector3d pos_noise, vel_noise, acc_noise, gyro_noise, mag_noise;

  noise_simulator acc_noise_simulator(ACC_NOISE_DENSITY, ACC_RANDOM_WALK,
                                      ACC_CONST_BIAS_MAX, freq);
  noise_simulator gyro_noise_simulator(GYRO_NOISE_DENSITY, GYRO_RANDOM_WALK,
                                       GYRO_CONST_BIAS_MAX, freq);
  noise_simulator pos_noise_simulator(
      GPS_POS_NOISE_DENSITY, GPS_POS_RANDOM_WALK, 0, freq / IMU_CNT_PER_GPS);
  noise_simulator vel_noise_simulator(
      GPS_VEL_NOISE_DENSITY, GPS_VEL_RANDOM_WALK, 0, freq / IMU_CNT_PER_GPS);
  noise_simulator mag_noise_simulator(MAG_NOISE_DENSITY, MAG_RANDOM_WALK, 0,
                                      freq / IMU_CNT_PER_MAG);
  ros::Publisher pub_path = n.advertise<nav_msgs::Path>("path", 1000);

  // use the trajectory
  double dt = 1.0 / freq;
  nav_msgs::Path path;
  path.header.frame_id = "world";

  ros::Rate loop_rate(freq);
  int j = 0;
  while (ros::ok()) {
    j++;
    double timestamp = j / freq;
    ROS_INFO("time: %lf", timestamp);

    simulator.generate_state(timestamp);
    simulator.generate_measurement(timestamp);

    vel = simulator.get_vel();
    pos = simulator.get_pos();
    q = simulator.get_rot();
    omega = simulator.get_angular_rate();
    linear_acc = simulator.get_linear_acc();

    pos_noise_simulator.generate_noise();
    vel_noise_simulator.generate_noise();
    acc_noise_simulator.generate_noise();
    gyro_noise_simulator.generate_noise();
    mag_noise_simulator.generate_noise();

    pos_noise = pos_noise_simulator.get_noise();
    vel_noise = vel_noise_simulator.get_noise();
    acc_noise = acc_noise_simulator.get_noise();
    gyro_noise = gyro_noise_simulator.get_noise();
    mag_noise = mag_noise_simulator.get_noise();

    acc_bias = acc_noise_simulator.get_bias();
    gyro_bias = gyro_noise_simulator.get_bias();

    // record measurements
    Vector3d pos_raw = pos + pos_noise;
    Vector3d vel_raw = vel + vel_noise;

    // imu data, acc and omega

    Matrix3d R = q.toRotationMatrix().transpose();  // ned to body
    Vector3d acc_raw = R * (simulator.gravity + linear_acc) + acc_noise;
    Vector3d omega_raw = omega + gyro_noise;

    Quaterniond q_clination = AngleAxisd(MAG_DECLINATION, Vector3d::UnitZ()) *
                              AngleAxisd(MAG_INCLINATION, Vector3d::UnitX());
    Vector3d mag_raw =
        R * q_clination.toRotationMatrix() * simulator.magnetic + mag_noise;

    // q = AngleAxisd(0.1, Vector3d::UnitZ());
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(timestamp);

    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = pos(0);
    odometry.pose.pose.position.y = pos(1);
    odometry.pose.pose.position.z = pos(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    odometry.twist.twist.linear.x = vel(0);
    odometry.twist.twist.linear.y = vel(1);
    odometry.twist.twist.linear.z = vel(2);
    odometry.twist.twist.angular.x = omega(0);
    odometry.twist.twist.angular.y = omega(1);
    odometry.twist.twist.angular.z = omega(2);
    pub_odometry.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(timestamp);
    pose_stamped.pose = odometry.pose.pose;
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);
    pub_pose.publish(pose_stamped);

    sensor_msgs::Imu imu;
    imu.header.frame_id = "world";
    imu.header.stamp = ros::Time(timestamp);
    imu.linear_acceleration.x = acc_raw(0);
    imu.linear_acceleration.y = acc_raw(1);
    imu.linear_acceleration.z = acc_raw(2);
    imu.angular_velocity.x = omega_raw(0);
    imu.angular_velocity.y = omega_raw(1);
    imu.angular_velocity.z = omega_raw(2);
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();
    pub_imu.publish(imu);
    ROS_INFO("generate q: %lf,(%lf,%lf,%lf,%lf,)", timestamp, q.w(), q.x(),
             q.y(), q.z());
    Vector3d euler = quat2euler(q);
    ROS_INFO("euler_zxy: %lf,(%lf,%lf,%lf)", timestamp, euler(0), euler(1),
             euler(2));
    // publish ahrs
    {
      static tf2_ros::TransformBroadcaster br;
      geometry_msgs::TransformStamped tr;
      tr.header.stamp = ros::Time(timestamp);
      tr.header.frame_id = "world";
      tr.child_frame_id = "base_link";
      tr.transform.translation.x = pos(0);
      tr.transform.translation.y = pos(1);
      tr.transform.translation.z = pos(2);
      tr.transform.rotation.x = q.x();
      tr.transform.rotation.y = q.y();
      tr.transform.rotation.z = q.z();
      tr.transform.rotation.w = q.w();
      br.sendTransform(tr);

      // static tf::TransformBroadcaster br;
      // tf::Transform transform;
      // transform.setOrigin(tf::Vector3(0, 0, 0));
      // tf::Quaternion qq(q.w(), q.x(), q.y(), q.z());
      // transform.setRotation(qq);
      // br.sendTransform(tf::StampedTransform(transform, ros::Time(timestamp),
      // "world", "base_link"));
    }

    if (j % IMU_CNT_PER_MAG == 0) {
      geometry_msgs::Vector3Stamped magnetic;
      magnetic.header.frame_id = "base_link";
      magnetic.header.stamp = ros::Time(timestamp);
      magnetic.vector.x = mag_raw(0);
      magnetic.vector.y = mag_raw(1);
      magnetic.vector.z = mag_raw(2);
      pub_mag.publish(magnetic);
    }

    if (j % IMU_CNT_PER_GPS == 0) {
      geometry_msgs::Vector3Stamped velocity;
      velocity.header.frame_id = "world";
      velocity.header.stamp = ros::Time(timestamp);
      // velocity in enu frame
      velocity.vector.x = vel_raw(1);
      velocity.vector.y = vel_raw(0);
      velocity.vector.z = -vel_raw(2);
      pub_fix_velocity.publish(velocity);

      double enu[3] = {0};  // ned to enu
      enu[0] = pos_raw(1);
      enu[1] = pos_raw(0);
      enu[2] = -pos_raw(2);
      double lla_ref[3] = {23.5 * M_PI / 180.0, 120.0 * M_PI / 180.0, 100};
      double ecef_ref[3] = {0};
      pos2ecef(lla_ref, ecef_ref);
      double r[3] = {0};
      enu2ecef(lla_ref, enu, r);
      double ecef[3] = {0};
      ecef[0] = ecef_ref[0] + r[0];
      ecef[1] = ecef_ref[1] + r[1];
      ecef[2] = ecef_ref[2] + r[2];

      double lla[3] = {0};
      ecef2pos(ecef, lla);

      sensor_msgs::NavSatFix fix;
      fix.header.frame_id = "world";
      fix.header.stamp = ros::Time(timestamp);
      fix.latitude = lla[0] * 180.0 / M_PI;
      fix.longitude = lla[1] * 180.0 / M_PI;
      fix.altitude = lla[2];
      pub_fix.publish(fix);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}