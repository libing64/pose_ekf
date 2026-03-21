#ifndef __POSE_EKF_H
#define __POSE_EKF_H
#include <iostream>

#include <Eigen/Eigen>

#include "eskf.h"
#include "sensor_config.h"

using namespace std;
using namespace Eigen;

class pose_ekf {
 public:
  // state vector[p, v, q, ba, bw]
  Vector3d pos;
  Vector3d vel;
  Quaterniond q;
  Vector3d acc_bias;
  Vector3d gyro_bias;

  const int n = 15;
  double imu_freq = 100;
  double timestamp;

  const Vector3d gravity = Vector3d(0, 0, -9.8);
  const Vector3d magnetic = Vector3d(1, 0, 0);
  const double gyro_bias_noise = GYRO_RANDOM_WALK;
  const double gyro_noise = GYRO_NOISE_DENSITY;
  const double acc_noise = ACC_NOISE_DENSITY;
  const double acc_bias_noise = ACC_RANDOM_WALK;
  const double mag_noise = MAG_NOISE_DENSITY;
  const double gps_pos_noise = GPS_POS_NOISE_DENSITY;
  const double gps_vel_noise = GPS_VEL_NOISE_DENSITY;

  MatrixXd Q;
  MatrixXd Phi;

  eskf kf;
  bool is_atti_init_done;
  bool is_init_done;

  Vector3d delta_vel;
  Vector3d delta_pos;
  VectorXd delta_gps;
  Vector3d delta_theta_acc;
  Vector3d delta_theta_mag;

  pose_ekf();
  ~pose_ekf();

  void pose_init(Vector3d pos, Vector3d vel);
  void atti_init(Vector3d acc);
  void atti_init(Vector3d acc, Vector3d mag);

  void predict(Vector3d acc, Vector3d omega_raw, double dt);

  void update_gps_vel(Vector3d vel);
  void update_gps_pos(Vector3d pos);
  void update_gps(Vector3d pos, Vector3d vel);
  void update_acc(Vector3d acc);
  void update_magnetic(Vector3d mag);
  void update_linear_acc(Vector3d linear_acc, Vector3d acc);

  void set_imu_freq(double freq) { this->imu_freq = freq; }
  void set_Q_matrix(double dt);
  void set_P_matrix();
  void update_state();
  void print_state();
  Vector3d get_atti_euler();
  void set_timestatmp(double t) { timestamp = t; }
};

#endif