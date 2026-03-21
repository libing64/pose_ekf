#ifndef __SENSOR_SIMULATOR_H
#define __SENSOR_SIMULAOTR_H

#include <iostream>
#include <map>

#include <Eigen/Eigen>

#include "noise_simulator.h"
#include "sensor_config.h"

using namespace std;
using namespace Eigen;

class sensor_simulator {
 private:
  int n = 10;
  double eps = 1e-5;
  double t0 = 0.0;  // static time
  MatrixXd fs;
  MatrixXd A;
  MatrixXd phi;

  // state
  double timestamp;
  Vector3d pos;
  Vector3d vel;
  Vector3d linear_acc;
  Quaterniond rot;
  Vector3d angular_rate;
  Vector3d acc_bias;
  Vector3d gyro_bias;

  // measurement
  Vector3d acc_raw;
  Vector3d gyro_raw;
  Vector3d mag_raw;
  Vector3d gps_pos;
  Vector3d gps_vel;

  // noise simulator

 public:
  // contants
  const Vector3d gravity = Vector3d(0, 0, -9.8);  // NED
  const Vector3d magnetic = Vector3d(1, 0, 0.2);  //[1, 0, x]

  double max_acc;

 public:
  sensor_simulator(/* args */);
  ~sensor_simulator();

  void generate_state(double tt);
  void generate_measurement(double tt);

  Vector3d generate_pos(double t);
  Quaterniond generate_rot(double t);
  Vector3d generate_vel(double t);
  Vector3d generate_linear_acc(double t);
  Vector3d generate_angular_rate(double t);

  // get state and measurement
  Vector3d get_pos() { return pos; };
  Quaterniond get_rot() { return rot; };
  Vector3d get_vel() { return vel; };
  Vector3d get_linear_acc() { return linear_acc; };
  Vector3d get_angular_rate() { return angular_rate; };

  double get_time_stamp() { return timestamp; }

  void set_max_acc(double val) { max_acc = val; }
};

#endif