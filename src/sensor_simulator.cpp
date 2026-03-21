#include "sensor_simulator.h"

#include <iostream>

#include <Eigen/Eigen>

#include "noise_simulator.h"
#include "sensor_config.h"

using namespace std;
using namespace Eigen;

sensor_simulator::sensor_simulator(/* args */) {
  fs = MatrixXd::Random(6, n) * 0.1;
  A = MatrixXd::Random(6, n);
  phi = MatrixXd::Zero(6, n);
}

sensor_simulator::~sensor_simulator() {}

void sensor_simulator::generate_state(double tt) {
  this->timestamp = tt;
  this->pos = generate_pos(tt);
  this->vel = generate_vel(tt);
  this->rot = generate_rot(tt);
  this->linear_acc = generate_linear_acc(tt);
  this->angular_rate = generate_angular_rate(tt);
}

void sensor_simulator::generate_measurement(double tt) {
  if (fabs(tt - timestamp) > eps) {
    this->generate_state(tt);
  }

  Matrix3d R = rot.toRotationMatrix().transpose();  // ned to body
  acc_raw = R * (gravity + linear_acc);
  gyro_raw = angular_rate;
  mag_raw = R * magnetic;
}

Vector3d sensor_simulator::generate_pos(double tt) {
  if (max_acc <= 0.0) {
    Vector3d pos = Vector3d::Zero();
    return pos;
  }
  double t = 0;
  if (tt >= t0) {
    t = tt - t0;
  }
  Vector3d pos = Vector3d::Zero();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < n; j++) {
      pos(i) += A(i, j) * cos(2 * M_PI * fs(i, j) * t + phi(i, j));
    }
  }
  return pos;
}

Quaterniond sensor_simulator::generate_rot(double tt) {
  double t = 0;
  if (tt >= t0) {
    t = tt - t0;
  }
  Vector3d rot = Vector3d::Zero();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < n; j++) {
      rot(i) += A(i + 3, j) * cos(2 * M_PI * fs(i + 3, j) * t + phi(i + 3, j));
    }
  }

  Quaterniond q = AngleAxisd(rot(2), Vector3d::UnitZ()) *
                  AngleAxisd(rot(1), Vector3d::UnitY()) *
                  AngleAxisd(rot(0), Vector3d::UnitX());

  if (q.w() < 0) q.coeffs() *= -1;
  return q;
}

Vector3d sensor_simulator::generate_vel(double t) {
  Vector3d p1 = generate_pos(t - eps);
  Vector3d p2 = generate_pos(t + eps);
  Vector3d vel = (p2 - p1) / (2 * eps);
  return vel;
}

Vector3d sensor_simulator::generate_linear_acc(double t) {
  Vector3d v1 = generate_vel(t - eps);
  Vector3d v2 = generate_vel(t + eps);
  Vector3d linear_acc = (v2 - v1) / (2 * eps);
  return linear_acc;
}

Vector3d sensor_simulator::generate_angular_rate(double t) {
  Quaterniond q1 = generate_rot(t - eps);
  Quaterniond q2 = generate_rot(t + eps);
  Quaterniond dq = q1.conjugate() * q2;  // body frame
  AngleAxisd angle_aixs = AngleAxisd(dq);
  Vector3d omega = angle_aixs.angle() * angle_aixs.axis() / (2 * eps);
  return omega;
}