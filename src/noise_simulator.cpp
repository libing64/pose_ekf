#include "noise_simulator.h"

#include <iostream>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

noise_simulator::noise_simulator(double sigma_white_noise,
                                 double sigma_rw_noise, double const_bias_max,
                                 double freq) {
  this->sigma_white_noise = sigma_white_noise;
  this->sigma_random_walk_noise = sigma_rw_noise;
  this->const_bias_max = const_bias_max;
  this->freq = freq;

  const_bias = Vector3d::Random() * const_bias_max;
  drift_bias = Vector3d::Zero();
  white_noise = Vector3d::Zero();
}

noise_simulator::~noise_simulator() {}

Vector3d noise_simulator::random_vec() {
  Vector3d v;
  for (int i = 0; i < 3; i++) {
    v(i) = this->distribution(random_engine);
  }
  return v;
}
Vector3d noise_simulator::get_white_noise() { return white_noise; }
Vector3d noise_simulator::get_random_walk_noise() { return drift_bias; }

Vector3d noise_simulator::get_noise() {
  return white_noise + drift_bias + const_bias;
}

Vector3d noise_simulator::get_bias() { return const_bias + drift_bias; }

void noise_simulator::generate_noise() {
  white_noise = random_vec() * sigma_white_noise * sqrt(freq);
  Vector3d rate = random_vec() * sigma_random_walk_noise * sqrt(freq);
  this->drift_bias += rate * 1.0 / freq;
}
