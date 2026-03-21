#ifndef __NOISE_SIMULATOR_H
#define __NOISE_SIMULATOR_H

#include <iostream>
#include <random>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class noise_simulator {
 private:
  double sigma_white_noise;
  double sigma_random_walk_noise;
  float freq;

  Vector3d white_noise;
  Vector3d const_bias;
  Vector3d drift_bias;

  double const_bias_max;
  std::default_random_engine random_engine;
  std::normal_distribution<double> distribution;
  Vector3d random_vec();

 public:
  noise_simulator(double sigma_white_noise, double sigma_rw_noise,
                  double const_bias_max, double freq);
  ~noise_simulator();

  Vector3d get_white_noise();
  Vector3d get_random_walk_noise();
  Vector3d get_noise();
  Vector3d get_bias();

  void generate_noise();
};

#endif
