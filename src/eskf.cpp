#include "eskf.h"

eskf::eskf() {}

eskf::eskf(int n) {
  this->P = MatrixXd::Identity(n, n);
  this->n = n;
}

eskf::~eskf() {}

void eskf::eskf_init_P(VectorXd p) {
  for (int i = 0; i < n; i++) {
    P(i, i) = p(i);
  }
}

void eskf::update(VectorXd dz, MatrixXd H, MatrixXd R) {
  MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  this->dx = K * dz;
  MatrixXd I = MatrixXd::Identity(n, n);
  P = (I - K * H) * P;
}