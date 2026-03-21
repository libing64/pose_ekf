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
  /*
   * Standard Kalman Filter Measurement Update Step
   *
   * Mathematical Formulation:
   * 1. Innovation (or residual) covariance:
   *    S = H * P * H^T + R
   *
   * 2. Kalman Gain:
   *    K = P * H^T * S^-1
   *
   * 3. State update (Error state computation):
   *    δx = K * dz   (where dz is the measurement residual)
   *
   * 4. Covariance update (Joseph Form to maintain symmetry and positive
   * definiteness): P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
   */
  MatrixXd S = H * P * H.transpose() + R;
  // 使用 ldlt() 求解提高数值稳定性，代替直接 .inverse()
  MatrixXd K = P * H.transpose() *
               S.ldlt().solve(MatrixXd::Identity(S.rows(), S.cols()));
  this->dx = K * dz;

  MatrixXd I = MatrixXd::Identity(n, n);
  MatrixXd I_KH = I - K * H;
  // 使用 Joseph 形式更新协方差，保证正定性和数值稳定性
  P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
}