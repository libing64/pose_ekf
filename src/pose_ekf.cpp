#include "pose_ekf.h"

Matrix3d _skew_symmetric3(Vector3d v) {
  Matrix3d m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

pose_ekf::pose_ekf() {
  is_init_done = false;
  is_atti_init_done = false;
  this->pos.setZero();
  this->vel.setZero();
  this->q.setIdentity();
  this->acc_bias.setZero();
  this->gyro_bias.setZero();
}
pose_ekf::~pose_ekf() {}

void pose_ekf::set_P_matrix() {
  VectorXd p = VectorXd::Ones(n);
  p.segment(0, 3) = Vector3d::Ones() * gps_pos_noise * gps_pos_noise;
  p.segment(3, 3) = Vector3d::Ones() * gps_vel_noise * gps_vel_noise;
  p.segment(6, 3) = Vector3d::Ones() * 0.01 * 0.01;
  p.segment(9, 3) = Vector3d::Ones() * 0.01 * 0.01;
  p.segment(12, 3) = Vector3d::Ones() * 0.01 * 0.01;
  kf.eskf_init_P(p);
}
void pose_ekf::set_Q_matrix(double dt) {
  Q = MatrixXd::Zero(n, n);
  Matrix3d I3 = Matrix3d::Identity();
  Q.block(0, 0, 3, 3) = I3 * gps_vel_noise * gps_vel_noise;
  Q.block(3, 3, 3, 3) = I3 * acc_noise * acc_noise;
  Q.block(6, 6, 3, 3) = I3 * gyro_noise * gyro_noise;
  Q.block(9, 9, 3, 3) = I3 * acc_bias_noise * acc_bias_noise;
  Q.block(6, 12, 3, 3) =
      -gyro_bias_noise * gyro_bias_noise * I3 * dt * dt / 2.0;
  Q.block(12, 6, 3, 3) =
      -gyro_bias_noise * gyro_bias_noise * I3 * dt * dt / 2.0;
  Q.block(12, 12, 3, 3) = I3 * gyro_bias_noise * gyro_bias_noise;
}

void pose_ekf::pose_init(Vector3d pos, Vector3d vel) {
  kf = eskf(n);

  this->pos = pos;
  this->vel = vel;
  is_init_done = true;

  this->set_Q_matrix(1.0 / imu_freq);
  this->set_P_matrix();
  print_state();
}

void pose_ekf::atti_init(Vector3d acc) {
  // cout << "gravity: " << this->gravity.transpose() << endl;
  // cout << "acc: " << acc.transpose() << endl;
  this->q.setFromTwoVectors(acc, gravity);

  is_atti_init_done = true;
  cout << "atti init with acc " << endl;

  this->set_Q_matrix(1.0 / imu_freq);
  this->set_P_matrix();
  this->print_state();
}

void pose_ekf::atti_init(Vector3d acc_raw, Vector3d mag) {
  // cout << "gravity: " << this->gravity.transpose() << endl;
  // cout << "acc: " << acc.transpose() << endl;
  Vector3d acc = acc_raw - acc_bias;
  this->q.setFromTwoVectors(acc, this->gravity);

  Vector3d mag_ned = q.toRotationMatrix() * mag;
  mag_ned(2) = 0;  // remove z direction
  Vector3d mag_hori = mag_ned.normalized();

  Quaterniond dq;
  dq.setFromTwoVectors(mag_hori, magnetic);

  this->q = dq * this->q;

  is_atti_init_done = true;
  cout << "atti ekf init with acc and mag" << endl;
  this->print_state();
}

void pose_ekf::predict(Vector3d omega_raw, Vector3d acc_raw, double dt) {
  /*
   * Prediction Step (Nominal State Kinematics & Error State Propagation)
   *
   * 1. Nominal State Propagation:
   *    q_{k+1} = q_k ⊗ [1, 0.5 * (w_m - b_w) * dt]^T
   *    a_ned   = R(q_k) * (a_m - b_a) - g
   *    p_{k+1} = p_k + v_k * dt + 0.5 * a_ned * dt^2
   *    v_{k+1} = v_k + a_ned * dt
   */
  Vector3d acc = acc_raw - acc_bias;
  Vector3d omega = omega_raw - gyro_bias;

  Quaterniond dq;
  Vector3d half_theta = omega * dt * 0.5;
  dq.vec() = half_theta;
  double vec_sq_norm = half_theta.squaredNorm();
  // 增加边界保护，防止因为奇异值或过大的角速度导致 w 出现 NaN
  dq.w() = (vec_sq_norm < 1.0) ? sqrt(1.0 - vec_sq_norm) : 0.0;
  dq.normalize();  // 归一化保证严格的旋转属性

  this->q *= dq;
  if (q.w() < 0) q.coeffs() *= -1;

  Matrix3d Rot = q.toRotationMatrix();  // body to NED
  Vector3d acc_ned = Rot * acc;
  Vector3d linear_acceleration = acc_ned - gravity;

  pos += (vel * dt + 0.5 * linear_acceleration * dt * dt);
  vel += linear_acceleration * dt;

  Matrix3d Rnb = Rot;  // body to ned
  /*
   * 2. Error State Transition Matrix (Phi):
   *    δp_{k+1} = δp_k + δv_k * dt
   *    δv_{k+1} = δv_k - R * [a_m - b_a]_x * δθ * dt - R * δb_a * dt
   *    δθ_{k+1} = δθ_k - [w_m - b_w]_x * δθ * dt - δb_w * dt
   *    δb_a_{k+1} = δb_a_k
   *    δb_w_{k+1} = δb_w_k
   */
  Phi = MatrixXd::Identity(n, n);
  Matrix3d I3 = Matrix3d::Identity();
  // Phi.block(3, 6, 3, 3) = -I3 * dt + dt * dt *
  // _skew_symmetric3(linear_acceleration);

  // first order
  // pos part
  Phi.block(0, 3, 3, 3) = I3 * dt;

  // vel part
  Phi.block(3, 6, 3, 3) = -Rnb * _skew_symmetric3(acc) * dt;
  Phi.block(3, 9, 3, 3) = -Rnb * dt;

  // rot part, indirect kalman filter
  Phi.block(6, 6, 3, 3) = I3 - _skew_symmetric3(omega) * dt;
  Phi.block(6, 12, 3, 3) = -I3 * dt + dt * dt / 2.0 * _skew_symmetric3(omega);

  // higher order
  // pos part
  // Phi.block(0, 3, 3, 3) = I3 * dt;
  // Phi.block(0, 6, 3, 3) = -Rnb * _skew_symmetric3(acc) * 0.5 * dt * dt;
  // Phi.block(3, 9, 3, 3) = -Rnb * 0.5 * dt * dt;
  // Phi.block(3, 12, 3, 3)= Rnb * _skew_symmetric3(acc) * 0.5 * dt * dt;

  // //vel part
  // Phi.block(3, 6, 3, 3) = -Rnb * _skew_symmetric3(acc) *(I3 - 0.5 *
  // _skew_symmetric3(omega)* dt)* dt; Phi.block(3, 9, 3, 3) = -Rnb * dt;

  // //rot part, indirect kalman filter
  // Phi.block(6, 6, 3, 3) = I3 - _skew_symmetric3(omega) * dt;
  // Phi.block(6, 12, 3, 3) = -I3 * dt + dt * dt / 2.0 *
  // _skew_symmetric3(omega);

  MatrixXd Fi = MatrixXd::Zero(n, n - 3);
  MatrixXd QQ = MatrixXd::Zero(n - 3, n - 3);
  Fi.block(3, 0, n - 3, n - 3) = MatrixXd::Identity(n - 3, n - 3);
  QQ.block(0, 0, 3, 3) = I3 * ACC_NOISE_DENSITY * ACC_NOISE_DENSITY * dt;
  QQ.block(3, 3, 3, 3) = I3 * GYRO_NOISE_DENSITY * GYRO_NOISE_DENSITY * dt;
  // 偏置的随机游走方差从连续时间到离散时间需要乘以 dt
  QQ.block(6, 6, 3, 3) = I3 * ACC_RANDOM_WALK * ACC_RANDOM_WALK * dt;
  QQ.block(9, 9, 3, 3) = I3 * GYRO_RANDOM_WALK * GYRO_RANDOM_WALK * dt;
  // kf.P = Phi * kf.P * Phi.transpose() + Q;
  kf.P = Phi * kf.P * Phi.transpose() + Fi * QQ * Fi.transpose();

  // MatrixXd QQQ = Fi * QQ * Fi.transpose();
  // cout << "QQQ: " << QQQ << endl;
  // cout << "P: " << kf.P << endl;
}

void pose_ekf::update_gps(Vector3d pos, Vector3d vel) {
  /*
   * GPS Position and Velocity Update
   * Measurement Model:
   *  z = [p_gps, v_gps]^T
   *  z_pred = [p_nom, v_nom]^T
   *  H = [ I_{3x3}   0       0   0   0 ]
   *      [ 0       I_{3x3}   0   0   0 ]
   */
  MatrixXd H = MatrixXd::Zero(6, n);
  H.block(0, 0, 3, 3) = Matrix3d::Identity();
  H.block(3, 3, 3, 3) = Matrix3d::Identity();

  MatrixXd R = MatrixXd::Zero(6, 6);
  R.block(0, 0, 3, 3) = Matrix3d::Identity() * gps_pos_noise;
  R.block(3, 3, 3, 3) = Matrix3d::Identity() * gps_vel_noise;

  VectorXd delta = VectorXd::Zero(6);
  delta.segment(0, 3) = pos - this->pos;
  delta.segment(3, 3) = vel - this->vel;

  kf.update(delta, H, R);

  update_state();
  this->delta_gps = delta;
}

void pose_ekf::update_gps_vel(Vector3d vel) {
  /*
   * GPS Velocity Update
   * Measurement Model:
   *  H = [ 0   I_{3x3}   0   0   0 ]
   */
  MatrixXd H = MatrixXd::Zero(3, n);
  H.block(0, 3, 3, 3) = Matrix3d::Identity();

  Matrix3d R = Matrix3d::Identity() * gps_vel_noise * gps_vel_noise;

  Vector3d delta = vel - this->vel;
  kf.update(delta, H, R);

  update_state();
  this->delta_vel = delta;
}

void pose_ekf::update_gps_pos(Vector3d pos) {
  /*
   * GPS Position Update
   * Measurement Model:
   *  H = [ I_{3x3}   0   0   0   0 ]
   */
  MatrixXd H = MatrixXd::Zero(3, n);
  H.block(0, 0, 3, 3) = Matrix3d::Identity();

  Matrix3d R = Matrix3d::Identity() * gps_pos_noise * gps_pos_noise;

  Vector3d delta = pos - this->pos;
  kf.update(delta, H, R);

  update_state();
  this->delta_pos = delta;
}

void pose_ekf::update_acc(Vector3d acc_raw) {
  /*
   * Accelerometer Update (Attitude Correction)
   * Uses Multiplicative EKF approach:
   * 1. Expected gravity in body frame: vz = R^T * g
   * 2. Calculate small rotation dq between measured acc and expected vz
   * 3. Convert dq to angle-axis (δθ) as measurement residual
   *
   * Measurement Matrix:
   *  H = [ 0   0   I_{3x3}   0   0 ]
   */
  Vector3d acc = acc_raw - acc_bias;

  Matrix3d Rot = q.toRotationMatrix().transpose();
  Vector3d vz = Rot * gravity;

  Quaterniond dq;
  dq.setFromTwoVectors(acc, vz);

  AngleAxisd angle_axis(dq);
  Vector3d delta_theta = angle_axis.angle() * angle_axis.axis();
  this->delta_theta_acc = delta_theta;
  // cout << "delta_theta: " << delta_theta.transpose() << endl;

  MatrixXd H = MatrixXd::Zero(3, n);
  H.block(0, 6, 3, 3) = Matrix3d::Identity();

  // cout << "H: " << H << endl;
  Matrix3d R = Matrix3d::Identity() * acc_noise;
  kf.update(delta_theta, H, R);
  update_state();
}

void pose_ekf::update_magnetic(Vector3d mag_raw) {
  /*
   * Magnetometer Update (Heading Correction)
   * Uses Multiplicative EKF approach:
   * Calculates a small rotation error between measured magnetic field
   * and expected reference magnetic field, and maps directly to δθ.
   *
   * Measurement Matrix:
   *  H = [ 0   0   I_{3x3}   0   0 ]
   */
  printf("%s, %d\n", __FUNCTION__, __LINE__);
  print_state();
  // TODO magenetic in horizon
  Vector3d mag_ned = q.toRotationMatrix() * mag_raw;

  cout << "mag_ned: " << mag_ned.transpose() << endl;

  mag_ned(2) = 0;  // remove z direction
  Vector3d mag_hori = q.toRotationMatrix().transpose() * mag_ned;
  cout << "mag_hori: " << mag_hori.transpose() << endl;

  Matrix3d Rot = q.toRotationMatrix().transpose();
  Vector3d vx = Rot * magnetic;

  Quaterniond dq;
  dq.setFromTwoVectors(mag_raw, vx);

  AngleAxisd angle_axis(dq);
  Vector3d delta_theta = angle_axis.angle() * angle_axis.axis();
  this->delta_theta_mag = delta_theta;
  // cout << "delta_theta: " << delta_theta.transpose() << endl;

  MatrixXd H = MatrixXd::Zero(3, n);
  H.block(0, 6, 3, 3) = Matrix3d::Identity();
  // cout << "H: " << H << endl;
  Matrix3d R = Matrix3d::Identity() * mag_noise * mag_noise;
  R(0, 0) *= 100;
  R(1, 1) *= 100;
  R = q.toRotationMatrix().transpose() * R * q.toRotationMatrix();
  kf.update(delta_theta, H, R);
  update_state();
}

void pose_ekf::update_state() {
  /*
   * Error State Injection (Reset/Update)
   * Injects the estimated error state into the nominal state:
   *  p = p + δp
   *  v = v + δv
   *  q = q ⊗ exp(δθ / 2)
   *  b_a = b_a + δb_a
   *  b_w = b_w + δb_w
   */
  VectorXd dx = kf.get_state();
  cout << "kf.dx: " << dx.transpose() << endl;
  pos += dx.segment(0, 3);
  vel += dx.segment(3, 3);

  Quaterniond dq;
  dq.vec() = dx.segment(6, 3) * 0.5;
  dq.w() = sqrt(1.0 - dq.vec().dot(dq.vec()));
  q = q * dq;
  q.normalize();
  if (q.w() < 0) q.coeffs() *= -1;
  acc_bias += dx.segment(9, 3);
  gyro_bias += dx.segment(12, 3);
}

void pose_ekf::print_state() {
  cout << "pos: " << pos.transpose() << endl;
  cout << "vel: " << vel.transpose() << endl;
  cout << "q: " << q.w() << "  " << q.vec().transpose() << endl;
  cout << "acc_bias: " << acc_bias.transpose() << endl;
  cout << "gyro_bias: " << gyro_bias.transpose() << endl;
}