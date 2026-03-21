#include "math_utils.h"

#include <Eigen/Eigen>

using namespace Eigen;

Vector3d quat2euler(Quaterniond qq) {
  // yaw = euler(0); roll = euler(1); pitch = euler(2);
  Vector3d euler = qq.toRotationMatrix().eulerAngles(2, 0, 1);  // ZXY
  return euler;
}
Quaterniond euler2quat(Vector3d euler) {
  Quaterniond qq = AngleAxisd(euler(0), Vector3d::UnitZ()) *
                   AngleAxisd(euler(1), Vector3d::UnitX()) *
                   AngleAxisd(euler(2), Vector3d::UnitY());
  return qq;
}

Matrix3d skew_symmetric(Vector3d v) {
  Matrix3d m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}