#ifndef __ESKF_H
#define __ESKF_H
#include <iostream>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class eskf {
 public:
  int n;
  VectorXd dx;
  MatrixXd P;

  eskf();
  eskf(int n);
  ~eskf();
  void eskf_init_P(VectorXd p);
  void update(VectorXd v, MatrixXd H, MatrixXd R);
  VectorXd get_state() { return dx; };
};

#endif