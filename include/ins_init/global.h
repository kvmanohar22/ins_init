#ifndef _INS_INIT_GLOBAL_H_
#define _INS_INIT_GLOBAL_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#include <cassert>
#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <chrono>

#include <ros/ros.h>

namespace ins_init
{
  using namespace std;
  using namespace Eigen;

  static constexpr double PI = 3.1415926535;  //!< pi
  static constexpr double GR = 9.780327;      //!< gravity vector magnitude
  static constexpr double WE = 7.292115e-5;   //!< angular velocity of earth [rad/s]

  typedef Eigen::Matrix<double, 3, 3> Matrix3d;
  typedef Eigen::Matrix<double, 2, 2> Matrix2d;
  typedef Eigen::Matrix<double, 3, 1> Vector3d;
  typedef Eigen::Matrix<double, 2, 1> Vector2d;

  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorNd;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixNd;

} // namespace ins_init

#endif
