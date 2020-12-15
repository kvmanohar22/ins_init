#include "ins_init/ekf.h"

namespace ins_init
{

EKF::EKF(size_t dim_x,
    size_t dim_w,
    size_t dim_z) :
  dim_x_(dim_x),
  dim_w_(dim_w),
  dim_z_(dim_z),
  t_(0.0)
{
  // set the correct dimensions for states  
  x_.resize(dim_x_);
  P_.resize(dim_x_, dim_x_);  
  
  F_.resize(dim_x_, dim_x_);  
  G_.resize(dim_x_, dim_w_);  
  H_.resize(dim_z_, dim_x_);  
  Q_.resize(dim_w_, dim_w_);  
  R_.resize(dim_z_, dim_z_);  
}

void EKF::propagateState()
{
  const MatrixNd phi = getStateTransition();
  x_ = phi * x_;
}

void EKF::propagateCovariance()
{
  const MatrixNd phi = getStateTransition();
  P_ = phi * P_ * phi.transpose() + G_ * Q_ * G_.transpose();
}

void EKF::predict(const double dt)
{
  t_ += dt;
  propagateState();
  propagateCovariance();
}

} // namespace ins_init

