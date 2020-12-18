#ifndef _INS_INIT_FINE_INIT_ACC_H_
#define _INS_INIT_FINE_INIT_ACC_H_

#include "ins_init/global.h"
#include "ins_init/ekf.h"
#include <cmath>

namespace ins_init
{

class FineInitAcc : public EKF
{
public:
  FineInitAcc(size_t dim_x, size_t dim_w, size_t dim_z) :
    EKF(dim_x, dim_w, dim_z)
  {}
  virtual ~FineInitAcc() {}

  virtual MatrixNd getStateTransition() override
  {
    // since earth angular velocity is of the order of 1e-5, 
    // it is approximated to zero and first order approximation
    // gives identity matrix
    MatrixNd I(dim_x_, dim_x_); I.setIdentity();
    return I;
  }

  virtual VectorNd getObservationError(const VectorNd& z) override
  {
    // we are assuming the platform to be stationary
    // and this error is the difference between accelerometer values
    // and horizontal acceleration of system (which is presumed zero) 
    return z;
  }

  virtual void update(const double t, const VectorNd& z) override
  {
    // get observation error
    VectorNd obs_error = getObservationError(z);
    if(obs_error.hasNaN())
      ROS_WARN_STREAM_THROTTLE(2.0, "Nan detected in observation error: " << obs_error.transpose());

    // reset the state
    x_.setZero();

    // propagate covariance.
    const double dt = t - prev_t_;
    const double delta_t = t - t0_; 
    const double g2 = GR*GR;
    P_(0,0) = (R_(0,0)/(g2*delta_t/dt))*(1.0/(1.0 + (R_(0,0)*dt)/(g2*delta_t*P0_(0,0))));
    P_(1,1) = (R_(1,1)/(g2*delta_t/dt))*(1.0/(1.0 + (R_(1,1)*dt)/(g2*delta_t*P0_(1,1))));
    P_(2,2) = P0_(2,2);

    // kalman gain 
    MatrixNd K = P_*H_.transpose()*(R_ + H_*P_*H_.transpose()).inverse();
    if(K.hasNaN())
      ROS_WARN_STREAM_THROTTLE(2.0, "Nan detected in Kalman Gain.");
 
    // update state
    // WARNING: This state will be reset before next update. 
    x_ = x_ + K * obs_error; 

    if(x_.hasNaN())
      ROS_WARN_STREAM_THROTTLE(2.0, "Nan detected in state.");

    // update time
    prev_t_ = t;
  }

}; // class FineInitAcc

} // namespace ins_init

#endif // _INS_INIT_FINE_INIT_ACC_H_
