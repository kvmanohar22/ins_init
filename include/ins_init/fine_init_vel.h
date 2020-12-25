#ifndef _INS_INIT_FINE_INIT_VEL_H_
#define _INS_INIT_FINE_INIT_VEL_H_

#include "ins_init/global.h"
#include "ins_init/ekf.h"
#include "ins_init/imu.h"
#include <cmath>

namespace ins_init
{

class FineInitVel final : public EKF
{
public:
  FineInitVel(size_t dim_x, size_t dim_w, size_t dim_z) :
    EKF(dim_x, dim_w, dim_z)
  {}
  virtual ~FineInitVel() {}

  inline Vector2d getVelocity() const { return vel_; }
  inline Vector3d getGyroBias() const { return bias_; }

  /// Predict horizontal velocity
  /// Input is raw acceleration in NED frame
  void predictNominalVel(const double dt, const Vector3d& acc_n)
  {
    // gravity doesn't affect horizontal velocity in NED frame
    // & hence excluded here.
    // Further, accelerometer errors are neglected hence no bias compensation is done
    vel_ = vel_ + acc_n.head(2) * dt;
  }

  /// Close the loop by compensating with bias error
  void closeGyroBiasLoop(const Vector3d& db)
  {
    bias_ += db;
  }

  /// Close the loop by compensating with velocity error
  void closeVelocityLoop(const Vector2d& dv)
  {
    vel_ += dv;
  }

  virtual MatrixNd getStateTransition() override
  {
    MatrixNd phi;
    // TODO

    return phi;
  }

  virtual VectorNd getObservationError(const VectorNd& z) override
  {
    // TODO
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

    // TODO: kalman gain 
    MatrixNd K;
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

private:
  Vector2d vel_;  //!< velocity in NED frame. only NE components are estimated
  Vector3d bias_; //!< gyroscope bias in NED frame
}; // class FineInitVel

} // namespace ins_init

#endif // _INS_INIT_FINE_INIT_VEL_H_
