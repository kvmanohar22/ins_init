#ifndef _INS_INIT_EKF_H_
#define _INS_INIT_EKF_H_

#include "ins_init/global.h"

namespace ins_init
{

/// Implements abstract Extended Kalman Filter using errors as state of the system
/// i.e, Error State Kalman Filter (ESKF)
class EKF
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EKF(size_t dim_x, size_t dim_w, size_t dim_z);
  virtual ~EKF() {}

  inline size_t dimX() const { return dim_x_; }
  inline size_t dimW() const { return dim_w_; }
  inline size_t dimZ() const { return dim_z_; }

  inline VectorNd getState() const { return x_; }
  inline MatrixNd getStateCov() const { return P_; }
  inline MatrixNd getFisherInformation() const { return P_.inverse(); }

  inline MatrixNd& P() { return P_; }
  inline MatrixNd& F() { return F_; }
  inline MatrixNd& G() { return G_; }
  inline MatrixNd& H() { return H_; }
  inline MatrixNd& Q() { return Q_; }
  inline MatrixNd& R() { return R_; }
  inline MatrixNd& P0() { return P0_; }

  /// propagates state
  void propagateState();

  /// propagates covariance
  void propagateCovariance();

  /// predict step
  void predict(const double dt);

  /// this must be implemented in the derived class. Typically first order approximation
  virtual MatrixNd getStateTransition() =0;

  /// gets observation error. Need to be implemented in derived class
  virtual VectorNd getObservationError(const VectorNd& z) =0;

  /// Needs to be implemented. Either discrete/continuous propagation of covariance
  virtual void update(const double t, const VectorNd& z) =0;

  /// set times
  void inline setInitTime(const double t) { prev_t_ = t; t0_ = t; }

  /// reset the state
  void inline resetState() { x_.setZero(); }

protected:
  size_t dim_x_;  //!< dimension of state 
  size_t dim_w_;  //!< dimension of noise 
  size_t dim_z_;  //!< dimension of observations 
  double prev_t_; //!< current time
  double t_;      //!< current time. WARNING: This is not epoch time!
  double t0_;     //!< initial time

  VectorNd x_;    //!< state of the system
  MatrixNd P_;    //!< covariance of state of the system
  MatrixNd F_;    //!< coefficient of state of the system 
  MatrixNd G_;    //!< coefficient of (white) noise terms
  MatrixNd H_;    //!< (linearized) measurement function
  MatrixNd Q_;    //!< covariance matrix of state propagation
  MatrixNd R_;    //!< covariance matrix of measurement
  MatrixNd P0_;   //!< initial covariance matrix
}; // class EKF

} // namespace ins_init

#endif // _INS_INIT_EKF_H_

