#ifndef _INS_INIT_COARSE_INIT_H_
#define _INS_INIT_COARSE_INIT_H_

#include "ins_init/global.h"
#include "ins_init/imu.h"
#include "ins_init/utils.h"

namespace ins_init
{

// Input is the latitude in degrees
class CoarseInit
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CoarseInit(double phi)
    : phi_(phi*PI/180.0),
      done_(false)
  {}
  virtual ~CoarseInit() {}

  /// start the init procedure
  bool init(const ImuPacket& imu);

  inline Matrix3d getR() const { return R_n_b_; }
  inline bool done() const { return done_; }

private:
  Matrix3d R_n_b_; //<! Rotation matrix from (b)ody to (n)ED.
  double   phi_;   //<! latitude
  bool     done_;  //<! set to true if estimated
}; // class CoarseInit

bool CoarseInit::init(const ImuPacket& pkt)
{
  Matrix3d T;
  T << 0.0, 0.0, -GR,
       WE*cos(phi_), 0.0, -WE*sin(phi_),
       0.0, GR*WE*cos(phi_), 0.0;

  // angular velocity of body wrt inertial frame in body frame
  Vector3d w_i_b = pkt.gyr_;

  // acceleration in body frame
  Vector3d a_b = pkt.acc_;

  Matrix3d M; M << a_b.transpose(), w_i_b.transpose(), (sqew(w_i_b)*a_b).transpose();
  R_n_b_ = T.inverse() * M;
  done_ = true;

  return true;
}

} // namespace coarse_init

#endif // _INS_INIT_COARSE_INIT_H_

