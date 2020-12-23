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
  const double ax = pkt.acc_.x();
  const double ay = pkt.acc_.y();
  const double az = pkt.acc_.z();

  const double roll = atan2(ay, az);
  const double pitch = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2)));

  const double cos_roll = cos(roll);
  const double sin_roll = sin(roll);
  const double sin_pitch = sin(pitch);


  // NOTE: unless a magnetometer is used, this estimate will remain unobserved
  const double yaw = atan2(
     -pkt.gyr_.y() * cos_roll + pkt.gyr_.z() * sin_roll,
      pkt.gyr_.x() * cos(pitch) + pkt.gyr_.y() * sin_pitch * sin_roll + pkt.gyr_.z() * sin_pitch * cos_roll);
  ROS_INFO_STREAM("RPY = " << roll << " " << pitch << " " << yaw);
  R_n_b_ = rz(-yaw) * ry(-pitch) * rx(-roll);

  Vector3d rpy = dcm2rpy(R_n_b_);
  ROS_INFO_STREAM("RPY = " << rpy[0] << " " << rpy[1] << " " << rpy[2]);
  done_ = true;

  return true;
}

} // namespace coarse_init

#endif // _INS_INIT_COARSE_INIT_H_

