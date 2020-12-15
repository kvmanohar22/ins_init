#ifndef _INS_INIT_IMU_H_
#define _INS_INIT_IMU_H_

#include "ins_init/global.h"

namespace ins_init
{

class ImuPacket
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

  Vector3d gyr_;    //!< gyroscope data
  Vector3d acc_;    //!< accelerometer data
  double   t_;      //!< timestamp

  ImuPacket(const Vector3d& gyr, const Vector3d& acc, double t)
    : gyr_(gyr),
      acc_(acc),
      t_(t)
  {}
  virtual ~ImuPacket();

}; // class ImuPacket

typedef list<ImuPacket> ImuStream;

} // namespace ins_init


#endif // _INS_INIT_IMU_H_
