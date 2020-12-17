#ifndef _INS_INIT_UTILS_H_
#define _INS_INIT_UTILS_H_

#include "ins_init/global.h"
#include <math.h>

namespace ins_init
{

// skew symmetric matrix representation of a vector
inline Matrix3d sqew(const Vector3d& v)
{
  Matrix3d m;
  m <<      0, -v.z(),  v.y(),
        v.z(),      0, -v.x(),
       -v.y(),  v.x(),      0;
  return m;
}

// first order approximation of rotation
inline Matrix3d getRotationMatrix(const Vector3d& v)
{
  return Matrix3d::Identity() - sqew(v);
}

inline Matrix3d getRotationMatrix(const Matrix3d& m)
{
  return Matrix3d::Identity() - m;
}

/// Converts Direction cosine matrix to roll, pitch and yaw
inline Vector3d dcm2rpy(const Matrix3d& R)
{
  Vector3d rpy;
  rpy[1] = atan2( -R(2,0), sqrt( pow( R(0,0), 2 ) + pow( R(1,0), 2 ) ) );
  if( fabs( rpy[1] - M_PI/2 ) < 0.00001 )
  {
    rpy[2] = 0;
    rpy[0] = -atan2( R(0,1), R(1,1) );
  }
  else
  {
    if( fabs( rpy[1] + M_PI/2 ) < 0.00001 )
    {
      rpy[2] = 0;
      rpy[0] = -atan2( R(0,1), R(1,1) );
    }
    else
    {
      rpy[2] = atan2( R(1,0)/cos(rpy[1]), R(0,0)/cos(rpy[1]) );
      rpy[0] = atan2( R(2,1)/cos(rpy[1]), R(2,2)/cos(rpy[1]) );
    }
  }
  return rpy;
}

} // namespace ins_init

#endif // _INS_INIT_UTILS_H_

