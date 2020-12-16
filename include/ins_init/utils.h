#ifndef _INS_INIT_UTILS_H_
#define _INS_INIT_UTILS_H_

#include "ins_init/global.h"

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

} // namespace ins_init

#endif // _INS_INIT_UTILS_H_

