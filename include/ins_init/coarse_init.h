#ifndef _INS_INIT_COARSE_INIT_H_
#define _INS_INIT_COARSE_INIT_H_

#include "ins_init/global.h"
#include "ins_init/imu.h"

namespace ins_init
{

class CoarseInit
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CoarseInit();
  virtual ~CoarseInit();

  /// start the init procedure
  bool init(const ImuPacket& imu);

  inline Matrix3d getR() const { return R_n_b_; }

private:
  Matrix3d R_n_b_; //<! Rotation matrix from (b)ody to (n)ED.

}; // class CoarseInit

} // namespace coarse_init

#endif // _INS_INIT_COARSE_INIT_H_

