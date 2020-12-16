#ifndef _INS_INIT_READER_H_
#define _INS_INIT_READER_H_

#include "ins_init/imu.h"
#include <fstream>
#include <string>


namespace ins_init
{

/// Given a text file, it reads the IMU readings
class ReadImuText
{
public:
  ReadImuText() {}
  virtual ~ReadImuText() {}

  static ImuStream read(const string path)
  {
    ImuStream stream;
    std::ifstream ofs(path.c_str());
    if(!ofs.is_open())
    {
      cerr << "Couldn't open the file: " << path << endl;
      return stream; 
    }

    // Each line: (ts, acc_x, acc_y, acc_z, w_x, w_y, w_z) 
    string s;
    while(!ofs.eof())
    {
      ImuPacket imu; 
      std::getline(ofs, s, ',');
      imu.t_ = atof(s.c_str()); 
 
      for (int i = 0; i < 3; ++i)
      {
        std::getline(ofs, s, ',');
        imu.acc_(i) = atof(s.c_str()); 
      }
      
      for (int i = 0; i < 3; ++i)
      {
        std::getline(ofs, s, ',');
        imu.gyr_(i) = atof(s.c_str()); 
      }
      stream.push_back(imu);   
    }
    return stream;
  }

}; // class ReadImuText

} // namespace ins_init

#endif // _INS_INIT_READER_H_

