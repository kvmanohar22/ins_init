#include "ins_init/coarse_init.h"
#include "ins_init/fine_init_acc.h"
#include "ins_init/imu.h"
#include "ins_init/utils.h"

#include <ios>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>

namespace ins_init
{

/// Uses accelerometer data as observations and estimates roll pitch and yaw angles
/// This doesn't use magnetometer data and hence yaw estimation is uncertain
class TestInsInitAccObs
{
public:
  TestInsInitAccObs(ros::NodeHandle& nh);
  virtual ~TestInsInitAccObs();

  // ROS callback function for imu messages
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

  // Main function to estimate the state of system
  void feedImu(const ImuPacket& packet);

  // initialize kalman filter's parameters
  void initKF();

  inline bool quit() const { return quit_; }
  inline ros::Rate rate() const { return rate_; }

  void writeLog(const double t, Vector3d& rpy, Matrix3d& P);

private:
  Matrix3d          R_n_b_;       //!< Estimated rotation matrix from (b)ody frame to (n)ED frame
  ros::NodeHandle   nh_;
  ros::Subscriber   imu_sub_; 
  bool              quit_;        //!< set to true to quit the loop
  ros::Rate         rate_;
  CoarseInit*       coarse_init_;
  FineInitAcc*      fine_init_;
  double            prev_t_;
  double            t0_;
  std::ofstream     ofs_;
  bool              start_;
}; // class TestInsInitAccObs

TestInsInitAccObs::TestInsInitAccObs(ros::NodeHandle& nh)
  : nh_(nh),
    quit_(false),
    rate_(1000),
    prev_t_(0.0),
    t0_(0.0),
    start_(false)
{
  imu_sub_ = nh_.subscribe("/mavros/imu/data_raw", 1000, &TestInsInitAccObs::imuCb, this);
  coarse_init_ = new CoarseInit(60.0);
  fine_init_ = new FineInitAcc(3, 0, 3);

  initKF();

  // open file for writing debug data and add header
  ofs_.open("/tmp/ins_init.csv");
  ofs_ << "time" << ","
       << "cov00" << "," << "cov11" << ","<< "cov22" << ","
       << "roll" << "," << "pitch" << "," << "yaw" << "\n";
}

void TestInsInitAccObs::writeLog(const double t, Vector3d& rpy, Matrix3d& P)
{
  // convert to degrees
  rpy = rpy * 180/PI;
  P(0,0) = sqrt(P(0,0)) * 180 / PI;
  P(1,1) = sqrt(P(1,1)) * 180 / PI;
  P(2,2) = sqrt(P(2,2)) * 180 / PI;

  ofs_ << t << ","
       << P(0,0) << "," << P(1,1) << ","<< P(2,2) << ","
       << rpy[0] << "," << rpy[1] << ","<< rpy[2] << "\n";

  ROS_DEBUG_STREAM_THROTTLE(1.0,
      "cov = " << P(0,0) << " " << P(1,1) << " " << P(2,2) << "\t"
      "rpy = " << rpy[0] << " " << rpy[1] << " " << rpy[2]);
}

TestInsInitAccObs::~TestInsInitAccObs()
{
  ofs_.close();
  delete coarse_init_;
  delete fine_init_;
}

void TestInsInitAccObs::initKF()
{
  const double deg1 = PI/180.0;
  const double init_var_sq = deg1*deg1;
  fine_init_->P()(0,0) = init_var_sq;
  fine_init_->P()(1,1) = init_var_sq;
  fine_init_->P()(2,2) = init_var_sq;

  fine_init_->P0()(0,0) = init_var_sq;
  fine_init_->P0()(1,1) = init_var_sq;
  fine_init_->P0()(2,2) = init_var_sq;

  const double phi = 60. * PI / 180.0;
  fine_init_->F() = ins_init::sqew(Vector3d(-WE*cos(phi), 0.0, WE*sin(phi)));
  fine_init_->H() = ins_init::sqew(Vector3d(0.0, 0.0, -GR));

  // 0.1 m/s^2 acceleration std-dev
  fine_init_->R()(0,0) = 1e-2; 
  fine_init_->R()(1,1) = 1e-2; 
  fine_init_->R()(2,2) = 1e-2; 
}

void TestInsInitAccObs::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO_STREAM_ONCE("Imu callback started.");

  Vector3d omg(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  ImuPacket packet(
      omg, acc, msg->header.stamp.toSec());
  feedImu(packet);
}

void TestInsInitAccObs::feedImu(const ImuPacket& packet)
{
  // 0. skip some initial readings
  ROS_INFO_STREAM_ONCE("Skipping 2s worth of data.");
  if(!start_)
  {
    t0_ = packet.t_;
    start_ = true;
    return;
  } else {
    if((packet.t_ - t0_) < 2.0)
      return;
  }

  // 1. coarse init
  if(!coarse_init_->done())
  {
    if(coarse_init_->init(packet))
    {
      ROS_DEBUG_STREAM("Coarse init done.");
      R_n_b_ = coarse_init_->getR();
      ROS_DEBUG_STREAM("Estimated RPY (degree) = " << dcm2rpy(R_n_b_).transpose()*180.0/PI);
      fine_init_->setInitTime(packet.t_); 

      Matrix3d cov = fine_init_->getStateCov();
      Vector3d rpy = dcm2rpy(R_n_b_);
      writeLog(0.0, rpy, cov);
    }
    prev_t_ = packet.t_;
    t0_ = packet.t_;
    return;
  }

  // 2. fine init
  const double dt = packet.t_ - prev_t_; 

  // 2.1: Kalman filter predict step 
  fine_init_->predict(dt);
  // 2.2: update with new measurement
  //      observations are accelerations in NED frame
  Vector3d acc_n = R_n_b_ * packet.acc_;
  fine_init_->update(packet.t_, acc_n);

  // 2.3: update nominal state
  Vector3d dx = fine_init_->getState();
  R_n_b_ = ins_init::getRotationMatrix(dx) * R_n_b_;

  // 2.4: reset state
  fine_init_->resetState(); 

  // for computing dt
  prev_t_ = packet.t_;

  Matrix3d cov = fine_init_->getStateCov();

  // write to file
  Vector3d rpy = ins_init::dcm2rpy(R_n_b_);
  writeLog(packet.t_ - t0_, rpy, cov);
}

} // namespace ins_init

using namespace ins_init;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ins_init");

  ROS_INFO_STREAM("Starting ins_init node");
  ros::NodeHandle nh;
  TestInsInitAccObs estimator(nh);
  while(ros::ok() && !estimator.quit())
  {
    ros::spinOnce();
    estimator.rate().sleep();
  }
  ROS_INFO_STREAM("Node killed.");
}

