#include "ins_init/coarse_init.h"
#include "ins_init/fine_init_acc.h"
#include "ins_init/imu.h"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

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

  inline bool quit() const { return quit_; }
  inline ros::Rate rate() const { return rate_; }

private:
  Matrix3d          R_n_b_;       //!< Estimated rotation matrix from (b)ody frame to (n)ED frame
  ros::NodeHandle   nh_;
  ros::Subscriber   imu_sub_; 
  bool              quit_;        //!< set to true to quit the loop
  ros::Rate         rate_;
  CoarseInit*       coarse_init_;
}; // class TestInsInitAccObs

TestInsInitAccObs::TestInsInitAccObs(ros::NodeHandle& nh)
  : nh_(nh),
    quit_(false),
    rate_(1000)
{
  imu_sub_ = nh_.subscribe("/mavros/imu/data_raw", 1000, &TestInsInitAccObs::imuCb, this);
  coarse_init_ = new CoarseInit(60.0);
}

TestInsInitAccObs::~TestInsInitAccObs()
{
  delete coarse_init_;
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
  // 1. coarse init
  if(!coarse_init_->done())
  {
    if(coarse_init_->init(packet))
    {
      ROS_DEBUG_STREAM("Coarse init done.");
      R_n_b_ = coarse_init_->getR();
    }
  }

  // 2. fine init

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

