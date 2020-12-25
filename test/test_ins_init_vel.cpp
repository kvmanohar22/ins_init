#include "ins_init/coarse_init.h"
#include "ins_init/fine_init_vel.h"
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

/// Uses accelerometer data to generate velocity observations
class TestInsInitVelObs
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TestInsInitVelObs(ros::NodeHandle& nh);
  virtual ~TestInsInitVelObs();

  // ROS callback function for imu messages
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

  // Main function to estimate the state of system
  void feedImu(const ImuPacket& packet);

  // initialize kalman filter's parameters
  void initKF();

  inline bool quit() const { return quit_; }
  inline ros::Rate rate() const { return rate_; }

  void writeLog(const double t, Vector3d& rpy, MatrixNd& P);

private:
  Matrix3d          R_n_b_;       //!< Estimated rotation matrix from (b)ody frame to (n)ED frame
  ros::NodeHandle   nh_;
  ros::Subscriber   imu_sub_; 
  bool              quit_;        //!< set to true to quit the loop
  ros::Rate         rate_;
  CoarseInit*       coarse_init_;
  FineInitVel*      fine_init_;
  double            prev_t_;
  double            t0_;
  std::ofstream     ofs_;
  bool              start_;
}; // class TestInsInitVelObs

TestInsInitVelObs::TestInsInitVelObs(ros::NodeHandle& nh)
  : nh_(nh),
    quit_(false),
    rate_(1000),
    prev_t_(0.0),
    t0_(0.0),
    start_(false)
{
  imu_sub_ = nh_.subscribe("/mavros/imu/data_raw", 1000, &TestInsInitVelObs::imuCb, this);
  coarse_init_ = new CoarseInit(60.0);
  fine_init_ = new FineInitVel(8, 0, 2);

  initKF();

  // open file for writing debug data and add header
  ofs_.open("/tmp/ins_init.csv");
  ofs_ << "time" << ","
       << "cov_rr" << "," << "cov_pp" << ","<< "cov_yy" << "," /* covariance of attitude */
       << "cov_nn" << "," << "cov_ee" << ","<< "cov_dd" << "," /* covariance of bias in NED frame */
       << "roll" << "," << "pitch" << "," << "yaw" << ","      /* attitude */
       << "bN" << "," << "bE" << "," << "bD" << ","            /* gyroscope bias in NED frame */
       << "vN" << "," << "vE" << "\n";                         /* horizontal velocity in NED frame */
}

void TestInsInitVelObs::writeLog(const double t, Vector3d& rpy, MatrixNd& P)
{
  // convert to degrees
  rpy = rpy * 180/PI;
  for(int i=0; i<8; ++i)
  {
    P(i,i) = sqrt(P(i,i));
    if(i < 3)
      P(i,i) *= (180/PI);
  }

  const Vector3d gy_b = fine_init_->getGyroBias();
  const Vector2d vel  = fine_init_->getVelocity();
  ofs_ << t << ","
       << P(0,0) << "," << P(1,1) << ","<< P(2,2) << ","
       << P(3,3) << "," << P(4,4) << ","<< P(5,5) << ","
       << rpy[0] << "," << rpy[1] << ","<< rpy[2] << ","
       << gy_b[0] << "," << gy_b[1] << ","<< gy_b[2] << ","
       << vel[0] << "," << vel[1] << "\n";

  ROS_DEBUG_STREAM_THROTTLE(1.0,
      "cov = " << P(0,0) << " " << P(1,1) << " " << P(2,2) << "\t"
      "rpy = " << rpy[0] << " " << rpy[1] << " " << rpy[2]);
}

TestInsInitVelObs::~TestInsInitVelObs()
{
  ofs_.close();
  delete coarse_init_;
  delete fine_init_;
}

void TestInsInitVelObs::initKF()
{
  const double deg1 = PI/180.0; // 1 degree
  const double init_var_sq = deg1*deg1;
  for(int i=0; i<3; ++i)
  {
    fine_init_->P()(i,i)  = init_var_sq;
    fine_init_->P0()(i,i) = init_var_sq;
  }
  const double bias_std = PI/180.0 * (1/3600.0); // 1 deg/hr
  const double bias_var = bias_std*bias_std;
  for(int i=3; i<6; ++i)
  {
    fine_init_->P()(i,i)  = bias_var;
    fine_init_->P0()(i,i) = bias_var;
  }
  const double vel_std = 0.1; // 0.1 m/s 
  const double vel_var = vel_std*vel_std;
  for(int i=6; i<8; ++i)
  {
    fine_init_->P()(i,i)  = vel_var;
    fine_init_->P0()(i,i) = vel_var;
  }

  // observation 
  fine_init_->H().topRightCorner(2, 2) = Matrix2d::Identity();

  // dynamic matrix 
  const double phi = 28.0 * PI / 180.0;
  const double r = 6371000; 
  const double cos_phi = cos(phi); 
  const double sin_phi = sin(phi); 
  fine_init_->F().topLeftCorner(3,3) = ins_init::sqew(Vector3d(-WE*cos_phi, 0.0, WE*sin_phi));
  fine_init_->F().block(0, 3, 3, 3) = -Matrix3d::Identity();
  fine_init_->F()(0, 7) =  1.0/r;
  fine_init_->F()(1, 6) = -1.0/r;
  fine_init_->F()(2, 7) = tan(phi)/r;
  
  fine_init_->F()(6, 1) =  GR;
  fine_init_->F()(7, 0) = -GR;
  fine_init_->F()(6, 7) = -2.0 * WE * sin_phi;
  fine_init_->F()(7, 6) =  2.0 * WE * sin_phi;

  // 0.1 m/s^2 acceleration std-dev
  fine_init_->R()(0,0) = vel_var;
  fine_init_->R()(1,1) = vel_var;
}

void TestInsInitVelObs::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO_STREAM_ONCE("Imu callback started.");

  Vector3d omg(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  ImuPacket packet(
      omg, acc, msg->header.stamp.toSec());
  feedImu(packet);
}

void TestInsInitVelObs::feedImu(const ImuPacket& packet)
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

      MatrixNd cov = fine_init_->getStateCov();
      Vector3d rpy = dcm2rpy(R_n_b_);
      writeLog(0.0, rpy, cov);
    }
    prev_t_ = packet.t_;
    t0_ = packet.t_;
    return;
  }

  // 2. fine init
  const double dt = packet.t_ - prev_t_; 

  // 2.1.1: Kalman filter predict step 
  fine_init_->predict(dt);
  
  // 2.1.2: predict nominal velocity state
  Vector3d acc_n = R_n_b_ * packet.acc_;
  fine_init_->predictNominalVel(dt, acc_n);

  // 2.2: update with new measurement
  //      observations are velocity in NED frame
  const Vector2d vel = fine_init_->getVelocity(); 
  fine_init_->update(packet.t_, vel);

  // 2.3: update nominal state
  const VectorNd state = fine_init_->getState();
  const Vector3d d_theta =  state.block(0, 0, 3, 1);
  R_n_b_ = ins_init::getRotationMatrix(d_theta) * R_n_b_; /* attitude */
  fine_init_->closeVelocityLoop(state.tail(2));           /* velocity */
  fine_init_->closeGyroBiasLoop(state.block(3, 0, 3, 1)); /* bias     */

  // re-normalize attitude
  // ROS_DEBUG_STREAM("rtr(before) = " << R_n_b_.transpose() * R_n_b_);
  // Eigen::Quaterniond q(R_n_b_); q.normalize();
  // R_n_b_ = q.toRotationMatrix();
  ROS_DEBUG_STREAM("rtr(after) = " << R_n_b_.transpose() * R_n_b_);

  // 2.4: reset state
  fine_init_->resetState(); 

  // for computing dt
  prev_t_ = packet.t_;

  // write to file
  MatrixNd cov = fine_init_->getStateCov();
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
  TestInsInitVelObs estimator(nh);
  while(ros::ok() && !estimator.quit())
  {
    ros::spinOnce();
    estimator.rate().sleep();
  }
  ROS_INFO_STREAM("Node killed.");
}

