# INS initialization
This repository implements the solution to Inertial Navigation System (INS) initialization. More precisely, the system estimates `roll` and `pitch` angles in addition to accelerometer and gyroscope biases (additionally `yaw` angle if magnetometer data is provided) so as to align the gravity vector direction along the local NED frame. The estimation occurs in two phases: coarse initialization followed by fine initialization. An Extended Kalman Filter (EKF) is used for fine initialization.

For mathematical treatment of the problem, head over to [blog post](https://kvmanohar22.github.io/ins_init).

## Dependencies
- Eigen 3.4+
- numpy (optional)
- matplotlib (optional)
- python3 (optional)
- ROS melodic (optional)

## Usage
I have tested this on Arch Linux and should run out of the box on any Linux based distribution. Feel free to open an issue if faced with any problem.

- Build

```bash
  git clone git@github.com:kvmanohar22/ins_init.git
  cd ins_init/build
  cmake ..
  make
```

- Usage

  This repository can either be used as standalone package or as part of ROS. The input to the system in either case is stream of IMU (3 accelerometers, 3 gyroscopes) data. All the data used in this repository has been generated using pixhawk flight controller 2.4.6.

  - **Stanadalone package**: Sample data is provided under `assets/imu_*.txt` 
  ```bash
    cd ins_init  
    ./bin/ins_init /path/to/txt/file 
  ```

  - **ROS package**: Sample data is provided under `assets/imu_*.bag` 
  ```bash
    rosrun ins_init ins_init bag_path:=/path/to/bag
  ```
  OR
  ```bash
    roslaunch ins_init ins_init.launch bag_path:=/path/to/bag
  ```
  OR

  if you have live stream of data (you might want to change ROS topic names in `launch/ins_init_stream.launch` file,

  ```bash
    roslaunch ins_init ins_init_stream.launch
  ```

  **NOTE**: The sensor has to be kept static during initialization.

