# pose_ekf

An Error-State Kalman Filter (ESKF) for 6D pose estimation using GPS, IMU, and Magnetometer data. This package also includes a built-in sensor simulator for generating noisy synthetic data for testing.

![image](https://github.com/libing64/pose_ekf/blob/master/pose_ekf.gif)

## 1. Filter Formulation

**Nominal State:** 15-DOF
```
[p, v, q, ba, bw]
```
- `p`: Position (3D)
- `v`: Velocity (3D)
- `q`: Quaternion Attitude (4D)
- `ba`: Accelerometer Bias (3D)
- `bw`: Gyroscope Bias (3D)

**Error State:**
```
[dp, dv, d_theta, dba, dbw]
```

## 2. Coordinate Frame

The inertial frame used in the filter is **NED** (North-East-Down).
- X: North
- Y: East
- Z: Down (aligned with Gravity)

---

## 3. Installation & Building

### 3.1 Dependencies
- **ROS 1** (e.g., Noetic, Melodic)
- **Eigen3**
- **RTKLIB** (A subset of headers and static libraries are required. Expected `rtklib.h` in `/usr/local/include/rtklib` or locally).

### 3.2 Build Instructions

Clone the repository into your ROS workspace and build:

```bash
mkdir -p ~/sensor_fusion_ws/src
cd ~/sensor_fusion_ws/src
git clone git@github.com:libing64/pose_ekf.git
cd ~/sensor_fusion_ws

# Build the package
catkin_make -DCATKIN_WHITELIST_PACKAGES="pose_ekf"

# IMPORTANT: Source the workspace environment
source devel/setup.bash
```

---

## 4. Running the Package

### 4.1 Running with the Built-in Simulator

The package comes with a built-in trajectory generator and sensor noise simulator. This is the easiest way to verify that the ESKF is working correctly.

```bash
# Make sure you have sourced the workspace
source devel/setup.bash

# Launch the simulator, EKF node, and RViz
roslaunch pose_ekf pose_ekf_simulator.launch
```

What this launch file does:
1. Starts the `sensor_simulator` node, which generates synthetic Ground Truth and noisy `/imu`, `/fix`, `/fix_velocity`, and `/magnetic` data.
2. Starts the `pose_eskf` node, which subscribes to these topics and performs state estimation.
3. Opens `RViz` to visualize the Ground Truth Path (from simulator) vs the Estimated Path (from EKF).

### 4.2 Running with Real Data or Rosbag (TODO)

To use your own rosbag or real sensors, you can create a custom launch file or run the node directly, remapping the input topics to match your dataset:

```bash
rosrun pose_ekf pose_eskf \
    _imu:=/your_imu_topic \
    _fix:=/your_gps_fix_topic \
    _fix_velocity_v3:=/your_gps_vel_topic \
    _magnetic_field_v3:=/your_mag_topic
```

*(Note: Ensure that your real sensor data is transformed into the NED coordinate frame before feeding it into the EKF, or adjust the observation models accordingly.)*

## 5. Architecture Documentation
For a detailed breakdown of the codebase structure, mathematical formulations, and data flow, please refer to the architecture documents:
- [ARCHITECTURE.md](ARCHITECTURE.md) (English)
- [ARCHITECTURE_zh.md](ARCHITECTURE_zh.md) (Chinese)
