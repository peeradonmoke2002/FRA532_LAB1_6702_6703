
# FRA532 Mobile Robot: LAB1.2

## Table of Contents
- [PID Controller](#pid-controller)
- [Pure Pursuit Controller](#pure-pursuit-controller)

## Installation

### Step 1: Create Workspace
```bash
mkdir ros2_lab1_m 
cd ros2_lab1_m 
mkdir src
cd src
git clone https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703.git -b Path-Tracking-Controller
cd ..
colcon build 
source install/setup.bash
export ROS_WORKSPACE=~/ros2_lab1_m\
source  ~/.bashrc
cd src/FRA532_LAB1_6702_6703/path_tracking/scripts

python3 pid.py
python3 purepursuit.py
python3 mpc.py
```
## PID Controller
PID it separate to steering and speed 

| Parameter | Description |
|-----------|------------|
| `Kp_steer` | Improves robot response to direction changes. |
| `Ki_steer` | Helps correct long-term drift in steering. |
| `Kd_steer` | Dampens sudden steering adjustments to reduce oscillation. |

| Parameter | Description |
|-----------|------------|
| `Kp_speed` | Controls speed response to target velocity. |
| `Ki_speed` | Corrects long-term speed drift. |
| `Kd_speed` | Smooths speed changes for stability. |

The formula of the PID controller is 

$$ u(t) = K_p e(t) + K_I \int_0^t e(t) \,dt + K_d \frac{de}{dt} $$

<!-- ## PID Controller Results
### Path Tracking
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/PID/PID_path.png)

### Speed Profile
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/PID/PID_speed.png)

---

## Pure Pursuit Controller


## Pure Pursuit Controller Results

### Path Tracking

![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/purepursuit/purepursuit_path.png)

### Speed Profile
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/purepursuit/purepursuit_speed.png)


## MPC Controller

## MPC Controller Results

### Path Tracking
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/MPC/mpc_path.png)

### Speed Profile
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/MPC/mpc_speed.png) -->

# FRA532 Mobile Robot: LAB1.3



## yaw rate 

yawrate it very percise  then we can lower Q more than R from imu 

### PID 

![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/yawrate/PID/pid_yawrate.png)


### purepursuit 
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/yawrate/purepursuit/pp_yawrate.png)

## single track

single track model heading it  too many error  then we use lower R matrix more than Q matrix


![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/single_track/PID/SingleTrack-PID.png)

## Our Team


1. 67340700402 พงษ์พัฒน์ วงศ์กำแหงหาญ
2. 67340700403 พีรดนย์ เรืองแก้ว
