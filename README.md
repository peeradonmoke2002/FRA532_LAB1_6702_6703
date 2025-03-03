
<script type="text/javascript" async
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.7/MathJax.js?config=TeX-MML-AM_CHTML">
</script>


# FRA532 Mobile Robot: LAB1.2

## Table of Contents
<!-- - [PID Controller](#pid-controller)
- [Pure Pursuit Controller](#pure-pursuit-controller) -->

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
<!-- ## PID Controller
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


<!-- 
## yaw rate 

yawrate it very percise  then we can lower Q more than R from imu  -->
<!-- 
### PID 

![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/yawrate/PID/pid_yawrate.png)


### purepursuit 
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/yawrate/purepursuit/pp_yawrate.png)

## single track

single track model heading it  too many error  then we use lower R matrix more than Q matrix


![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/single_track/PID/SingleTrack-PID.png) -->
 -->

# FRA532 Mobile Robot: LAB1.3

## State Vector Representation

The system state is defined as:
$$
\mathbf{X}_k =
\begin{bmatrix}
p_{x,k} \\
p_{y,k} \\
p_{z,k} \\
\text{roll}_k \\
\text{pitch}_k \\
\text{yaw}_k \\
v_{x,k} \\
v_{y,k} \\
v_{z,k} \\
\omega_{x,k} \\
\omega_{y,k} \\
\omega_{z,k} \\
a_{x,k} \\
a_{y,k} \\
a_{z,k}
\end{bmatrix}
$$

$$
\begin{aligned}
p_{x,k}, p_{y,k}, p_{z,k} & \quad \text{represent the position.} \\
\text{roll}_k, \text{pitch}_k, \text{yaw}_k & \quad \text{represent the orientation.} \\
v_{x,k}, v_{y,k}, v_{z,k} & \quad \text{represent the linear velocity.} \\
\omega_{x,k}, \omega_{y,k}, \omega_{z,k} & \quad \text{represent the angular velocity.} \\
a_{x,k}, a_{y,k}, a_{z,k} & \quad \text{represent the linear acceleration.}
\end{aligned}
$$




## State Transition Equations

$$
p_{k+1} = p_k + R(r_k) \left[ v_k dt + \frac{1}{2} a_k dt^2 \right]
$$

$$
r_{k+1} = r_k + J(r_k) \cdot \omega_k \cdot dt
$$

$$
v_{k+1} = v_k + a_k dt
$$

$$
\omega_{k+1} = \omega_k + u_{\alpha} dt
$$

$$
a_{k+1} = a_k
$$

##  How EKF Handles Nonlinearity

EKF is designed to work with nonlinear systems by approximating them linearly at each time step using Jacobian matrices.

### **Jacobian of the State Transition:**
To approximate the system dynamics, EKF computes the Jacobian matrix of the state transition function:

$$
F = \frac{\partial f}{\partial X}
$$

where:
- \( X \) is the state vector
- \( f(X) \) is the nonlinear motion model

### **Jacobian of the Measurement Model:**
Similarly, the measurement function is linearized using the Jacobian:

$$
H = \frac{\partial h}{\partial X}
$$

where:
- \( h(X) \) is the nonlinear measurement function




## Design the Process Noise Matrix

covaraince uncertainty in the motion model.
```bash 
Q = np.diag([
    0.02, 0.02, 0.02,        # position noise
    np.deg2rad(0.1), ...,    # orientation noise
    0.1, 0.1, 0.1,           # velocity noise
    np.deg2rad(0.1), ...,    # angular velocity noise
    0.2, 0.2, 0.2            # linear acceleration noise
]) ** 2

```

## Design the Control Function

```bash
self.u_alpha = np.zeros((3,1))  # Assumed zero if no external angular acceleration

```
Integrated in the state transition to update angular velocity

$$
\omega_{k+1} = \omega_k + u_{\alpha} dt
$$

## Design the Measurement Function


This function `ekf_update_odom(...)` processes a **6D measurement** vector:

$$
\mathbf{z} =
\begin{bmatrix}
p_x \\
p_y \\
p_z \\
v_x \\
v_y \\
v_z
\end{bmatrix}
$$

 EKF Update - Odometry Measurement (6D)

The function `ekf_update_imu(...)` processes a **9D measurement vector**:

$$
\mathbf{z} =
\begin{bmatrix}
\text{roll} \\
\text{pitch} \\
\text{yaw} \\
\omega_x \\
\omega_y \\
\omega_z \\
a_x \\
a_y \\
a_z
\end{bmatrix}
$$


## Design the Measurement Noise Matrix

Captures sensor noise in odometry and IMU measurements.

```bash
R_odom = np.diag([...]) ** 2  # 6x6
R_imu  = np.diag([...]) ** 2  # 9x9

```

## Explain the Kalman Gain Computation
$$
K = P H^T (H P H^T + R)^{-1}
$$

P = Current state covariance (uncertainty in prediction)  
H = Measurement function Jacobian  
R = Measurement noise covariance  
S = H P H^T + R is the innovation covariance.  



### How to Tune \( K \)
- Lower \( R \) → Higher trust in sensor.  
- Lower \( Q \) → Higher trust in model.  
-  Start with large Q R and gradually decrease while monitoring performance.
- Check innovation covariance $$ S=HPH 
T
 +R$$

### imprementation



config in yaml
```bash  


odom0_config: [true,  true,  false,   # Use x, y position from Odometry
               false, false, true,    # Use Yaw from Odometry (Disable Roll, Pitch)
               true, true, false,     # Use velocity (vx, vy) from Odometry
               false, false, true,    # Use vyaw from Odometry
               false, false, false]   # Ignore linear acceleration


imu0_config:  [false, false, false,  # Ignore x, y, z position
               true,  true,  false,   # Use only Roll, Pitch from IMU (Yaw from Odometry)
               false, false, false,  # Ignore linear velocities (vx, vy, vz)
               false, false, false,  # Ignore angular velocity from IMU (Odometry handles it)
               false, false, false]  # Ignore linear acceleration



```
## YAW rate Q and R tuning
- becase odom forn yaw rate it are very percise so we can lower R odom for make it trust odom more then imu 

```bash



Q = np.diag([
    0.02, 0.02, 0.02,            # position noise
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),  # orientation noise (rad) roll pitch yaw
    0.1, 0.1, 0.1,               # linear velocity noise
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),  # angular velocity noise (rad/s)
    0.2, 0.2, 0.2                # linear acceleration noise
]) ** 2

# Measurement noise covariance for odometry (6x6): [p (3), v (3)]
R_odom = np.diag([0.1, 0.1, 0.1, # Position noise (x, y, z)
                   0.1, 0.1, 0.1]) ** 2 # Velocity noise (vx, vy, vz)


# Measurement noise covariance for IMU (9x9): [orientation (3), angular velocity (3), linear acceleration (3)]
R_imu = np.diag([
    np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),# Orientation noise (roll, pitch, yaw)
    np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5),# Angular velocity noise (ωx, ωy, ωz)
    0.2, 0.2, 0.2 # Linear acceleration noise (ax, ay, az)
]) ** 2
``` 
### RESULT

![PID Yaw Rate](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/yawrate/PID/pid-yawrate.png)
![Purepursuit Yaw Rate](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/yawrate/purepursuit/purepursuit_yawrate.png)
![Stanlee Yaw Rate](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/yawrate/stanlee/stanlee-yawrate.png) 
## Single track Q and R tuning

```bash 
Q = np.diag([
    0.02, 0.02, 2.02,            # position noise
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(2.5),  # orientation noise (rad) roll pitch yaw
    0.1, 0.1, 0.1,               # linear velocity noise
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(2.5),  # angular velocity noise (rad/s)
    0.2, 0.2, 0.2                # linear acceleration noise
]) ** 2

# Measurement noise covariance for odometry (6x6): [p (3), v (3)]
R_odom = np.diag([1.0, 1.0, 1.1,# Position noise (x, y, z)
                   1.1, 1.1, 1.1]) ** 2 # Velocity noise (vx, vy, vz)


# Measurement noise covariance for IMU (9x9): [orientation (3), angular velocity (3), linear acceleration (3)]
R_imu = np.diag([
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),# Orientation noise (roll, pitch, yaw)
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),# Angular velocity noise (ωx, ωy, ωz)
    0.2, 0.2, 0.3 # Linear acceleration noise (ax, ay, az)
]) ** 2

```
![PID-single ](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/single_track/PID/PID-singletrack.png)

![pp-single](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/single_track/purepursuit/purepursuit_singletrack.png) 

![stanle-single](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/single_track/stanlee/stanlee_singletrack.png)


## Double  track Q and R tuning

```bash 
Q = np.diag([
    0.02, 0.02, 2.02,            # position noise
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(2.5),  # orientation noise (rad) roll pitch yaw
    0.1, 0.1, 0.1,               # linear velocity noise
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(2.5),  # angular velocity noise (rad/s)
    0.2, 0.2, 0.2                # linear acceleration noise
]) ** 2

# Measurement noise covariance for odometry (6x6): [p (3), v (3)]
R_odom = np.diag([2.0, 2.0, 2.1,# Position noise (x, y, z)
                   2.1, 2.1, 2.1]) ** 2 # Velocity noise (vx, vy, vz)


# Measurement noise covariance for IMU (9x9): [orientation (3), angular velocity (3), linear acceleration (3)]
R_imu = np.diag([
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),# Orientation noise (roll, pitch, yaw)
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),# Angular velocity noise (ωx, ωy, ωz)
    0.2, 0.2, 0.3 # Linear acceleration noise (ax, ay, az)
]) ** 2
```

image
image
image
## Our Team


1. 67340700402 พงษ์พัฒน์ วงศ์กำแหงหาญ
2. 67340700403 พีรดนย์ เรืองแก้ว
