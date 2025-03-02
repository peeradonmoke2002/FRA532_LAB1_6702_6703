# FRA532 Mobile Robot

## Table of Contents
- [System Overview](#system-overview)
- [Installation](#installation)
- [LAB 1.1](#lab-11)
    - [Inverse Kinematics](#inverse-kinematics)
    - [Forward Kinematics](#forward-kinematics)
    - [Methodology and Results](#methodology-and-results)
- [LAB 1.2](#lab-12)
    - [PID Controller](#pid-controller)
    - [Pure Pursuit Controller](#pure-pursuit-controller)
    - [Methodology and Results](#methodology-and-results)


## System Overview
![Personal diagram - FRA532 Mobile Robot-2](https://github.com/user-attachments/assets/e4a6f163-1add-4d8e-b1a1-1536304b5301)



## Installation

### Step 1: Create Workspace
```bash
mkdir -p ~/FRA532_MobileRobot/src
```

### Step 2: Clone the Repository
```bash
cd ~/FRA532_MobileRobot/src
git clone https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703.git
```

### Step 3: Build Workspace
```bash
cd ~/FRA532_MobileRobot
colcon build
source install/setup.bash
```

### Testing Rviz View and Gazebo Simulation

1) Build workspace

```bash
cd ~/FRA532_MobileRobot

sudo apt update

sudo apt install -y python3-rosdep

rosdep update --rosdistro=humble

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

colcon build

source install/setup.bash
```

2) Run launch file
```bash
ros2 launch limo_description sim.launch.py
```

It should show the Rviz view and Gazebo simulation as seen in the figure below:

![Simulation Screenshot](https://github.com/user-attachments/assets/e245c9de-abda-4360-9457-68f8df1d112a)

## LAB 1.1

### Inverse Kinematics

In this lab, we have used two models to compare the performance and accuracy of inverse kinematics models:

- [Bicycle Model](#bicycle-model)
- [Ackermann Model](#ackermann-model) (no-slip condition constraints)

#### Bicycle Model

Try running the following commands to test the bicycle model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Run controller
```bash
ros2 run robot_controller ackerman_controller_basic_model.py
```
3) Run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
4) Try controlling the robot using the keyboard and observe the results in Rviz and Gazebo simulation.

#### Ackermann Model

Try running the following commands to test the Ackermann model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Run controller
```bash
ros2 run robot_controller ackerman_controller_no_slip.py
```
3) Run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
4) Try controlling the robot using the keyboard and observe the results in Rviz and Gazebo simulation.

### Forward Kinematics

In this lab, we have used three models to compare the performance and accuracy of forward kinematics models:

- [Yaw-Rate Model](#yaw-rate-model)
- [Single Track Model](#single-track-model)
- [Double Track Model](#double-track-model)

#### Yaw-Rate Model

Try running the following commands to test the yaw-rate model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Choose an inverse kinematics model from [Inverse Kinematics](#inverse-kinematics)
3) Run yaw rate odometry
```bash
ros2 launch robot_controller ackerman_yaw_rate_odom.py
```
4) Run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
5) Use the odometry recording and plotting tool to compare results.

#### Single Track Model

Try running the following commands to test the single track model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Choose an inverse kinematics model from [Inverse Kinematics](#inverse-kinematics)
3) Run yaw rate odometry
```bash
ros2 launch robot_controller ackerman_yaw_rate_odom.py
```
4) Run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
5) Use the odometry recording and plotting tool to compare results.

#### Double Track Model

Try running the following commands to test the double track model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Choose an inverse kinematics model from [Inverse Kinematics](#inverse-kinematics)
3) Run yaw rate odometry
```bash
ros2 launch robot_controller ackerman_yaw_rate_odom.py
```
4) Run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
5) Use the odometry recording and plotting tool to compare results.

### Methodology and Results

This section compares inverse and forward kinematics by recording the odometry of each forward kinematics model using different inverse kinematics models for data collection and comparison.

#### Steps for Testing

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```

2) Choose and run the inverse and forward kinematics model:

**Basic Model + All Odometry:**
```bash
ros2 launch robot_controller basic_model+all_odom.launch.py
```

**No-Slip Model + All Odometry:**
```bash
ros2 launch robot_controller noslip_model+all_odom.launch.py
```

3) Use the odometry recording and plotting tool to compare results.

### **Results**
We tested the system by making the robot move in a **circular left turn** using different kinematic models.

#### **Basic Model Output**
<p align="center">
  <img src="./results_data/lab1.1/basic_odom_all_3.png" alt="Basic Model Odom" width="35%"/>
  <img src="./results_data/lab1.1/basic_odom_all_3_yaw.png" alt="Basic Model Yaw Odom" width="47%"/>
</p>

### **No-Slip Model Output**
<p align="center">
  <img src="./results_data/lab1.1/noslip_odom_all_3.png" alt="No-Slip Model Odom" width="35%"/>
  <img src="./results_data/lab1.1/noslip_odom_all_3_yaw.png" alt="No-Slip Model Yaw Odom" width="47%"/>
</p>

### **RMSE Results**
The following table presents the **Root Mean Square Error (RMSE)** values comparing the odometry performance across different models.  
Lower RMSE values indicate **better accuracy** in following the expected trajectory.

#### **XY Position RMSE Data**
| Model Type  | Yaw Rate (RMSE) | Single Track (RMSE) | Double Track (RMSE) |
|-------------|----------------|----------------------|----------------------|
| **Basic Model** | 0.1187 | 0.2372 | 0.7821 |
| **No-Slip** | 0.1143 | 0.4142 | 0.1599 |

#### **Yaw Orientation RMSE Data (Radians)**
| Model Type  | Yaw Rate (RMSE) | Single Track (RMSE) | Double Track (RMSE) |
|-------------|----------------|----------------------|----------------------|
| **Basic Model** | 0.1256 | 0.9162 | 2.2293 |
| **No-Slip** | 0.0037 | 2.4682 | 0.6105 |

---

### **Observations**
- The **Basic Model** has a **higher yaw rate RMSE** but shows **better position tracking** than the No-Slip Model in the **Single Track and Double Track** configurations.
- The **No-Slip Model** has a **lower yaw rate RMSE**, which suggests it provides a more stable and accurate heading estimation.
- In **XY Position RMSE**, the **No-Slip Model performs better** in the **Double Track** configuration, showing the lowest RMSE (0.1599), while the Basic Model performs better in the **Single Track** configuration.
- In **Yaw Orientation RMSE**, the **Basic Model has significantly higher errors** in the **Single Track and Double Track** configurations, whereas the **No-Slip Model shows more stable yaw tracking**, especially in the **Yaw Rate** configuration with an RMSE of just **0.0037 radians**.
- Overall, the **No-Slip Model provides better yaw stability**, while the **Basic Model maintains better positional accuracy** in certain cases.

## LAB 1.2
### PID Controller
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

### PID Controller Results
#### Path Tracking
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/PID/PID_path.png)

#### Speed Profile
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/PID/PID_speed.png)

---

### Pure Pursuit Controller


### Pure Pursuit Controller Results

#### Path Tracking

![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/purepursuit/purepursuit_path.png)

#### Speed Profile
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/purepursuit/purepursuit_speed.png)


### MPC Controller

### MPC Controller Results

#### Path Tracking
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/MPC/mpc_path.png)

#### Speed Profile
![Image Description](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/path_tracking/result/MPC/mpc_speed.png)


## Our Team
- **67340700402** พงษ์พัฒน์ วงศ์กำแหงหาญ
- **67340700403** พีรดนย์ เรืองแก้ว


## References

### Bicycle Model
- [Algorithms for Automated Driving - Bicycle Model](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html)
- [ROS2 Controllers - Bicycle Model](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#car-like-bicycle-model)

### Ackermann Model
- [MathWorks - Kinematic Steering](https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html)
- [Ackermann Steering](https://raw.org/book/kinematics/ackerman-steering/)
- [ROS2 Controllers - Ackermann Model](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#ackermann-steering)

### Yaw-Rate, Single Track, and Double Track Models
- [IEEE Paper on Vehicle Dynamics](https://ieeexplore.ieee.org/document/8574906)


