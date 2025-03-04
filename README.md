# FRA532 Mobile Robot

## Table of Contents
- [System Overview](#system-overview)
- [Installation](#installation)
- [LAB 1.1](#lab-11)
    - [Create Robot Model](#create-robot-model)
    - [Inverse Kinematics](#inverse-kinematics)
    - [Forward Kinematics](#forward-kinematics)
    - [Methodology and Results lab 1.1](#methodology-and-results-lab-11)  
- [LAB 1.2](#lab-12)
    - [PID Controller](#pid-controller)
    - [Pure Pursuit Controller](#pure-pursuit-controller)
    - [Stanley Controller](#stanley-controller)
    - [Methodology and Results lab 1.2](#methodology-and-results-lab-12)

- [LAB 1.3](#lab-13)

- [Our Team](#our-team)
- [References](#references)



## System Overview
![Personal diagram - FRA532 Mobile Robot-3](https://github.com/user-attachments/assets/10c20c86-fc86-4021-bdda-3a982760244b)

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
> [!NOTE]  
> - The lab 1.1 package folder is `~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/robot_controller`
> - For Robot Model package are locate at `~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/limo_description`

### Create Robot Model

#### Robot Dimension
![image](https://github.com/user-attachments/assets/d33851ab-3935-4c03-9aee-bf0b32545424)

Based on the robot dimensions set in this lab, you need to configure the following parameters in `limo_description` as shown below:

- **Wheel base:** `0.2 m`
- **Wheel radius:** `0.045 m`
- **Wheel length:** `0.001 m`
- **Track width:** `0.13 m`

> [!WARNING]  
> Incorrect dimensions will negatively impact the robot's performance and accuracy in Labs 1.1, 1.2, and 1.3. Please ensure that you have set the correct dimensions.


### TF (Transform Frames)

After creating the robot model based on the specified robot dimensions, the following coordinate frame transformations should be set up:

![Screenshot from 2025-03-03 23-10-37](https://github.com/user-attachments/assets/995d742f-06c9-4ac5-9dd7-c04f061cbb04)

#### **Transformations**
1. **`base_footprint` → `base_link`**  
   - Represents the static relationship between the center of the robot base (`base_link`) and the floor (`base_footprint`).
   - The `base_footprint` frame is usually at ground level, while `base_link` is at the actual center of the robot.

2. **`base_link` → `front_left_wheel_link`**  
   - Defines the transformation from the robot’s main body (`base_link`) to the **front-left wheel**.
   - The position is offset forward and to the left, based on the track width and wheelbase dimensions.

3. **`base_link` → `front_right_wheel_link`**  
   - Defines the transformation from `base_link` to the **front-right wheel**.
   - The position is offset forward and to the right, similar to the **front-left wheel**, but on the opposite side.

4. **`base_link` → `rear_left_wheel_link`**  
   - Connects `base_link` to the **rear-left wheel**.
   - This transformation moves the reference frame to the rear of the robot and offsets it to the left.

5. **`base_link` → `rear_right_wheel_link`**  
   - Connects `base_link` to the **rear-right wheel**.
   - This transformation moves the reference frame to the rear of the robot and offsets it to the right.

6. **`base_link` → `depth_camera_link`**  
   - Represents the transformation from `base_link` to the **depth camera** mounted on the robot.
   - The camera is typically placed at a fixed height and slightly forward to provide a clear field of view.

7. **`base_link` → `imu_link`**  
   - Defines the transformation from `base_link` to the **IMU (Inertial Measurement Unit)**.
   - The IMU is usually mounted at the robot’s center to minimize bias in angular velocity and acceleration readings.

8. **`base_link` → `laser_link`**  
   - Connects `base_link` to the **laser scanner (LiDAR) sensor**.
   - The LiDAR is commonly placed at the front or center of the robot, slightly above the base, to provide optimal mapping and obstacle detection.

9. **`depth_camera_link` → `camera_depth_optical_frame`**  
   - This transformation represents the coordinate change from the **depth camera frame** to its **optical frame**.
   - It ensures that depth perception calculations align with the camera’s internal processing frame.

10. **`base_link` → `depth_link`**  
   - Defines a transformation to another reference frame used for depth calculations.
   - This could be an intermediate frame used for sensor fusion or processing depth images.

<img width="1371" alt="Screenshot 2568-03-03 at 23 49 39" src="https://github.com/user-attachments/assets/e9974f2c-f8a7-4839-95ea-7bbec239c075" />




### Inverse Kinematics

In this lab, we have used two models to compare the performance and accuracy of inverse kinematics models:

- [Bicycle Model](#bicycle-model)
- [Ackermann Model](#ackermann-model) (no-slip condition constraints)

#### Bicycle Model
For the bicycle model, the two front wheels as well as the two rear wheels are lumped into one wheel each.

![image](https://github.com/user-attachments/assets/8084ff17-cbdd-4f04-a37f-424a7ce0a5a1)

From the figure above, we can determine the steering angle $\delta$ using the geometric relationship:

```math
\tan(\delta) = \frac{L}{R}
```

where:
- $L$ is the wheelbase (distance between the front and rear axle)
- $R$ is the turning radius

We also know the relationship between velocity $v$ and angular velocity $\omega_z$:

```math
v = \omega_z R
```

Rearranging for $R$:

```math
R = \frac{v}{\omega_z}
```

Substituting into the first equation:

```math
\delta = \arctan \left( \frac{L \cdot \omega_z}{v} \right)
```
For more information -> [Bicycle Model References](#bicycle-model-references)

#### Running Bicycle Model
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

### Ackermann Model
For the Ackermann model, to prevent the front wheels from slipping, the steering angles of the front wheels cannot be equal.

![image](https://github.com/user-attachments/assets/65b5b864-37c0-40ca-aa54-e12060a3246a)


The turning radius of the robot is:

```math
R = \frac{WB}{\tan(\delta)}
```
The steering angles of the front wheels must satisfy these conditions to avoid skidding:

```math
\delta_{left} = \arctan \left( \frac{2 \cdot WB \cdot \sin(\delta)}{2 \cdot WB \cdot \cos(\delta) - TW \cdot \sin(\delta)} \right)
```
```math
\delta_{right} = \arctan \left( \frac{2 \cdot WB \cdot \sin(\delta)}{2 \cdot WB \cdot \cos(\delta) + TW \cdot \sin(\delta)} \right)
```
where

- `WB` is the **wheel_base**
- `TW` is the **track_width**

For more information -> [Ackermann Model References](#ackermann-model-references)

#### Running Ackermann Model
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
For the concept of dead-reckoning, modeling or odometry is a method used to estimate the current position of an object by using its last known location and information about its velocity or traveled distance.
```math
\begin{align} 
\begin{bmatrix} 
x_{k+1} \\ 
y_{k+1} \\ 
\theta_{k+1} 
\end{bmatrix} 
= 
\begin{bmatrix} 
x_k \\ 
y_k \\ 
\theta_k 
\end{bmatrix} 
+ 
\begin{bmatrix} 
v_k \cdot \Delta t \cdot \cos \left(\beta_k + \theta_k + \frac{\omega_k \cdot \Delta t}{2} \right) \\ 
v_k \cdot \Delta t \cdot \sin \left(\beta_k + \theta_k + \frac{\omega_k \cdot \Delta t}{2} \right) \\ 
\omega_k \cdot \Delta t 
\end{bmatrix}. 
\end{align}
```

where:

- $$\( x_k, y_k \)$$ are the current coordinates of the object.
- $$\( \theta_k \)$$ is the current orientation angle.
- $$\( v_k \)$$ is the linear velocity.
- $$\( \omega_k \)$$ is the angular velocity.
- $$\( \beta_k \)$$ is the slip angle (if applicable).
- $$\( \Delta t \)$$ is the time step.

  
For more information about forward kinematics, refer to this reference, which includes details from above and below: -> [Yaw-Rate, Single Track, and Double Track Models References](#yaw-rate-single-track-and-double-track-models-references)


In this lab, we have used three models to compare the performance and accuracy of forward kinematics models:

- [Yaw-Rate Model](#yaw-rate-model)
- [Single Track Model](#single-track-model)
- [Double Track Model](#double-track-model)

#### Yaw-Rate Model
For the model yaw-rate are uses the yaw rate $$ω$$ directly measured by the gyroscope sensor as the information for the rotation and for the model also uses the average rear wheel velocirites as the information on the translation motion

#### Running Yaw-Rate Model
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
For the single track model the translation motion is determined by average velocity of the rear wheels and for the roation is determind by use $$δ$$ front to calculate rotation rate $$ω$$

```math
\begin{align} 
\begin{bmatrix} 
x_k\\ 
y_k\\ 
\theta_k\\ 
\beta_k\\ 
v_k\\ 
\omega_k 
\end{bmatrix} 
= 
\begin{bmatrix} 
x_{k-1} + v_{k-1} \cdot \Delta t \cdot \cos \left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2} \right)\\ 
y_{k-1} + v_{k-1} \cdot \Delta t \cdot \sin \left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2} \right)\\ 
\theta_{k-1} + \omega_{k-1} \cdot \Delta t\\ 
\beta^*_{R,k}\\ 
\frac{\tilde{v}_{RL,k}^\times + \tilde{v}_{RR,k}^\times}{2}\\ 
\frac{v_{k-1}}{r_b} + \tan (δ_{F,k})
\end{bmatrix} 
\end{align}
```
where:

- $$\( x_k, y_k \)$$ are the current coordinates of the object.
- $$\( \theta_k \)$$ is the current orientation angle.
- $$\( \beta_k \)$$ is the slip angle.
- $$\( v_k \)$$ is the linear velocity.
- $$\( \omega_k \)$$ is the angular velocity.
- $$\( \delta_{F,k} \)$$ is the front steering angle.
- $$\( r_b \)$$ is the wheelbase.
- $`(\tilde{v}_{RL,k}^\times, \tilde{v}_{RR,k}^\times)`$ are the left and right rear wheel velocities.
- $$\( \Delta t \)$$ is the time step.


#### Running Single Track Model
Try running the following commands to test the single track model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Choose an inverse kinematics model from [Inverse Kinematics](#inverse-kinematics)
3) Run yaw rate odometry
```bash
ros2 launch robot_controller ackerman_odom_single_track.py
```
4) Run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
5) Use the odometry recording and plotting tool to compare results.

#### Double Track Model
For double track model are determines the pose by suing basic of two single wheel velocities, For translation motion is calculate by average of the velocities of the two wheels and the roatation is determined on the basic of the velocity difference with respect to the single wheel angle and wheel contact points

```math
\begin{align} 
\begin{bmatrix} 
x_k\\ 
y_k\\ 
\theta_k\\ 
\beta_k\\ 
v_k\\ 
\omega_k 
\end{bmatrix} 
= 
\begin{bmatrix} 
x_{k-1} + v_{k-1} \cdot \Delta t \cdot \cos \left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2} \right)\\ 
y_{k-1} + v_{k-1} \cdot \Delta t \cdot \sin \left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2} \right)\\ 
\theta_{k-1} + \omega_{k-1} \cdot \Delta t\\ 
0\\
\frac{\tilde{v}_{RL,k}^\times + \tilde{v}_{RR,k}^\times}{2}\\ 
\omega_{k-1}
\end{bmatrix}
\end{align}
```

where:

- $$\( x_k, y_k \)$$ are the current coordinates of the object.
- $$\( \theta_k \)$$ is the current orientation angle.
- $$\( \beta_k \)$$ is the slip angle.
- $$\( v_k \)$$ is the linear velocity.
- $$\( \omega_k \)$$ is the angular velocity.
- $`(\tilde{v}_{RL,k}^\times, \tilde{v}_{RR,k}^\times)`$ are the left and right rear wheel velocities.
- $$\( \Delta t \)$$ is the time step.


To find $$\omega_{k-1}$$

```math
\begin{align} 
\omega = 
\frac{ v_1\cdot \cos (\delta_2 - \beta) - v_2\cdot \cos (\delta_1 - \beta) }
{\begin{array}{l} 
r_{1,x} \cdot \sin (\delta_1) \cos (\delta_2 - \beta) - r_{1,y} \cdot \cos (\delta_1) \cos (\delta_2 - \beta) \\ 
- r_{2,x} \cdot \sin (\delta_2) \cos (\delta_1 - \beta) + r_{2,y} \cdot \cos (\delta_2) \cos (\delta_1 - \beta) 
\end{array}} 
\end{align}
```
where:

- $$\( x_k, y_k \)$$ are the current coordinates of the object.
- $$\( \theta_k \)$$ is the current orientation angle.
- $$\( \beta_k \)$$ is the slip angle.
- $$\( v_k \)$$ is the linear velocity.
- $$\( \omega_k \)$$ is the angular velocity.
- $`( \tilde{v}_{RL,k}^\times, \tilde{v}_{RR,k}^\times)`$ are the left and right rear wheel velocities.
- $$\( \Delta t \)$$ is the time step.
- $$\( \omega \)$$ is the calculated angular velocity.
- $$\( \delta_1, \delta_2 \)$$ are the front wheel steering angles.
- $$\( v_1, v_2 \)$$ are the wheel velocities.
- $$\( r_{1,x}, r_{1,y}, r_{2,x}, r_{2,y} \)$$ are the contact points of the wheels.





#### Running Double Track Model
Try running the following commands to test the double track model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Choose an inverse kinematics model from [Inverse Kinematics](#inverse-kinematics)
3) Run yaw rate odometry
```bash
ros2 launch robot_controller ackerman_odom_double_track.py
```
4) Run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
5) Use the odometry recording and plotting tool to compare results.

### Methodology and Results lab 1.1

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

3) Use the odometry recording  `record_odom_all.py` and plotting `plot_all.py` to compare results.

> [!WARNING] 
> the data will be overwrite if you run the launch file again.

### **Results**
We tested the system by making the robot move in a **circular left turn** using different kinematic models.

#### **Basic Model Output**
- at speed `v = 0.5 m/s`
<p align="center">
  <img src="./results_data/lab1.1/basic_odom_all_3.png" alt="Basic Model Odom" width="35%"/>
  <img src="./results_data/lab1.1/basic_odom_all_3_yaw.png" alt="Basic Model Yaw Odom" width="47%"/>
</p>

- at speed `v = 0.32 m/s`
<p align="center">
  <img src="./results_data/lab1.1/basic_odom_all_speed(0.32).png" alt="Basic Model Odom" width="35%"/>
  <img src="./results_data/lab1.1/basic_odom_all_speed(0.32)_yaw.png" alt="Basic Model Yaw Odom" width="47%"/>
</p>

- at speed `v = 0.72 m/s`
<p align="center">
  <img src="./results_data/lab1.1/basic_odom_all_speed(0.72).png" alt="Basic Model Odom" width="35%"/>
  <img src="./results_data/lab1.1/basic_odom_all_speed(0.72)_yaw.png" alt="Basic Model Yaw Odom" width="47%"/>
</p>

#### **No-Slip Model Output**
- at speed `v = 0.5 m/s`
<p align="center">
  <img src="./results_data/lab1.1/noslip_odom_all_3.png" alt="No-Slip Model Odom" width="35%"/>
  <img src="./results_data/lab1.1/noslip_odom_all_3_yaw.png" alt="No-Slip Model Yaw Odom" width="47%"/>
</p>

- at speed `v = 0.32 m/s`
<p align="center">
  <img src="./results_data/lab1.1/noslip_odom_all_speed(0.32).png" alt="Basic Model Odom" width="35%"/>
  <img src="./results_data/lab1.1/noslip_odom_all_speed(0.32)_yaw.png" alt="Basic Model Yaw Odom" width="47%"/>
</p>

- at speed `v = 0.72 m/s`
<p align="center">
  <img src="./results_data/lab1.1/noslip_odom_all_speed(0.72).png" alt="Basic Model Odom" width="35%"/>
  <img src="./results_data/lab1.1/noslip_odom_all_speed(0.72)_yaw.png" alt="Basic Model Yaw Odom" width="47%"/>
</p>


### **RMSE Results**
The following table presents the **Root Mean Square Error (RMSE)** values comparing the odometry performance across different models.  
Lower RMSE values indicate **better accuracy** in following the expected trajectory.

#### **XY Position RMSE Data**
- at speed `v = 0.5 m/s`

| Model Type  | Yaw Rate (RMSE) | Single Track (RMSE) | Double Track (RMSE) |
|-------------|----------------|----------------------|----------------------|
| **Basic Model** | 0.1187 | 0.2372 | 0.7821 |
| **No-Slip** | 0.1143 | 0.4142 | 0.1599 |

- at speed `v = 0.32 m/s`

| Model Type  | Yaw Rate (RMSE) | Single Track (RMSE) | Double Track (RMSE) |
|-------------|----------------|----------------------|----------------------|
| **Basic Model** | 0.1736 | 0.4021 | 0.7766 |
| **No-Slip** | 0.1335 | 0.3486 | 0.1296 |

- at speed `v = 0.72 m/s`

| Model Type  | Yaw Rate (RMSE) | Single Track (RMSE) | Double Track (RMSE) |
|-------------|----------------|----------------------|----------------------|
| **Basic Model** | 0.1250 | 0.4048 | 0.7455 |
| **No-Slip** | 0.1745 | 1.4030 | 2.3143 |

#### **Yaw Orientation RMSE Data (Radians)**
- at speed `v = 0.5 m/s`

| Model Type  | Yaw Rate (RMSE) | Single Track (RMSE) | Double Track (RMSE) |
|-------------|----------------|----------------------|----------------------|
| **Basic Model** | 0.1256 | 0.9162 | 2.2293 |
| **No-Slip** | 0.0037 | 2.4682 | 0.6105 |

- at speed `v = 0.32 m/s`

| Model Type  | Yaw Rate (RMSE) | Single Track (RMSE) | Double Track (RMSE) |
|-------------|----------------|----------------------|----------------------|
| **Basic Model** | 0.0026 | 1.5300 | 2.3891 |
| **No-Slip** | 0.1350 | 1.8313 | 0.3017 |

- at speed `v = 0.72 m/s`

| Model Type  | Yaw Rate (RMSE) | Single Track (RMSE) | Double Track (RMSE) |
|-------------|----------------|----------------------|----------------------|
| **Basic Model** | 0.1143 | 0.3979 | 0.8415 |
| **No-Slip** | 0.2185 | 1.8949 | 1.9204 |
---

### **Observations**
- The **Basic Model** has a **higher yaw rate RMSE** but performs **better in position tracking** than the No-Slip Model in the **Single Track and Double Track** configurations.  
- The **No-Slip Model** maintains a **lower yaw rate RMSE**, indicating it provides a more **stable and accurate heading estimation** across all speeds.  
- At **lower speeds (`v = 0.32 m/s`)**, both models perform **similarly** in terms of position tracking, but the **No-Slip Model shows better yaw stability**.  
- As speed increases (**`v = 0.5 m/s` to `v = 0.72 m/s`**), the **Basic Model struggles with yaw drift**, especially in **Double Track mode**, whereas the **No-Slip Model maintains a more stable yaw rate**.  
- In **XY Position RMSE**, the **No-Slip Model outperforms** in the **Double Track** configuration, achieving the **lowest RMSE (0.1599)**, while the Basic Model performs better in the **Single Track** configuration.  
- In **Yaw Orientation RMSE**, the **Basic Model shows significantly higher errors** in **Single Track and Double Track** setups. The **No-Slip Model maintains more stable yaw tracking**, particularly in the **Yaw Rate** configuration, with an RMSE of just **0.0037 radians**.  
- **Higher speeds amplify the differences** between the models—**yaw errors increase in the Basic Model**, while the **No-Slip Model holds its accuracy better**, particularly in yaw tracking.  
- Overall, the **No-Slip Model delivers better yaw stability**, while the **Basic Model maintains stronger positional accuracy** in certain scenarios.  

## LAB 1.2
> [!NOTE]  
> The lab 1.2 package folder is `~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/path_tracking`

In this lab, we have used three traking controller to compare the performance and accuracy of each model:
- [PID Controller](#pid-controller)
- [Pure Pursuit Controller](#pure-pursuit-controller) 
- [Stanley Controller](#stanley-controller) 

For more information -> [Path Tracking Controllers References
](#path-tracking-controllers-references)

### PID Controller

PID it separate to steering and speed 

| Parameter | Description |
|-----------|------------|
| Kp_steer | Improves robot response to direction changes. |
| Ki_steer | Helps correct long-term drift in steering. |
| Kd_steer | Dampens sudden steering adjustments to reduce oscillation. |

| Parameter | Description |
|-----------|------------|
| Kp_speed | Controls speed response to target velocity. |
| Ki_speed | Corrects long-term speed drift. |
| Kd_speed | Smooths speed changes for stability. |

The formula of the PID controller is 

$$ u(t) = K_p e(t) + K_I \int_0^t e(t) ,dt + K_d \frac{de}{dt} $$

#### Running PID Controller

Try running the following commands to test the PID controller:

1. Run Simulation
```bash
ros2 launch limo_description sim.launch.py
```
2. Run PID launch file by can swtich the parameter to use basic model or noslip model:

- basic model:
```bash
ros2 launch path_tracking pid.launch.py mode:=basic
```

- noslip model:
```bash
ros2 launch path_tracking pid.launch.py mode:=noslip
```

3. See the results and stop the launch file when the robot completes one round

### Pure Pursuit Controller

Pure Pursuit is a path-tracking algorithm. The method calculates a steering angle to reach a lookahead point based on the robot's current position and orientation.

Lookahead Point Calculation  
The lookahead point $$\(x_L, y_L\)$$ in the global frame is calculated as:

```math
x_L = x + L_d \cos(\theta)
```

```math
y_L = y + L_d \sin(\theta)
```

where:
- $$\( x, y \)$$ are the current coordinates of the robot.
- $$\( \theta \)$$ is the current heading of the robot.
- $$\( L_d \)$$ is the lookahead distance.

Error in Global Frame  
The difference between the waypoint and the robot’s position:

```math
dx = x_{wp} - x
```

```math
dy = y_{wp} - y
```

where $$\(x_{wp}, y_{wp}\)$$ is the closest waypoint.

Transforming to Local Frame  
To compute the error in the robot’s local frame:

```math
x' = dx \cos(\theta) + dy \sin(\theta)
```

```math
y' = -dx \sin(\theta) + dy \cos(\theta)
```

where:
- $$\( x', y' \)$$ are the waypoint coordinates in the robot’s local frame.

Curvature Calculation  
Curvature $$\( \kappa \)$$ of the arc to the lookahead point is:

```math
\kappa = \frac{2 y'}{L_d^2}
```

where:
- $$\( y' \)$$ is the transformed y-coordinate in the robot’s local frame.
- $$\( L_d \)$$ is the lookahead distance.

Steering Angle Calculation
The required steering angle $$\( \delta \)$$ is determined using:

```math
\delta = \tan^{-1} (WB \cdot \kappa)
```

where:
- $$WB$$ is the wheelbase of the robot.

#### Running Pure Pursuit Controller

Try running the following commands to test the Pure Pursuit controller:

1. Run Simulation
```bash
ros2 launch limo_description sim.launch.py
```
2. Run Pure Pursuit launch file by can swtich the parameter to use basic model or noslip model:

basic model:
```bash
ros2 launch path_tracking pure_pursuit.launch.py mode:=basic
```

noslip model:
```bash
ros2 launch path_tracking pure_pursuit.launch mode:=noslip
```

3. See the results and stop the launch file when the robot completes one round

### Stanley Controller

The Stanley controller is a path-tracking algorithm that calculates a steering angle correction based on both the heading error and the cross-track error.

Lookahead Position (Front Axle Projection)
The front axle position $$\(x_f, y_f\)$$ is computed using:

```math
x_f = x + L \cos(\theta)
```

```math
y_f = y + L \sin(\theta)
```

where:
- $$\( x, y \)$$ are the current coordinates of the vehicle.
- $$\( \theta \)$$ is the vehicle’s heading angle.
- $$\( L \)$$ is the distance from the center of mass to the front axle.

Cross-Track Error Calculation
The cross-track error $$\( e \)$$ is the perpendicular distance from the front axle position to the nearest point on the reference path:

```math
e = (x_{	ext{wp}} - x_f) (-\sin(	\theta)) + (y_{	ext{wp}} - y_f) \cos(\theta)
```

where:
- $$\( x_{	ext{wp}}, y_{	ext{wp}} \)$$ are the nearest waypoint coordinates.
- $$\( x_f, y_f \)$$ are the front axle coordinates.
- $$\( e \)$$ is the cross-track error.

Heading Error Calculation
The heading error $$\( \theta_e \)$$ is given by the difference between the path yaw angle and the vehicle’s heading:

```math
\theta_e = \theta_{	ext{wp}} - \theta
```

where:
- $$\( \theta_{	ext{wp}} \)$$ is the yaw angle of the path at the nearest waypoint.

Steering Angle Calculation
The Stanley controller computes the desired steering angle $$\( \delta \)$$ as:

```math
\delta = \theta_e + \tan^{-1} \left( \frac{k e}{v} \right)
```

where:
- $$\( k \)$$ is the Stanley gain parameter.
- $$\( e \)$$ is the cross-track error.
- $$\( v \)$$ is the vehicle’s velocity.
- $$\( \theta_e \)$$ is the heading error.

#### Running Stanley Controller

Try to run following commands to test Stanley controller:
1. Run Simulation
```bash
ros2 launch limo_description sim.launch.py
```
2. Run Stanley launch file by can swtich the parameter to use basic model or noslip model:

basic model:
```bash
ros2 launch path_tracking stanley.launch mode:=basic
```

noslip model:
```bash
ros2 launch path_tracking stanley.launch mode:=noslip
```

3. See the results and stop the launch file when the robot completes one round


### Methodology and Results lab 1.2
This section compares the tracking controllers for each inverse kinematics model from **LAB 1.1** by recording **XY position changes over time** against the reference path defined in `path_tracking/path_data/path.yaml`. Additionally, we record the **speed profile** to analyze tracking performance.

#### Steps for Testing
Follow the steps for each tracking controller to run the test:
- [PID Controller](#pid-controller)
- [Pure Pursuit Controller](#pure-pursuit-controller) 
- [Stanley Controller](#stanley-controller) 

For case want record data:
 uncomment the following line in each launch file
```bash    
  launch_description.add_action(path_data_record)
```
For case don't want record data comment the following line in each launch file:
```bash
  # launch_description.add_action(path_data_record)
```
it will show the result in the `path_tracking/path_data/record_data` folder use the `plot_data_path` to plot the result.

> [!WARNING] 
> the data will be overwrite if you run the launch file again.

### **Results**
We conducted tests on the tracking controllers by making the robot complete one full round and analyzed the results.

#### PID Controller Results
##### Path Tracking and Speed Profile
- use dynamically speed for pid controller

![PID Controller Path Tracking](/results_data/lab1.2/pid_results.png)

#### Pure Pursuit Controller Results
##### Path Tracking and Speed Profile
- use speed at `0.5 m/s` for pure pursuit controller

![Pure Pursuit Controller Path Tracking](/results_data/lab1.2/purepursuit_results.png)

#### Stanley Controller Results
##### Path Tracking and Speed Profile
- use speed at `0.5 m/s` for stanley controller

![Stanley Controller Path Tracking](/results_data/lab1.2/stanley_results.png)

> [!NOTE] 
> can't compare in term of speed different due to PID use dynamically speed and Pure Pursuit and Stanley use fixed speed

### **RMSE Results**
The following table presents the **Root Mean Square Error (RMSE)** values comparing the path tracking controllers in different invese kinematics model from **Lab 1.1**.  
Lower RMSE values indicate **better accuracy** in following the expected trajectory.

#### **XY Position RMSE Data**
<table>
  <thead>
    <tr>
      <th rowspan="2">Model types</th>
      <th colspan="3">PID (RMSE)</th>
      <th colspan="3">Pure Pursuit (RMSE)</th>
      <th colspan="3">Stanley (RMSE)</th>
    </tr>
    <tr>
      <th>x</th>
      <th>y</th>
      <th>overall</th>
      <th>x</th>
      <th>y</th>
      <th>overall</th>
      <th>x</th>
      <th>y</th>
      <th>overall</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Basic</td>
      <td>0.024</td>
      <td>0.025</td>
      <td>0.035</td>
      <td>0.027</td>
      <td>0.029</td>
      <td>0.039</td>
      <td>0.021</td>
      <td>0.021</td>
      <td>0.030</td>
    </tr>
    <tr>
      <td>Noslip</td>
      <td>0.025</td>
      <td>0.025</td>
      <td>0.036</td>
      <td>0.026</td>
      <td>0.027</td>
      <td>0.038</td>
      <td>0.021</td>
      <td>0.021</td>
      <td>0.030</td>
    </tr>
  </tbody>
</table>

---
### **Observations**

- The **Stanley Controller** achieves the **lowest overall RMSE** (0.030 m) for both Basic and Noslip models, making it the most precise in terms of path tracking.
- The **PID Controller** follows closely behind, with overall RMSE values between **0.035–0.036 m**, showing reliable performance.
- The **Pure Pursuit Controller** has a slightly higher RMSE (**0.038–0.039 m**), indicating minor deviations from the reference path.
- **Basic vs. Noslip Models** show only **small differences in RMSE** (within 0.001–0.002 m), suggesting that both inverse kinematics models perform similarly in this scenario.
- The **x and y RMSE values are nearly equal**, indicating that all controllers maintain **balanced tracking in both axes**, without favoring one direction.
- **Speed profiles remain stable** around their set values, with minor fluctuations occurring when the controllers adjust for **tight turns or larger steering corrections**.
- Overall, **all three controllers track the reference path effectively**, with only small deviations. However, the **Stanley Controller stands out for its more precise path-following**, and the differences between **Noslip and Basic models are minimal**, likely due to limited wheel slip at these speeds.




## LAB 1.3
## State Vector Representation

The system state is defined as:

```math
X_k =
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
```
where:
- $`p_{x,k}, p_{y,k}, p_{z,k}`$ represent the **position**.
- $`\text{roll}_k, \text{pitch}_k, \text{yaw}_k`$ represent the **orientation**.
- $`v_{x,k}, v_{y,k}, v_{z,k}`$ represent the **linear velocity**.
- $`\omega_{x,k}, \omega_{y,k}, \omega_{z,k}`$ represent the **angular velocity**.
- $`a_{x,k}, a_{y,k}, a_{z,k}`$ represent the **linear acceleration**.



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

![image](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/doubletrack/PID/PID-doubletrack-crash.png)
![image](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/doubletrack/pp/purepursuit-doubletrack.png)
![image](https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/Path-Tracking-Controller/localization_ekf/result/doubletrack/stanlee/stanlee-doubletrack.png)

image
image
image




## Our Team
- **67340700402** พงษ์พัฒน์ วงศ์กำแหงหาญ
- **67340700403** พีรดนย์ เรืองแก้ว


## References

### Bicycle Model References
- [Algorithms for Automated Driving - Bicycle Model](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html)
- [ROS2 Controllers - Bicycle Model](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#car-like-bicycle-model)

### Ackermann Model References
- [MathWorks - Kinematic Steering](https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html)
- [Ackermann Steering](https://raw.org/book/kinematics/ackerman-steering/)
- [ROS2 Controllers - Ackermann Model](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#ackermann-steering)

### Yaw-Rate, Single Track, and Double Track Models References
- [IEEE Paper on Vehicle Dynamics](https://ieeexplore.ieee.org/document/8574906)

### Path Tracking Controllers References
- [PID Controller](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PID.html)
- [Pure Pursuit Controller](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html)
- [Stanley Controller](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)



