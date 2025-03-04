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
    - [State Vector Representation](#state-vector-representation)
    - [State Transition Equations](#state-transition-equations)
    - [How EKF Handles Nonlinearity](#how-ekf-handles-nonlinearity)
    - [Design the Process Noise Matrix](#design-the-process-noise-matrix)
    - [Design the Control Function](#design-the-control-function)
    - [Design the Measurement Function](#design-the-measurement-function)
    - [Design the Measurement Noise Matrix](#design-the-measurement-noise-matrix)
    - [Explain the Kalman Gain Computation](#explain-the-kalman-gain-computation)
    - [Implementation](#implementation)
    - [Methodology and Results lab 1.3](#methodology-and-results-lab-13)




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

3) Use the odometry recording  `record_odom_all.py` and plotting tool `plot_all.py` to compare results.

- cd to folder `data_analysis`
```bash
cd ~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/robot_controller/scripts/data_analysis/
```

- run record odom for collect data
```bash
python3 record_odom_all.py
```

- run plot all for plot data
```bash
python3 plot_all.py
```

> [!WARNING] 
> the data will be overwrite if you run odometry recording  `record_odom_all.py` file again.

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
> Running a launch file without commenting out the recording line will overwrite previous data.
> If you want to keep existing data, make sure to disable recording before running a new test.

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
> [!NOTE]  
> The lab 1.3 package folder is `~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/localization_ekf`

In this lab, we extend the odometry models from LAB 1.1 by fusing GPS data using an Extended Kalman Filter (EKF). The EKF output provides a pose estimate, which we then use as the input for path tracking, similar to LAB 1.2, but now relying on the EKF-estimated pose instead of raw odometry data.

## State Vector Representation

$$
x= 
​
  
x
y
θ
​
  
​
$$


- x, y: Position coordinates
- θ: Heading angle



## State Transition Equations

The state transition (or motion) model follows a simple kinematic model for a mobile robot. Given a control input:

$$
\mathbf{u} =
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
$$

where:
- \( v \) is the linear velocity
- \( \omega \) is the angular velocity

The state update equations are:

$$
x_{k+1} = x_k + v \cos(\theta_k) \Delta t
$$

$$
y_{k+1} = y_k + v \sin(\theta_k) \Delta t
$$

$$
\theta_{k+1} = \theta_k + \omega \Delta t
$$

where:
- \( x_k, y_k \) are the position coordinates at time step \( k \),
- \( \theta_k \) is the orientation (heading angle),
- \( \Delta t \) is the time step.


- This is implemented in the motion_model function:

```bash 
def motion_model(self, x, u):
    v, omega = u
    theta = x[2]
    x_next = np.zeros(3)
    x_next[0] = x[0] + v * math.cos(theta) * self.dt
    x_next[1] = x[1] + v * math.sin(theta) * self.dt
    x_next[2] = x[2] + omega * self.dt
    return x_next
```

##  How EKF Handles Nonlinearity

EKF is designed to work with nonlinear systems by approximating them linearly at each time step using Jacobian matrices.


### **Jacobian of the State Transition**
The **Jacobian matrix** \( \mathbf{F} \) of the motion model with respect to the state is computed to approximate the transition. For the state transition model given above, the **Jacobian** is:

$$
\mathbf{F} =
\begin{bmatrix}
1 & 0 & -v \sin(\theta) \Delta t \\
0 & 1 & v \cos(\theta) \Delta t \\
0 & 0 & 1
\end{bmatrix}
$$

This matrix linearizes the nonlinear motion model, allowing **Extended Kalman Filter (EKF)** to work efficiently.

### **Implementation in Python**
This is implemented in the `jacobian_F` function:

```python
import numpy as np
import math

def jacobian_F(self, x, u):
    """
    Compute the Jacobian matrix F for the state transition model.
    
    Parameters:
        x (numpy array): The current state [x, y, theta]
        u (numpy array): The control input [v, omega]
    
    Returns:
        numpy array: The Jacobian matrix F (3x3)
    """
    v, _ = u
    theta = x[2]
    
    # Initialize F as an identity matrix
    F = np.eye(3)
    
    # Compute the Jacobian terms for x and y with respect to theta
    F[0, 2] = -v * math.sin(theta) * self.dt
    F[1, 2] =  v * math.cos(theta) * self.dt
    
    return F
```


## Design the Process Noise Matrix

The process noise covariance matrix 
𝑄
Q represents the uncertainty in the state prediction. This uncertainty can come from model imperfections or unmodeled dynamics. In our implementation, 
```bash 

self.Q = np.diag([0.015, 0.015, 0.015])

```

## Design the Measurement Function

The measurement function maps the state vector into the measurement space. In our implementation, the measurement function is assumed to be:

$$
\mathbf{z} =
\begin{bmatrix}
x \\
y
\end{bmatrix}
$$

This means that our sensor (e.g., GPS) provides only the \(x\) and \(y\) positions. Since the measurement model is linear, its Jacobian \( \mathbf{H} \) is constant:

$$
\mathbf{H} =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 0
\end{bmatrix}
$$

*Note:* In our EKF update step for GPS, only \(x\) and \(y\) are used, so \( \mathbf{H} \) is effectively:

$$
\mathbf{H} =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0
\end{bmatrix}
$$

This measurement function is used directly in the EKF update step to compare the predicted state with the sensor measurements.

---

## Design the Measurement Noise Matrix

Two measurement noise covariance matrices are defined to model the uncertainty of our sensors:

1. **For GPS (which measures \(x\) and \(y\)):**

$$
\mathbf{R} = \operatorname{diag}(0.025,\ 0.025)
$$

2. **For odometry (which is assumed to provide the full state \(x\), \(y\), and \(\theta\)):**

$$
\mathbf{R}_{odom} = \operatorname{diag}(0.055,\ 0.055,\ 0.055)
$$

These matrices are designed based on the expected noise characteristics of the sensors.

### Implementation in Python

```python
self.R = np.diag([0.025, 0.025])
self.R_odom = np.diag([0.055, 0.055, 0.055])
```


## Explain the Kalman Gain Computation



### How to Tune \( K \)
- Lower $`( R )`$ → Higher trust in gps.  
- Lower $`( Q )`$ → Higher trust in model.  
-  Start with large Q R and gradually decrease while monitoring performance.
- Check innovation covariance $` S=HPH 
T
 +R`$

### Implementation

## YAW rate Q and R tuning [ PID - purepursuit - stanlee bicyble model ]
- becase odom forn yaw rate it are very percise so we can lower R odom for make it trust odom more then imu 

```bash

        # State vector: [x, y, theta]
        self.x_est = np.array([0.0, 0.0, 0.0])  # initial estimate
        self.P_est = np.eye(3) * 1.0            # initial covariance matrix

        # Process noise covariance matrix (3x3)
        # Lower values indicate higher confidence in the motion model.
        self.Q = np.diag([0.015, 0.015, 0.015])
        
        # Measurement noise covariance matrix for GPS (2x2)
        self.R = np.diag([0.025, 0.025])
        
        # Measurement noise covariance for odometry (3x3)
        self.R_odom = np.diag([0.055, 0.055, 0.055])
        
        # Time step (s)
        self.dt = 0.01

``` 
## Single track Q and R tuning [ PID - bicyble model ]

- Different control strategies (PID-based and pure pursuit/Stanley) require slight adjustments in these matrices to balance the trust between model prediction and sensor measurement


```bash 
        # State vector: [x, y, theta]
        self.x_est = np.array([0.0, 0.0, 0.0015])  # initial estimate
        self.P_est = np.eye(3) * 1.0            # initial covariance matrix

        # Process noise covariance matrix (3x3)
        # Lower values indicate higher confidence in the motion model.
        self.Q = np.diag([0.0155, 0.0155, 0.0155])
        
        # Measurement noise covariance matrix for GPS (2x2)
        self.R = np.diag([0.02, 0.02])
        
        # Measurement noise covariance for odometry (3x3)
        self.R_odom = np.diag([0.048, 0.048, 0.048])
        
        # Time step (s)
        self.dt = 0.01

```
## Single track Q and R tuning [ purepursuit and stanley - ]

```bash

        self.x_est = np.array([0.0, 0.0, 0.001])  # initial estimate
        self.P_est = np.eye(3) * 1.0            # initial covariance matrix

        # Process noise covariance matrix (3x3)
        # Lower values indicate higher confidence in the motion model.
        self.Q = np.diag([0.010, 0.010, 0.010])
        
        # Measurement noise covariance matrix for GPS (2x2)
        self.R = np.diag([0.02, 0.02])
        
        # Measurement noise covariance for odometry (3x3)
        self.R_odom = np.diag([0.05, 0.05, 0.05])
        
        # Time step (s)
        self.dt = 0.01

```



### Methodology and Results lab 1.3

select only one launch

```bash

ros2 launch localization_ekf ekf-yawrate.launch.py 

ros2 launch localization_ekf ekf-singletrack.launch.py 

ros2 launch localization_ekf ekf-doubletrack.launch.py 


```
select only one controller

pp = purepursuit

basic = bicycle model

```bash

ros2 run localization_ekf pid-basic.py

ros2 run localization_ekf pp-basic.py

ros2 run localization_ekf stanlee-basic.py

ros2 run localization_ekf pid-noslip.py

ros2 run localization_ekf pp-noslip.py

ros2 run localization_ekf stanlee-noslip.py


```
* optinal run save data for visualized and save data 

```bash

ros2 run localization_ekf save_data.py

```

then use this command after we want to know result

```bash

ros2 run localization_ekf plot_daat.py

```



#### Steps for Testing

#### **Step 1: Launch EKF (Select Only One)**  
Select the kinematic model you want to test:

For `yaw rate`:  
```bash
ros2 launch localization_ekf ekf-yawrate.launch.py  
```
For `single track`:  
```bash
ros2 launch localization_ekf ekf-singletrack.launch.py
```
For `double track`:  
```bash
ros2 launch localization_ekf ekf-doubletrack.launch.py
```

---

#### **Step 2: Run Control Method (Select Only One)**




### Control Results

**Figure 1: PID Yaw Basic Control**  
<img src="https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/robot-controller/results_data/lab1.3/img/pid-yawrate-basic.png?raw=true" width="900px" alt="PID Yaw Basic Control">  

**Figure 2: Stanley Yaw Basic Control**  
<img src="https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/robot-controller/results_data/lab1.3/img/stanlee-yawrate-basic.png?raw=true" width="900px" alt="Stanley Yaw Basic Control">  

**Figure 3: PID Single Track Basic Control**  
<img src="https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/robot-controller/results_data/lab1.3/img/pid-single-basic.png?raw=true" width="900px" alt="PID Single Track Basic Control">  

**Figure 4: Pure Pursuit Single Track Basic Control**  
<img src="https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/robot-controller/results_data/lab1.3/img/pp-single-basic.png?raw=true" width="900px" alt="Pure Pursuit Single Track Basic Control">  

**Figure 5: Stanley Single Track Basic Control**  
<img src="https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/robot-controller/results_data/lab1.3/img/stanlee-sinlge-basic.png?raw=true" width="900px" alt="Stanley Single Track Basic Control">  

**Figure 6: PID Yaw Noslip Control**  
<img src="https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/robot-controller/results_data/lab1.3/img/pid-yawrate-noslip.png?raw=true" width="900px" alt="PID Yaw Noslip Control">  

**Figure 7: Pure Pursuit Yaw Noslip Control**  
<img src="https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/robot-controller/results_data/lab1.3/img/pp_yawrate_noslip.png?raw=true" width="900px" alt="Pure Pursuit Yaw Noslip Control">  

**Figure 8: Stanley Yaw Noslip Control**  
<img src="https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/robot-controller/results_data/lab1.3/img/stanlee-yawrate-noslip.png?raw=true" width="900px" alt="Stanley Yaw Noslip Control">  

**Figure 9: Pure Pursuit Single Track Noslip Control**  
<img src="https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703/blob/robot-controller/results_data/lab1.3/img/pp-single-noslip.png?raw=true" width="900px" alt="Pure Pursuit Single Track Noslip Control">  



This table presents the **Root Mean Squared Error (RMSE)** and **Mean Absolute Error (MAE)** values, which measure localization accuracy. Lower values indicate better performance.

RMSE Results (Lower is Better) 
| Model Type  | Yaw Rate (RMSE) | Single Track (RMSE) | Double Track (RMSE) |
|-------------|----------------|----------------------|----------------------|
| **PID-basic**         | 0.397  | 1.441  | Failure |
| **PID-noslip**         |  0.070 | Failure  | Failure |
| **Pure Pursuit-basic**| 0.397  | 0.397 | Failure |
| **Pure Pursuit-noslip**| 0.070  |  0.263  | Failure |
| **Stanley-basic**     | 0.445  | 0.386 | Failure |
| **Stanley-noslip**     | 0.202  | Failure | Failure |



MAE Results (Lower is Better) 
| Model Type  | Yaw Rate (MAE) | Single Track (MAE) | Double Track (MAE) |
|-------------|----------------|----------------------|----------------------|
| **PID-basic**         | 0.483  | 1.770  | Failure |
| **PID-noslip**         | 0.084  | Failure  | Failure |
| **Pure Pursuit-basic**| 0.483  | 0.490 | Failure |
| **Pure Pursuit-noslip**| 0.084  | 0.259 | Failure |
| **Stanley-basic**     | 0.520  | 0.482 | Failure |
| **Stanley-noslip**     | 0.206  | Failure | Failure |


Avg left angle % change Results (Lower is Better) 
| Model Type  | Yaw Rate  | Single Track  | Double Track  |
|-------------|----------------|----------------------|----------------------|
| **PID-basic**         | 288.7 %  | 251.5%  | Failure |
| **PID-noslip**         | 180.5%  | Failure  | Failure |
| **Pure Pursuit-basic**| 325.1%  |  170.2 % | Failure |
| **Pure Pursuit-noslip**| 180.5%  | 93.2 %  | Failure |
| **Stanley-basic**     | 282.0 %  | 266.1 % | Failure |
| **Stanley-noslip**     | 192.2%  | Failure | Failure |


Avg  right angle % change  Results (Lower is Better) 
| Model Type  | Yaw Rate  | Single Track  | Double Track  |
|-------------|----------------|----------------------|----------------------|
| **PID-basic**         | 325.1 %  | 250.7 %  | Failure |
| **PID-noslip**         | 176.6%  | Failure  | Failure |
| **Pure Pursuit-basic**| 288.7 %  | 149.5 %  | Failure |
| **Pure Pursuit-noslip**| 176.6 %  | 85.1 %  | Failure |
| **Stanley-basic**     | 249.1 %  | 252.3 % | Failure |
| **Stanley-noslip**     | 159.4 %  | Failure | Failure |



Avg % rear wheel slip Results (Lower is Better) 
| Model Type  | Yaw Rate (Avg) | Single Track (Avg) | Double Track (Avg) |
|-------------|----------------|----------------------|----------------------|
| **PID-basic**         | 1.4 %  | 1.3%  | Failure |
| **PID-noslip**         | 1.4 %  | Failure  | Failure |
| **Pure Pursuit-basic**| 1.4 %  | 1.5 %  | Failure |
| **Pure Pursuit-noslip**| 1.4 %  |  1.6 % | Failure |
| **Stanley-basic**     | 1.6 %  | 1.5 % | Failure |
| **Stanley-noslip**     | 1.6 %  | Failure | Failure |


# Summary of EKF Localization Performance

## 1. Noslip vs. Basic
- Noslip mode provides better accuracy:
  - Lower RMSE and MAE compared to Basic, indicating improved trajectory tracking.
  - Lower steering angle percent changes, meaning smoother steering corrections and less abrupt movements.

---

## 2. Double Track
- All Double Track models failed due to:
  - Heading estimation errors, leading to the robot crashing into walls.
  - Suggests further improvements needed for handling heading estimation.

---

## 3. Single Track vs. Yaw Rate
- Performance varies:
  - Sometimes Single Track RMSE/MAE is higher than Yaw Rate, sometimes lower.
  - Noslip consistently improves Single Track performance compared to Basic.

---

## 4. RMSE and MAE (Lower is Better)
### Lowest RMSE  
- 0.07 for Yaw Rate with Noslip (PID-noslip, Pure Pursuit-noslip)  
- 0.263 for Single Track (Pure Pursuit-noslip)  

### Lowest MAE  
- 0.08 for some Noslip Yaw Rate models (PID-noslip, Pure Pursuit-noslip).
- Single Track MAE is generally higher but still improves with Noslip.

---

## 5. Conclusion
- Noslip mode outperforms Basic in minimizing RMSE/MAE:
  - Works better in both Yaw Rate and Single Track setups.
  - Reduces steering oscillations, leading to smoother control.
  - Improves overall accuracy and stability in EKF localization.

---

### Key Takeaway
- Use Noslip mode for better accuracy, smoother steering, and improved localization performance.



#### Observations




- **PID achieves the lowest RMSE (10.01) in Yaw Rate and 12.83 in Single Track**, indicating the **most accurate and stable localization** among all models.  
- **Pure Pursuit has the highest RMSE (11.77 Yaw Rate, 13.32 Single Track)**, showing **significant deviation from the ground truth path**, making it the **least accurate**.  
- **Stanley performs slightly better than Pure Pursuit (RMSE: 11.67 Yaw Rate, 13.21 Single Track)**, but **still exhibits errors in the Single Track scenario**.  
- **Double Track consistently fails for all models**, suggesting that **the localization system cannot handle this complex trajectory properly**, potentially due to **sensor fusion inaccuracies or an unstable model configuration**.  
- **MAE values follow a similar trend to RMSE**, reinforcing that **PID is the most stable and Pure Pursuit is the least stable**.  
- **Across all models, Yaw Rate errors are lower than Single Track errors**, indicating that the **Yaw Rate configuration provides more reliable localization than the Single Track model**.  

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

### EKF References

- [EFK2](https://github.com/AtsushiSakai/PythonRobotics/blob/master/docs/modules/2_localization/extended_kalman_filter_localization_files/extended_kalman_filter_localization_main.rst)
- [EKF-package](https://github.com/cra-ros-pkg/robot_localization)



