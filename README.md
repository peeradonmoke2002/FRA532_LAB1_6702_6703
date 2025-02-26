# FRA532 Mobile Robot
## Table of Contents

- [Installation](#Installation)
- [LAB 1.1](#lab1.1)
    - [Inverse Kinematics](#inverse-kinematics)
    - [Forward Kinematics](#forward-kinematics)
    - [Methodology and Results](#methodology-and-results)


## Installation
step : create worksapce
```bash
mkdir -p ~/FRA532_MobileRobot/src
```
step 2: git clone the repository
```bash
cd ~/FRA532_MobileRobot/src
git clone https://github.com/peeradonmoke2002/FRA532_LAB1_6702_6703.git
```
step 3: build workspace
```bash
cd ~/FRA532_MobileRobot
colcon build
source install/setup.bash 
```


### Testing Rviz view and Gazebo simulation 
1) build workspace

```bash
cd ~/FRA532_MobileRobot
colcon build
source install/setup.bash 
```

2) run launch file
```bash
ros2 launch limo_description sim.launch.py
```
it shoud show rivz view and gazebo simulation as figure below:

<img width="1373" alt="Screenshot 2568-02-26 at 12 40 33" src="https://github.com/user-attachments/assets/e245c9de-abda-4360-9457-68f8df1d112a" />

## LAB 1.1

### Inverse Kinematics
In this lab we have use two model to compare performance and accuracy of each inverse kinematics models. which are: [Bicycle Model](#bicycle-model) and [Ackermann Model](#ackermann-model) (no slip condition constraints).

#### Bicycle Model
Try run the following command to test the bicycle model:

1) run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) run controller
```bash
ros2 run robot_controller ackerman_controller_basic_model.py
```
3) run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
4) Try control the robot by using keyboard and see the result in rviz view and gazebo simulation


#### Ackermann Model
Try run the following command to test the ackermann model:

1) run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) run controller
```bash
ros2 run robot_controller ackerman_controller_no_slip.py
```
3) run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
4) Try control the robot by using keyboard and see the result in rviz view and gazebo simulation


### Forward Kinematics
In this lab we have use 3 model to compare performance and accuracy of each forward kinematics models. which are: [Yaw-Rate, Single Track, and Double Track Models](#yaw-rate-single-track-and-double-track-models)


#### Yaw-Rate
Try run the following command to test the yaw-rate model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Run choose inverse kinematics from [Inverse Kinematics](#inverse-kinematics)

3) Run Yaw rate odom
```bash
ros2 launch robot_controller ackerman_yaw_rate_odom
```
4) run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
5) use odom record and plot tool for compare the results

#### Single Track
Try run the following command to test the single track model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Run choose inverse kinematics from [Inverse Kinematics](#inverse-kinematics)
3) Run Yaw rate odom
```bash
ros2 launch robot_controller ackerman_yaw_rate_odom
```
4) run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
5) use odom record and plot tool for compare the results


#### Double Track
Try run the following command to test the double track model:

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```
2) Run choose inverse kinematics from [Inverse Kinematics](#inverse-kinematics)
3) Run Yaw rate odom
```bash
ros2 launch robot_controller ackerman_yaw_rate_odom
```
4) run teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
5) use odom record and plot tool for compare the results


### Methodology and Results
In this section want to compare inverse and forward kinematics by record the odom of each forward kinematics model and use different model of inverse kinematics models for collect data and compareing 

#### Here step for testing

1) Run simulation
```bash
ros2 launch limo_description sim.launch.py
```

2) Choose launch file for run the inverse and forward kinematics

This is basic model + all odom
```bash
ros2 launch robot_controller basic_model+all_odom.launch.py
```
This is noslip model + all odom
```bash
ros2 launch robot_controller noslip_model+all_odom.launch.py
```
3) use odom record and plot tool for compare the results

#### Results

|  | Yaw Rate | Single Track | Double Track |
|-----------|------------|------------|------------|
| Basic Model |  |  |  | 
| No-slip |  |  |  | 


## Our Team

1. 67340700402 พงษ์พัฒน์ วงศ์กำแหงหาญ
2. 67340700403 พีรดนย์ เรืองแก้ว

## Reference

### Bicycle Model
https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html
https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#car-like-bicycle-model

### Ackermann Model
https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html
https://raw.org/book/kinematics/ackerman-steering/
https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#ackermann-steering

### Yaw-Rate, Single Track, and Double Track Models
https://ieeexplore.ieee.org/document/8574906



