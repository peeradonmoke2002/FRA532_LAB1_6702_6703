# FRA532 Mobile Robot

## Table of Contents

- [Installation](#installation)
- [LAB 1.1](#lab-11)
    - [Inverse Kinematics](#inverse-kinematics)
    - [Forward Kinematics](#forward-kinematics)
    - [Methodology and Results](#methodology-and-results)

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

- [Yaw-Rate Model](#yaw-rate)
- [Single Track Model](#single-track)
- [Double Track Model](#double-track)

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

#### Results

| Model Type  | Yaw Rate | Single Track | Double Track |
|-------------|---------|--------------|--------------|
| Basic Model | TBD     | TBD          | TBD          |
| No-Slip     | TBD     | TBD          | TBD          |

## Our Team

1. **67340700402** พงษ์พัฒน์ วงศ์กำแหงหาญ
2. **67340700403** พีรดนย์ เรืองแก้ว

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


