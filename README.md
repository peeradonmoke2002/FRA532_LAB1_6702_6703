# FRA532 Mobile Robot: LAB1
## Table of Contents
- [Forward Kinematics](#forward-kinematics)
  - [Bicycle model](#bicycle-model)
  - [No Slip condition constraints](#no-slip-condition-constraints)
- [Inverse Kinematics](#inverse-kinematics)
  - [Yaw Rate](#yaw-rate)
  - [Single-track model](#single-track-model)
  - [Double-track model](#double-track-model)

## Forward Kinematics

### Bicycle model
Converting wheel velocity and steering to robot twist (linear velocity, angular velocity)

#### Steering Angle Calculation

From the geometric relationship, we have:

https://quicklatex.com/cache3/01/ql_cf9749a75145c6d99c28506638e35001_l3.png



Using the velocity equation \( v = \Omega_z R \), where \( v \) represents the velocity magnitude, we can express the steering angle as:

$$
\delta = \arctan \left( \frac{L \Omega_z}{v} \right)
$$

where:
- \( \delta \) is the steering angle,
- \( L \) is the wheelbase (distance between front and rear axles),
- \( R \) is the turning radius,
- \( \Omega_z \) is the angular velocity,
- \( v \) is the linear velocity of the vehicle.

These equations describe the relationship between the steering angle and the vehicle's motion.



### No Slip condition constraints

## Inverse Kinematics

### Yaw Rate

### Single-track model

### Double-track model

## Our Team

1. 67340700402 พงษ์พัฒน์ วงศ์กำแหงหาญ
2. 67340700403 พีรดนย์ เรืองแก้ว

## Reference
https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html
