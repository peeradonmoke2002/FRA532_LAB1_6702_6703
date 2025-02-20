# FRA532 Mobile Robot: LAB1.1
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

Based on the velocity equation $$\ v = \Omega_z R \$$ where $$\ v \$$ represents the velocity magnitude, we can derive the steering angle as:

$$
\delta = \arctan \left( \frac{L \Omega_z}{v} \right)
$$

where:
- $$\ \delta \$$ is the steering angle,
- $$\ \Omega_z \$$ is the angular velocity,
- $$\ v \$$ is the linear velocity of the vehicle.

In this repository, we prefer to represent steering for both left and right turns so that when applying steering to this model, the same approach can be used for both directions.

#### Rear Wheel Speed Calculation

In this repository, we prefer to control the velocity of the left and right rear wheels by converting a twist message into rear wheel speed using the following formula:

$$\
\text{wheel}\_{speed} = \frac{v}{r}
\$$

where:
- $$\ v \$$ is the linear velocity,
- $$\ r \$$ is the wheel radius.

Additionally, we prefer to represent steering for both left and right turns so that the same approach can be applied consistently when controlling this model.

### No Slip condition constraints

## Inverse Kinematics

### Yaw Rate

### Single-track model

### Double-track model


# FRA532 Mobile Robot: LAB1.2

## Table of Contents
- [PID Controller](#pid-controller)
- [Pure Pursuit Controller](#pure-pursuit-controller)

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

## Pure Pursuit Controller



## Our Team

1. 67340700402 พงษ์พัฒน์ วงศ์กำแหงหาญ
2. 67340700403 พีรดนย์ เรืองแก้ว

## Reference
https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html
