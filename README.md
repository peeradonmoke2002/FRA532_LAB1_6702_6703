# FRA532 Mobile Robot: LAB1

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

## 📍 PID Controller Results

### Path Tracking
![PID Path](https://raw.githubusercontent.com/peeradonmoke2002/FRA532_LAB1_6702_6703/Path-Tracking-Controller/robot_controller/result/PID/PID_path.png)

### Speed Profile
![PID Speed](https://raw.githubusercontent.com/peeradonmoke2002/FRA532_LAB1_6702_6703/Path-Tracking-Controller/robot_controller/result/PID/PID_speed.png)

---

## 📍 Pure Pursuit Controller Results

### Path Tracking
![Pure Pursuit Path](https://raw.githubusercontent.com/peeradonmoke2002/FRA532_LAB1_6702_6703/Path-Tracking-Controller/robot_controller/result/purepursuit/purepursuit_path.png)

### Speed Profile
![Pure Pursuit Speed](https://raw.githubusercontent.com/peeradonmoke2002/FRA532_LAB1_6702_6703/Path-Tracking-Controller/robot_controller/result/purepursuit/purepursuit_speed.png)


## Pure Pursuit Controller



## Our Team

1. 67340700402 พงษ์พัฒน์ วงศ์กำแหงหาญ
2. 67340700403 พีรดนย์ เรืองแก้ว
