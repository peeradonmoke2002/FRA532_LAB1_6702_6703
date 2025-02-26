
colcon build
source install/setup.bash
export ROS_WORKSPACE=~/ros2_lab1_m
source  ~/.bashrc


Parameter	Before	After (ปรับปรุง)	ผลที่คาดหวัง
Q (Position Noise)	0.02	0.01	เชื่อ EKF มากขึ้น (ตำแหน่งนิ่งขึ้น)
Q (Yaw Noise)	0.1	0.2	อนุญาตให้ yaw เปลี่ยนแปลงได้ดีขึ้น
Q (Angular Velocity Noise)	0.1	0.05	ลด jitter ใน heading
R_odom (Position & Velocity Noise)	0.3	0.5	ลดอิทธิพลของ Odom
R_imu (Yaw Noise)	0.1	0.15	ให้ IMU ควบคุม yaw มากขึ้น
R_imu (Acceleration Noise)	0.2	0.3	ให้ acceleration smooth ขึ้น