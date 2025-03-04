import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def plot_covariance_ellipse(mean, cov, ax, n_std=2.0, edgecolor='blue'):
    """
    วาด ellipse จาก covariance matrix ที่มี mean เป็นจุดศูนย์กลาง
    Parameters:
      mean    : [x, y]
      cov     : 2x2 covariance matrix
      ax      : matplotlib Axes object สำหรับ plot
      n_std   : จำนวน standard deviations ที่ต้องการแสดง (เช่น 2 = ประมาณ 95% confidence)
      edgecolor: สีของขอบ ellipse
    """
    # หา eigenvalues และ eigenvectors
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]

    # คำนวณมุมของ ellipse (องศา)
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))

    # คำนวณความกว้างและความสูงของ ellipse
    width, height = 2 * n_std * np.sqrt(vals)

    ellipse = Ellipse(xy=mean, width=width, height=height, angle=theta,
                      edgecolor=edgecolor, facecolor='none', lw=2)
    ax.add_patch(ellipse)
    return ellipse

# สมมติว่าคุณมี callback ที่รับ Odometry ที่ผ่าน EKF filter
def odom_callback(msg):
    # ดึงตำแหน่งจาก odometry
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    estimated_position = np.array([x, y])
    
    # ดึง covariance matrix 6x6 แล้วเอาเฉพาะ block 2x2 สำหรับ x, y
    # (โดยที่ covariance ถูก flatten เป็น list ยาว 36 ตัว)
    cov_full = np.array(msg.pose.covariance).reshape(6,6)
    pos_cov = cov_full[0:2, 0:2]  # block สำหรับตำแหน่ง x,y
    
    # Plot position และ uncertainty ellipse
    fig, ax = plt.subplots()
    ax.plot(estimated_position[0], estimated_position[1], 'ro', label='Filtered Odometry')
    plot_covariance_ellipse(estimated_position, pos_cov, ax, n_std=2.0, edgecolor='green')
    ax.set_xlabel("X position")
    ax.set_ylabel("Y position")
    ax.set_title("Filtered Odometry with Gaussian Uncertainty Ellipse")
    ax.legend()
    ax.grid(True)
    plt.axis('equal')
    plt.show()

# ตัวอย่างนี้สามารถนำไป integrate กับ ROS2 subscriber callback ของคุณได้
