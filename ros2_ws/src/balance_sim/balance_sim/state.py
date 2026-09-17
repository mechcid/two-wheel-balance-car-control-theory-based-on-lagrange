"""状态来源适配：IMU姿态/体轴角速度，Gazebo模型原点的理想里程计。
轮关节角是轮相对车体的角度，不直接替代绝对轮角或车体速度。
"""
import math
import numpy as np


def rotation(q):
    """四元数对应的旋转矩阵：局部分量 → 参考系分量。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def extract(imu, odom, heading):
    R = rotation(imu.orientation)
    pitch = math.asin(float(np.clip(-R[2, 0], -1, 1)))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    omega = imu.angular_velocity

    # ZYX欧拉角速率：体轴角速度不能直接当成欧拉角导数。
    pitch_rate = math.cos(roll)*omega.y - math.sin(roll)*omega.z
    yaw_rate = (
        math.sin(roll)*omega.y + math.cos(roll)*omega.z
    ) / math.cos(pitch)

    # 固定名义朝向，而不是当前随车体旋转的前向。
    forward = np.array([math.cos(heading), math.sin(heading), 0.0])
    pos = odom.pose.pose.position
    v = odom.twist.twist.linear
    position = np.array([pos.x, pos.y, pos.z])
    local_velocity = np.array([v.x, v.y, v.z])

    # OdometryPublisher的twist用子坐标系分量；先转回world。
    world_v = rotation(odom.pose.pose.orientation) @ local_velocity
    yaw_error = math.atan2(math.sin(yaw-heading), math.cos(yaw-heading))

    return np.array([
        forward @ position,
        yaw_error,
        pitch,
        forward @ world_v,
        yaw_rate,
        pitch_rate,
    ])
