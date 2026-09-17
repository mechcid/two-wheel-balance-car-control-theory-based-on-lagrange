"""交互工具的ROS部分；通过既有wrench桥施加外扰，不直接改LQR或轮力矩。
脉冲期限按仿真时间计算。暂停/退出主动清除外力。
"""
import time
from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from ros_gz_interfaces.msg import EntityWrench, Entity
from .startup import world_control


class Interaction(Node):
    def __init__(self):
        super().__init__('balance_interaction')
        self.sim_time = 0.0
        self.last_clock_wall = 0.0
        self.last_state_wall = 0.0
        self.state = None
        self.until = None
        self.status = '等待仿真状态'
        self.pending = None
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.wrench = self.create_publisher(EntityWrench, '/world/balance_world/wrench/persistent', 10)
        self.clear_pub = self.create_publisher(Entity, '/world/balance_world/wrench/clear', 10)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(Float64MultiArray, '/balance/state_and_torque', self.on_state, qos_profile_sensor_data)
        # Wall timer remains alive when simulation is paused.
        self.create_timer(0.01, self.maintenance)

    def on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        if value != self.sim_time:
            self.last_clock_wall = time.monotonic()
        self.sim_time = value

    def on_state(self, msg):
        if len(msg.data) == 8:
            self.state = list(msg.data)
            self.last_state_wall = time.monotonic()
            if self.status == '等待仿真状态':
                self.status = '状态已就绪，可施加小扰动'

    @property
    def running(self):
        return time.monotonic() - self.last_clock_wall < 0.5

    def pulse(self, axis, magnitude, duration=0.1):
        if self.pending or self.until is not None:
            raise RuntimeError('请等待当前操作完成，或先清除外力')
        if not self.running or self.state is None or time.monotonic() - self.last_state_wall > 0.5:
            raise RuntimeError('仿真暂停或状态未就绪，请先继续运行')
        if abs(self.state[2]) > 0.3:
            raise RuntimeError('已超出当前小扰动实验范围，请重启试验')
        if not self.wrench.get_subscription_count():
            raise RuntimeError('外力桥接尚未就绪')
        if (axis, magnitude) not in [('x', 1.0), ('x', -1.0), ('yaw', 0.05), ('yaw', -0.05)]:
            raise ValueError('Only configured small test disturbances are supported')
        msg = EntityWrench()
        msg.entity.name = 'balance::base_link'
        msg.entity.type = Entity.LINK
        if axis == 'x':
            msg.wrench.force.x = magnitude
        else:
            msg.wrench.torque.z = magnitude
        self.wrench.publish(msg)
        self.until = self.sim_time + duration
        self.status = f'已请求 {axis} 扰动 {magnitude:+g}，持续 {duration:g} 仿真秒'
        self.get_logger().info(self.status)

    def clear_force(self):
        msg = Entity()
        msg.name = 'balance::base_link'
        msg.type = Entity.LINK
        self.clear_pub.publish(msg)
        self.until = None
        self.status = '已请求清除外力'

    def pause(self, paused):
        if self.pending:
            raise RuntimeError('正在等待Gazebo响应')
        if paused:
            self.clear_force()
        self.pending = self.pool.submit(world_control, f'pause: {str(paused).lower()}')
        self.status = '正在暂停' if paused else '正在继续'

    def maintenance(self):
        if self.until is not None and self.sim_time >= self.until:
            self.clear_force()
        if self.pending and self.pending.done():
            try:
                self.pending.result()
                self.status = 'Gazebo已确认操作'
            except Exception as exc:
                self.status = str(exc)
            self.pending = None

    def close(self):
        self.clear_force()
        self.pool.shutdown(wait=True, cancel_futures=True)
