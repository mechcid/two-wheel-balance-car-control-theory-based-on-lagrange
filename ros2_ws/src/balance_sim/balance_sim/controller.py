"""ROS力矩闭环。
启动后等待 IMU、odometry、joint_states 齐全；失去数据则零力矩。
CSV 每行是一次实际控制采样，不是重新求解线性方程生成的曲线。
"""

import csv
import signal
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from ros_gz_interfaces.msg import EntityWrench, Entity

from .state import extract


class Controller(Node):
    def __init__(self):
        super().__init__('balance_lqr')

        defaults = {
            'gain': [0.0] * 12,
            'control_frequency': 200.0,
            'torque_limit': 1.0,
            'reference_state': [0.0] * 6,
            'nominal_heading': 0.0,
            'log_path': '/tmp/balance.csv',
            'duration': 15.0,
            'disturbance': False,
            'probe_torque': [0.0, 0.0],
        }
        #声明可以从参数文件中接受的参数，并设置默认值
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        #定义一个便捷的lambda函数，用于获取参数值
        get = lambda k: self.get_parameter(k).value
        #读参数，存到类属性中
        self.K = np.array(get('gain')).reshape(2, 6)
        self.ref = np.array(get('reference_state'))
        self.limit = get('torque_limit')
        self.heading = get('nominal_heading')
        self.duration = get('duration')
        self.push = get('disturbance')
        self.probe = np.array(get('probe_torque'))

        #定义违法参数的条件，如果参数不符合要求，则抛出异常
        if (
            self.ref.shape != (6,)
            or not np.isfinite(self.K).all()
            or self.limit <= 0
            or get('control_frequency') <= 0
        ):
            raise ValueError('Invalid control parameters')

        self.data = {}
        self.received = {}
        self.start = None
        self.finished = False
        self.push_state = 0

        #定义订阅的的话题，话题返回信息类型，以及处理信息的回调函数
        subscriptions = [
            ('imu', Imu, '/balance/imu'),
            ('odom', Odometry, '/balance/odometry'),
            ('joints', JointState, '/joint_states'),
        ]
        for key, typ, topic in subscriptions:
            self.create_subscription(
                typ,
                topic,
                lambda msg, k=key: self.receive(k, msg),
                qos_profile_sensor_data,
            )

        #定义发布的话题，话题返回信息类型，以及队列大小
        self.pub = self.create_publisher(
            Float64MultiArray, '/wheel_effort/commands', 10
        )
        self.debug = self.create_publisher(
            Float64MultiArray, '/balance/state_and_torque', 10
        )
        self.wrench = self.create_publisher(
            EntityWrench, '/world/balance_world/wrench/persistent', 10
        )
        self.clear = self.create_publisher(
            Entity, '/world/balance_world/wrench/clear', 10
        )


        #记录各个参数的值到CSV文件中，便于后续分析，绘图等。与节点的逻辑没有关系
        path = Path(get('log_path'))
        path.parent.mkdir(parents=True, exist_ok=True)

        self.file = path.open('w', newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow([
            't',
            'x', 'psi', 'theta',
            'xdot', 'psidot', 'thetadot',
            'tau_L', 'tau_R',
            'q_L', 'q_R',
            'dq_L', 'dq_R',
            'saturated',
            'imu_age', 'odom_age',
        ])

        #调用ros2的定时器，每隔一定时间调用step函数，进行控制计算和发布命令，给系统一个时间基准，5ms一周期
        #每个周期调用一次step函数，作为节点的工作逻辑
        self.timer = self.create_timer(
            1 / get('control_frequency'),
            self.step,
        )
        #调试信息打印
        self.get_logger().info(
            f'K source: existing numerical export; '
            f'{get("control_frequency")} Hz, '
            f'limit {self.limit} Nm. DEMO parameters.'
        )

    def receive(self, key, msg):
        self.data[key] = msg
        self.received[key] = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

    def command(self, u):
        self.pub.publish(
            Float64MultiArray(data=list(map(float, u)))
        )

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if len(self.data) != 3 or any(
            now - stamp > 0.05 for stamp in self.received.values()
        ):
            self.command([0, 0])
            return

        if self.start is None:
            self.start = now

        t = now - self.start

        X = extract(
            self.data['imu'],
            self.data['odom'],
            self.heading,
        )

        if not np.isfinite(X).all() or abs(X[2]) > 0.6:
            self.command([0, 0])
            self.file.flush()

            if not self.finished:
                self.get_logger().error(
                    'Outside local pitch range; torque disabled.'
                )
                self.finished = True

            return

        if self.finished:
            self.command([0, 0])
            return

        raw = self.probe if np.any(self.probe) else -self.K @ (X - self.ref)
        u = np.clip(raw, -self.limit, self.limit)

        self.command(u)
        self.debug.publish(Float64MultiArray(data=[*X, *u]))

        j = self.data['joints']
        ids = [
            j.name.index('left_wheel_joint'),
            j.name.index('right_wheel_joint'),
        ]

        self.csv.writerow([
            t,
            *X,
            *u,
            *[j.position[i] for i in ids],
            *[j.velocity[i] for i in ids],
            int(np.any(abs(raw) > self.limit)),
            now - self.received['imu'],
            now - self.received['odom'],
        ])

        if self.push and self.push_state == 0 and t >= 3:
            msg = EntityWrench()
            msg.entity.name = 'balance::base_link'
            msg.entity.type = Entity.LINK
            msg.wrench.force.x = 1.0

            self.wrench.publish(msg)
            self.push_state = 1

            self.get_logger().info(
                'Requested world +x force 1 N for 0.1 s on chassis.'
            )

        if self.push_state == 1 and t >= 3.1:
            msg = Entity()
            msg.name = 'balance::base_link'
            msg.type = Entity.LINK

            self.clear.publish(msg)
            self.push_state = 2

        if t >= self.duration:
            self.file.flush()

            # Continue balancing while launch process awaits shutdown;
            # don't switch off at duration.
            if not hasattr(self, 'reported'):
                self.get_logger().info(
                    'Requested recording duration reached.'
                )
                self.reported = True


def main():
    # Keep the ROS context alive until zero torque and CSV flush finish.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)

    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

        if rclpy.ok():
            node.command([0, 0])

        node.file.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()