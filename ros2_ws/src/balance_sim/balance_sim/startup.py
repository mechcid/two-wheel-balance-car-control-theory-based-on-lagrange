"""只负责暂停世界中的控制器就绪处理，不计算控制力矩。
使用墙钟timer等待服务：若使用仿真时间，暂停世界中timer本身无法推进。
"""
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from rosgraph_msgs.msg import Clock


def world_control(request):
    result = subprocess.run(
        ['gz', 'service', '-s', '/world/balance_world/control',
         '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req', request],
        capture_output=True, text=True, timeout=5)
    if result.returncode or 'true' not in result.stdout:
        raise RuntimeError(result.stdout + result.stderr or 'Gazebo world control failed')
    return result.stdout


class Startup(Node):
    def __init__(self):
        super().__init__('balance_startup')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.hardware = self.create_client(ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces')
        self.sim_time = 0.0
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.query = None
        self.work = None
        self.stage = 'controllers'
        self.done = False
        self.error = None
        self.deadline = time.monotonic() + 120
        self.timer = self.create_timer(0.25, self.step)

    def on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def step(self):
        try:
            self.advance()
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error(self.error)
            self.done = True

    def advance(self):
        if self.done:
            return
        if time.monotonic() > self.deadline:
            raise RuntimeError('Startup timeout; inspect Gazebo and spawner logs')
        if self.work is not None:
            if not self.work.done():
                return
            self.work.result()
            self.work = None
            if self.stage == 'starting':
                self.get_logger().info('Both effort interfaces claimed. World running; interaction panel ready.')
                self.done = True
                return
        client = self.controllers if self.stage == 'controllers' else self.hardware
        if not client.service_is_ready():
            return
        if self.query is None:
            request = ListControllers.Request() if self.stage == 'controllers' else ListHardwareInterfaces.Request()
            self.query = client.call_async(request)
            return
        if not self.query.done():
            # During a paused controller switch the manager may hold its lock,
            # delaying even ListControllers. A physics step must not wait for
            # that service response, otherwise startup deadlocks.
            if self.stage == 'controllers' and self.sim_time < 0.15:
                self.work = self.pool.submit(world_control, 'pause: true, multi_step: 1')
            return
        result = self.query.result()
        self.query = None
        if self.stage == 'controllers':
            active = {c.name for c in result.controller if c.state == 'active'}
            if {'wheel_effort', 'joint_state_broadcaster'} <= active:
                self.stage = 'hardware'
            elif self.sim_time < 0.15:
                self.work = self.pool.submit(world_control, 'pause: true, multi_step: 1')
            else:
                raise RuntimeError('Controllers not ready within 0.15 s startup stepping budget')
        else:
            claimed = {i.name for i in result.command_interfaces if i.is_available and i.is_claimed}
            required = {'left_wheel_joint/effort', 'right_wheel_joint/effort'}
            if not required <= claimed:
                raise RuntimeError(f'Missing claimed effort interfaces: {required - claimed}')
            self.stage = 'starting'
            self.work = self.pool.submit(world_control, 'pause: false')


def main():
    rclpy.init()
    node = Startup()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.error:
            raise RuntimeError(node.error)
    except KeyboardInterrupt:
        pass
    finally:
        node.pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
