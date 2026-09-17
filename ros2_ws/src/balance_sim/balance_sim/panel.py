"""Tk按钮面板；UI事件调用Interaction，LQR仍在另一个进程独立运行。"""
import tkinter as tk
import signal
from tkinter import ttk
from collections import deque
import rclpy
from rclpy.signals import SignalHandlerOptions
from .interaction import Interaction


class Panel:
    def __init__(self, root, node):
        self.root, self.node = root, node
        root.title('平衡车 · 扰动实验面板')
        # Tk fonts follow desktop DPI; scale the window too, or large-font
        # desktops clip the state values and leave no room for the graph.
        scale = max(1.0, float(root.tk.call('tk', 'scaling')) / (96 / 72))
        root.geometry(f'{int(740*scale)}x{int(580*scale)}')
        root.minsize(int(700*scale), int(540*scale))
        self.history = deque(maxlen=600)
        body = ttk.Frame(root, padding=16)
        body.pack(fill='both', expand=True)
        ttk.Label(body, text='Gazebo 两轮平衡车', font=('', 17, 'bold')).pack(anchor='w')
        ttk.Label(body, text='原 LQR 持续控制；按钮只施加车体外扰，不修改轮力矩或增益。', wraplength=int(680*scale)).pack(anchor='w', pady=6)
        self.status = tk.StringVar(value='等待状态')
        ttk.Label(body, textvariable=self.status, wraplength=int(680*scale)).pack(anchor='w', pady=4)
        buttons = ttk.Frame(body)
        buttons.pack(fill='x', pady=6)
        self.buttons = {}
        actions = [
            ('resume', '继续运行', lambda: node.pause(False)),
            ('pause', '暂停', lambda: node.pause(True)),
            ('clear', '清除外力', node.clear_force),
            ('forward', '向前推 1 N', lambda: node.pulse('x', 1.0)),
            ('backward', '向后推 1 N', lambda: node.pulse('x', -1.0)),
            ('yaw_left', '正偏航扰动', lambda: node.pulse('yaw', 0.05)),
            ('yaw_right', '负偏航扰动', lambda: node.pulse('yaw', -0.05)),
        ]
        for i, (key, title, action) in enumerate(actions):
            button = ttk.Button(buttons, text=title, command=lambda f=action: self.invoke(f))
            button.grid(row=i//4, column=i%4, padx=3, pady=3, sticky='ew')
            self.buttons[key] = button
        ttk.Label(body, text='每次扰动持续 0.1 仿真秒；偏航扰动力矩为 ±0.05 N·m。').pack(anchor='w')
        self.values = tk.StringVar()
        ttk.Label(body, textvariable=self.values, font=('monospace', 11)).pack(anchor='w', pady=10)
        self.canvas = tk.Canvas(body, width=650, height=170, background='#f7f9fc', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        ttk.Label(body, text='近 10 秒：蓝=俯仰 θ，橙=偏航 ψ（rad）。关闭面板后仿真仍运行；终端 Ctrl+C 结束整套试验。', wraplength=int(680*scale)).pack(anchor='w', pady=6)
        root.protocol('WM_DELETE_WINDOW', self.stop)
        self.closed = False
        self.refresh()

    def invoke(self, action):
        try:
            action()
        except Exception as exc:
            self.node.status = str(exc)

    def refresh(self):
        if self.closed:
            return
        if not rclpy.ok():
            self.stop()
            return
        # Bounded ROS work in the GUI event loop, never block Tk waiting for a service.
        for _ in range(4):
            rclpy.spin_once(self.node, timeout_sec=0)
        n = self.node
        self.status.set(f't={n.sim_time:.3f}s  {"运行中" if n.running else "暂停/等待"} ｜ {n.status}')
        if n.state:
            x, psi, theta, vx, wz, wy, left, right = n.state
            self.values.set(f'x={x:+.4f} m   ψ={psi:+.4f} rad   θ={theta:+.4f} rad\n'
                            f'ẋ={vx:+.4f} m/s   ψ̇={wz:+.4f}   θ̇={wy:+.4f} rad/s\n'
                            f'τL={left:+.4f} N·m   τR={right:+.4f} N·m')
            if not self.history or n.sim_time > self.history[-1][0]:
                self.history.append((n.sim_time, theta, psi))
            points = [p for p in self.history if n.sim_time-p[0] <= 10]
            width, height = max(400,self.canvas.winfo_width()), max(120,self.canvas.winfo_height())
            bound = max(0.005, max(abs(v) for p in points for v in p[1:]))
            self.canvas.delete('all')
            self.canvas.create_line(10,height/2,width-10,height/2,fill='#bbc3ce')
            self.canvas.create_text(8,10,text=f'±{bound:.4f} rad',anchor='nw')
            for index,color in [(1,'#2266cc'),(2,'#df701b')]:
                xy=[]
                for p in points:
                    xy.extend([10+(p[0]-(n.sim_time-10))/10*(width-20),height/2-p[index]/bound*(height/2-24)])
                if len(xy)>=4:
                    self.canvas.create_line(*xy,fill=color,width=2)
        self.root.after(10,self.refresh)

    def stop(self):
        if self.closed:
            return
        self.closed=True
        if rclpy.ok():
            self.node.close()
        self.root.destroy()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Interaction()
    root=tk.Tk()
    panel=Panel(root,node)
    def request_stop(signum, frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        root.after(0, panel.stop)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        panel.stop()
    finally:
        node.pool.shutdown(wait=True,cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
