"""Moving-yaw branch. l = integral of current-heading forward speed.
This node never interprets world x as travel l. Original balance_lqr is unchanged.
"""
import csv,math,signal
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from ros_gz_interfaces.msg import Entity,EntityWrench
from balance_sim.state import rotation
from .dynamics import reference

class YawController(Node):
    def __init__(self):
        super().__init__('balance_yaw')
        for key,val in dict(gain=[0.]*12,control_frequency=200.,torque_limit=1.,forward_speed=.1,yaw_rate=.4,heading0=.7,log_path='/tmp/balance_yaw.csv',disturbance=False).items(): self.declare_parameter(key,val)
        get=lambda k:self.get_parameter(k).value
        self.K=np.array(get('gain')).reshape(2,6)
        if not np.isfinite(self.K).all() or not np.any(self.K):raise ValueError('Valid nonzero gain required')
        self.limit=get('torque_limit'); self.v0=get('forward_speed');self.w0=get('yaw_rate');self.heading=get('heading0');self.push=get('disturbance')
        self.data={};self.start=None;self.last=None;self.l=0.;self.psi=None;self.last_yaw=None;self.prev_v=0.;self.pushed=False;self.cleared=False;self.fault=False
        for key,typ,topic in [('imu',Imu,'/balance/imu'),('odom',Odometry,'/balance/odometry'),('joints',JointState,'/joint_states')]:self.create_subscription(typ,topic,lambda m,k=key:self.data.__setitem__(k,m),qos_profile_sensor_data)
        self.pub=self.create_publisher(Float64MultiArray,'/wheel_effort/commands',10)
        self.wrench=self.create_publisher(EntityWrench,'/world/balance_world/wrench/persistent',10)
        self.clear=self.create_publisher(Entity,'/world/balance_world/wrench/clear',10)
        out=Path(get('log_path'));out.parent.mkdir(parents=True,exist_ok=True);self.file=out.open('w',newline='');self.csv=csv.writer(self.file)
        self.csv.writerow(['t','l','psi','theta','v','omega','p','l_ref','psi_ref','theta_ref','v_ref','omega_ref','p_ref','x_world','y_world','roll','v_lateral','tau_L','tau_R','saturated','imu_age','odom_age'])
        self.create_timer(1/get('control_frequency'),self.step)
        self.get_logger().info('New branch: l=forward travel; COM height is l_c, not coordinate l.')

    def command(self,u): self.pub.publish(Float64MultiArray(data=list(map(float,u))))
    def clear_force(self):
        msg=Entity();msg.name='balance::base_link';msg.type=Entity.LINK;self.clear.publish(msg);self.cleared=True
    def step(self):
        now=self.get_clock().now().nanoseconds*1e-9
        t=0. if self.start is None else now-self.start
        if self.pushed and not self.cleared and t>=12.1:self.clear_force()
        if len(self.data)<3:self.command([0,0]);return
        ages={k:now-(m.header.stamp.sec+m.header.stamp.nanosec*1e-9) for k,m in self.data.items()}
        if max(ages.values())>.05:self.command([0,0]);return
        imu,odom=self.data['imu'],self.data['odom'];R=rotation(imu.orientation)
        th=math.asin(float(np.clip(-R[2,0],-1,1)));roll=math.atan2(R[2,1],R[2,2]);yaw=math.atan2(R[1,0],R[0,0])
        gy,gz=imu.angular_velocity.y,imu.angular_velocity.z
        pp=math.cos(roll)*gy-math.sin(roll)*gz;w=(math.sin(roll)*gy+math.cos(roll)*gz)/math.cos(th)
        vel=odom.twist.twist.linear;vw=rotation(odom.pose.pose.orientation)@np.array([vel.x,vel.y,vel.z])
        v=math.cos(yaw)*vw[0]+math.sin(yaw)*vw[1];lateral=-math.sin(yaw)*vw[0]+math.cos(yaw)*vw[1]
        if self.start is None:
            # Ignore the first odometry derivative transient by waiting until world t > .03.
            if now<.03:self.command([0,0]);return
            self.start=now;self.last=now;self.last_yaw=yaw
            self.psi=self.heading+math.atan2(math.sin(yaw-self.heading),math.cos(yaw-self.heading));self.prev_v=v;t=0.
        dt=now-self.last
        self.psi+=math.atan2(math.sin(yaw-self.last_yaw),math.cos(yaw-self.last_yaw))
        self.l+=.5*(v+self.prev_v)*dt;self.last=now;self.prev_v=v;self.last_yaw=yaw
        X=np.array([self.l,self.psi,th,v,w,pp]);ref=reference(t,self.v0,self.w0,self.heading)
        if not np.isfinite(X).all() or abs(th)>.5 or abs(roll)>.25:
            self.fault=True;self.command([0,0]);self.file.flush();return
        if self.fault:self.command([0,0]);return
        raw=-self.K@(X-ref);u=np.clip(raw,-self.limit,self.limit);self.command(u)
        pos=odom.pose.pose.position
        self.csv.writerow([t,*X,*ref,pos.x,pos.y,roll,lateral,*u,int(np.any(abs(raw)>self.limit)),ages['imu'],ages['odom']])
        if self.push and not self.pushed and t>=12:
            m=EntityWrench();m.entity.name='balance::base_link';m.entity.type=Entity.LINK
            m.wrench.force.x=math.cos(yaw);m.wrench.force.y=math.sin(yaw)
            self.wrench.publish(m);self.pushed=True;self.get_logger().info('1 N forward perturbation for .1 s while turning')
        if int(t*200)%200==0:self.file.flush()

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def stop(*args):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    n=YawController()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        n.command([0,0])
        if n.pushed and not n.cleared:n.clear_force()
        n.file.close();n.destroy_node();rclpy.shutdown()
