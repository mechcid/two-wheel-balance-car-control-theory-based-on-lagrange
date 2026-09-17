"""Preflight: check XML physical values, K provenance, state frame conversion.
This is NOT evidence of Gazebo closed-loop stability.
"""
from pathlib import Path
import json, math, subprocess, xml.etree.ElementTree as ET
import numpy as np
import yaml
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from balance_sim.state import extract
from ament_index_python.packages import get_package_share_directory
p=Path(get_package_share_directory('balance_sim'))
xml=subprocess.check_output(['xacro',str(p/'description/balance.urdf.xacro')],text=True)
root=ET.fromstring(xml)
for link in root.findall('link'):
    inertia=link.find('inertial/inertia'); diag=np.array([float(inertia.get(k)) for k in ['ixx','iyy','izz']])
    assert min(diag)>0 and 2*max(diag)<=sum(diag)+1e-12
joints=root.findall('joint'); assert len(joints)==2
for joint in joints: assert joint.find('axis').get('xyz')=='0 1 0' and joint.get('type')=='continuous'
imu=Imu(); odom=Odometry()
a=.04; imu.orientation.y=math.sin(a/2); imu.orientation.w=math.cos(a/2); imu.angular_velocity.y=.3
odom.pose.pose.orientation.w=1.; odom.pose.pose.position.x=.2; odom.twist.twist.linear.x=.1
np.testing.assert_allclose(extract(imu,odom,0),[.2,0,.04,.1,0,.3],atol=1e-12)
# Heading projection and child-frame velocity conversion at yaw=pi/2.
imu.orientation.y=0.; imu.orientation.z=math.sin(math.pi/4); imu.orientation.w=math.cos(math.pi/4)
odom.pose.pose.orientation=imu.orientation; odom.pose.pose.position.x=0.; odom.pose.pose.position.y=.2
np.testing.assert_allclose(extract(imu,odom,math.pi/2),[.2,0,0,.1,0,.3],atol=1e-12)
source=Path(__file__).resolve().parents[4]/'simulation/outputs/results.json'
original=json.loads(source.read_text())
config=yaml.safe_load((p/'config/lqr.yaml').read_text())['balance_lqr']['ros__parameters']
np.testing.assert_array_equal(np.array(config['gain']).reshape(2,6),original['matrices']['K_LQR'])
physical=yaml.safe_load((p/'config/physical.yaml').read_text())
for k,v in original['parameters'].items(): assert physical[k]==v
assert physical['psi0']==0, 'Current gain export is for heading zero'
print('PASS: inertias, joint axes, two state transforms, exact gain and parameter reuse. No physical rollout tested here.')
