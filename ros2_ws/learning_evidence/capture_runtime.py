"""Read-only ROS graph audit while a separate physical test is running.
Each command records its purpose, return code and raw output. No parameter writes.
"""
import subprocess,time,json,concurrent.futures
from pathlib import Path
out=Path(__file__).resolve().parent
checks=[
 ('nodes','核实实际ROS节点，而不是把launch动作数当作节点数',['ros2','node','list','--no-daemon']),
 ('topics','核实ROS侧数据通道及消息类型',['ros2','topic','list','-t','--no-daemon']),
 ('command','核实力矩发布者、订阅者和QoS',['ros2','topic','info','/wheel_effort/commands','--verbose']),
 ('imu','核实IMU消息的坐标标签、姿态和角速度',['ros2','topic','echo','/balance/imu','--once']),
 ('odom','核实里程计父子frame和twist内容',['ros2','topic','echo','/balance/odometry','--once']),
 ('joints','核实关节名称、相对角、角速度、effort输出',['ros2','topic','echo','/joint_states','--once']),
 ('tf','核实robot_state_publisher实际输出的TF边',['ros2','topic','echo','/tf','--once']),
 ('services','核实controller_manager及参数服务',['ros2','service','list','--no-daemon']),
 ('actions','确认项目是否真的有action接口',['ros2','action','list']),
 ('parameters','核实用户节点声明的参数',['ros2','param','list','/balance_lqr']),
 ('frequency','核实实际参数值而非YAML推断',['ros2','param','get','/balance_lqr','control_frequency']),
 ('controllers','确认两项控制器都active',['ros2','service','call','/controller_manager/list_controllers','controller_manager_msgs/srv/ListControllers','{}']),
 ('hardware','确认effort被认领而position/velocity是反馈',['ros2','service','call','/controller_manager/list_hardware_interfaces','controller_manager_msgs/srv/ListHardwareInterfaces','{}']),
 ('message_type','查看命令的真实数据契约',['ros2','interface','show','std_msgs/msg/Float64MultiArray'])]
def run(item):
 name,purpose,cmd=item
 try:
  r=subprocess.run(cmd,capture_output=True,text=True,timeout=12)
  result=dict(purpose=purpose,command=cmd,returncode=r.returncode,output=r.stdout+r.stderr)
 except subprocess.TimeoutExpired as e:
  result=dict(purpose=purpose,command=cmd,returncode='timeout',output=str(e.stdout)+str(e.stderr))
 (out/f'{name}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
 return name,result['returncode']
# Let the supervised test activate controllers and start its clock.
for i in range(60):
 try:
  r=subprocess.run(['ros2','service','call','/controller_manager/list_controllers','controller_manager_msgs/srv/ListControllers','{}'],capture_output=True,text=True,timeout=8)
  if 'wheel_effort' in r.stdout and r.stdout.count('active')>=2: break
 except subprocess.TimeoutExpired: pass
 time.sleep(.5)
else: raise RuntimeError('No active controllers')
with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
 print(list(pool.map(run,checks)))
