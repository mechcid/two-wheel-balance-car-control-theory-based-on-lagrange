#!/usr/bin/python3
"""Sequential physical yaw tests. Original balance_sim launch remains unchanged."""
import argparse,os,signal,subprocess,time,json
from pathlib import Path
import rclpy
from rosgraph_msgs.msg import Clock
from controller_manager_msgs.srv import ListHardwareInterfaces

def main():
    parser=argparse.ArgumentParser();parser.add_argument('case',choices=['turn','reverse','fixed_heading','turn_push']);parser.add_argument('--output',default='results_yaw');a=parser.parse_args()
    yaw,rate=( (-1.1,-.4) if a.case=='reverse' else ((1.2,0.) if a.case=='fixed_heading' else (.7,.4)))
    out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID='74',GZ_PARTITION=f'balance_yaw_{os.getpid()}',ROS_LOG_DIR=str(out/'ros_logs'))
    log=(out/f'{a.case}.log').open('w')
    proc=subprocess.Popen(['ros2','launch','balance_yaw','yaw.launch.py','gui:=false','panel:=false',f'yaw:={yaw}',f'yaw_rate:={rate}',f'disturbance:={str(a.case=="turn_push").lower()}',f'log_path:={out/a.case}.csv'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('yaw_supervisor');stamp=[0.]
    node.create_subscription(Clock,'/clock',lambda m:stamp.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
    hardware=node.create_client(ListHardwareInterfaces,'/controller_manager/list_hardware_interfaces');future=None
    try:
        deadline=time.monotonic()+240
        while stamp[0]<32 and proc.poll() is None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if stamp[0]>1 and future is None and hardware.service_is_ready():future=hardware.call_async(ListHardwareInterfaces.Request())
        if stamp[0]<32:raise RuntimeError('Simulation failed to reach32s; inspect log')
        if future is None or not future.done():raise RuntimeError('No hardware interface evidence')
        interfaces=[dict(name=i.name,claimed=i.is_claimed,available=i.is_available) for i in future.result().command_interfaces]
        (out/f'{a.case}.interfaces.json').write_text(json.dumps(interfaces,indent=2))
        assert len(interfaces)==2 and all(i['claimed'] and i['available'] for i in interfaces)
    finally:
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
    print(f'Physical yaw run complete: {a.case}')
if __name__=='__main__':main()
