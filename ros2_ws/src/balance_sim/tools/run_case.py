#!/usr/bin/python3
"""Run one isolated Gazebo case; refuse to proceed if effort plugins are absent.
调用前source ROS和本工作区。只终止本程序启动的进程组。
"""
import argparse, os, signal, subprocess, time, json
from pathlib import Path
import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from rosgraph_msgs.msg import Clock


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('case',choices=['upright','pitch','push','yaw','left_probe','right_probe']); ap.add_argument('--output',default='results'); ap.add_argument('--contact',choices=['sphere','cylinder'],default='sphere'); args=ap.parse_args()
    from ament_index_python.packages import get_package_share_directory
    for pkg in ['gz_ros2_control','effort_controllers','joint_state_broadcaster']:
        get_package_share_directory(pkg)
    output=Path(args.output).resolve(); output.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy(); env['ROS_LOG_DIR']=str(output/'ros_logs'); env['ROS_DOMAIN_ID']='73'; env['GZ_PARTITION']=f'balance_test_{os.getpid()}'
    os.environ.update(env)
    log=(output/f'{args.case}.log').open('w')
    cmd=['ros2','launch','balance_sim','balance.launch.py','gui:=false','panel:=false','auto_start:=false',f'wheel_contact:={args.contact}',f'pitch:={0.04 if args.case=="pitch" else 0.0}',f'yaw:={0.04 if args.case=="yaw" else 0.0}',f'disturbance:={str(args.case=="push").lower()}',f'log_path:={output/args.case}.csv',f'probe_left:={0.1 if args.case=="left_probe" else 0.0}',f'probe_right:={0.1 if args.case=="right_probe" else 0.0}']
    proc=subprocess.Popen(cmd,env=env,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init(); node=rclpy.create_node('test_supervisor'); stamp=[0.0]
    node.create_subscription(Clock,'/clock',lambda msg:stamp.__setitem__(0,msg.clock.sec+msg.clock.nanosec*1e-9),10)
    client=node.create_client(ListControllers,'/controller_manager/list_controllers')
    try:
        deadline=time.monotonic()+100
        ready=False
        while time.monotonic()<deadline and proc.poll() is None:
            if not client.wait_for_service(timeout_sec=1): continue
            future=client.call_async(ListControllers.Request()); rclpy.spin_until_future_complete(node,future,timeout_sec=2)
            if future.done() and future.result():
                active={c.name for c in future.result().controller if c.state=='active'}
                if {'wheel_effort','joint_state_broadcaster'}<=active: ready=True; break
            # Some ros2_control switches need a physics update while paused.
            # Advance one millisecond at a time, never a long uncontrolled run.
            if stamp[0] < 0.15:
                subprocess.run(['gz','service','-s','/world/balance_world/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','1000','--req','pause: true, multi_step: 1'],env=env,stdout=log,stderr=log,timeout=3)
        if not ready: raise RuntimeError('Effort controllers did not activate; inspect launch log.')
        hardware=node.create_client(ListHardwareInterfaces,'/controller_manager/list_hardware_interfaces')
        if not hardware.wait_for_service(timeout_sec=3): raise RuntimeError('No hardware interface service')
        future=hardware.call_async(ListHardwareInterfaces.Request()); rclpy.spin_until_future_complete(node,future,timeout_sec=3)
        if not future.done() or not future.result(): raise RuntimeError('No hardware interface response')
        interfaces=[{'name':i.name,'claimed':i.is_claimed,'available':i.is_available} for i in future.result().command_interfaces]
        (output/f'{args.case}.interfaces.json').write_text(json.dumps(interfaces,indent=2))
        assert all(any(i['name']==n+'/effort' and i['claimed'] and i['available'] for i in interfaces) for n in ['left_wheel_joint','right_wheel_joint'])
        # Initial zero command is retained while paused. Start time only after controller activation.
        reply=subprocess.run(['gz','service','-s','/world/balance_world/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: false'],env=env,capture_output=True,text=True,timeout=8)
        log.write(reply.stdout+reply.stderr); log.flush()
        if reply.returncode or 'true' not in reply.stdout: raise RuntimeError('World unpause failed')
        deadline=time.monotonic()+150
        target=0.12 if args.case.endswith('_probe') else 16
        while stamp[0]<target and time.monotonic()<deadline and proc.poll() is None: rclpy.spin_once(node,timeout_sec=.1)
        if stamp[0]<target: raise RuntimeError('Simulation did not reach 16 seconds')
    finally:
        proc.send_signal(signal.SIGINT)  # launch forwards SIGINT once to its children
        try: proc.wait(timeout=12)
        except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=5)
        log.close(); node.destroy_node(); rclpy.shutdown()
    print(f'Gazebo run completed; assess CSV with plot_results.py: {output/args.case}.csv')

if __name__=='__main__': main()
