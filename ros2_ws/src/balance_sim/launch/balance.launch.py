"""Gazebo先暂停；startup节点确认effort就绪后启动，交互面板提供扰动按钮。"""
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    p=Path(get_package_share_directory('balance_sim'))
    arg=lambda name:LaunchConfiguration(name).perform(context)
    if arg('wheel_contact') not in ('sphere', 'cylinder'):
        raise ValueError('wheel_contact must be sphere or cylinder')
    desc=xacro.process_file(
        str(p/'description/balance.urdf.xacro'),
        mappings={
            'wheel_contact':arg('wheel_contact')
            }
        ).toxml()

    import yaml
    r=yaml.safe_load(
        (p/'config/physical.yaml').read_text()
        )['r']
    
    gz=['gz','sim','-s','-v','3',str(p/'worlds/balance.sdf')]
    actions=[ExecuteProcess(cmd=gz,output='screen')]
    if arg('gui')=='true': 
        actions.append(ExecuteProcess(cmd=['gz','sim','-g'],output='screen'))

    actions.extend([
        #发布URDF描述，并根据joint_states计算TF；不是Gazebo实体创建器
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description':desc,'use_sim_time':True}]),
        #中间层，ros2和gazebo之间信息桥接，桥接规则似乎要自己定义
        Node(package='ros_gz_bridge',
             executable='parameter_bridge',
             parameters=[{'config_file':str(p/'config/bridge.yaml'),'use_sim_time':True}]),
        #在gazebo中创建机器人实体，指定模型、位置、姿态等，指定各个参数
        Node(package='ros_gz_sim',
             executable='create',
             arguments=['-world','balance_world',
                        '-name','balance',
                        '-topic','robot_description',
                        '-z',str(r+0.0001),
                        '-P',arg('pitch'),
                        '-Y',arg('yaw')],
             output='screen'),
        #短时spawner客户端：请求插件内的controller_manager加载并激活两个控制器
        Node(package='controller_manager',
             executable='spawner',
             arguments=['joint_state_broadcaster',
                        'wheel_effort',
                        '--controller-manager-timeout','120'],
                        output='screen'),
        #lqr控制算法控制器
        Node(package='balance_sim',
             executable='lqr_node',
             parameters=[str(p/'config/lqr.yaml'),
                        {'log_path':arg('log_path'),'disturbance':arg('disturbance')=='true',
                        'probe_torque':[float(arg('probe_left')),float(arg('probe_right'))]}],
             output='screen')
            ])
    
    if arg('auto_start') == 'true':
        actions.append(Node(package='balance_sim', executable='startup', output='screen'))
    if arg('panel') == 'true':
        actions.append(Node(package='balance_sim', executable='interaction_panel', output='screen'))
    return actions

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(k,default_value=v) 
            for k,v in {
                'wheel_contact':'sphere',
                'probe_left':'0.0',
                'probe_right':'0.0',
                'pitch':'0.03',
                'yaw':'0.0',
                'gui':'true',
                'auto_start':'true',
                'panel':'true',
                'disturbance':'false',
                'log_path':'/tmp/balance.csv'
            }.items()
        ]
        +[OpaqueFunction(function=setup)]
    )
