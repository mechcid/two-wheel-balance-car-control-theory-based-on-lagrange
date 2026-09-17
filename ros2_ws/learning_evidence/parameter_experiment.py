"""Isolated node with no physics: does parameter set update the cached torque limit?"""
from pathlib import Path
import json
import rclpy
from rclpy.parameter import Parameter
from balance_sim.controller import Controller
out=Path(__file__).resolve().parent
rclpy.init(args=['--ros-args','-p',f'log_path:={out}/parameter_experiment.csv'])
n=Controller()
before=dict(parameter=n.get_parameter('torque_limit').value,member=n.limit)
r=n.set_parameters([Parameter('torque_limit',value=0.2)])
after=dict(parameter=n.get_parameter('torque_limit').value,member=n.limit,successful=r[0].successful)
(out/'parameter_update.json').write_text(json.dumps(dict(before=before,after=after),indent=2))
print(before,after)
n.file.close();n.destroy_node();rclpy.shutdown()
