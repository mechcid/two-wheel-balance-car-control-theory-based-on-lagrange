from pathlib import Path
from setuptools import setup
from glob import glob
setup(name='balance_yaw',version='0.1.0',packages=['balance_yaw'],
 data_files=[('share/ament_index/resource_index/packages',['resource/balance_yaw']),('share/balance_yaw',['package.xml'])]+[(f'share/balance_yaw/{d}',[f for f in glob(f'{d}/*') if Path(f).is_file()]) for d in ['config','launch','tools']],
 entry_points={'console_scripts':['yaw_node=balance_yaw.controller:main']})
