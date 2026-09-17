from pathlib import Path
from setuptools import setup
from glob import glob
setup(name='balance_sim',
    version='0.1.0',
    packages=['balance_sim'],
    data_files=[
            ('share/ament_index/resource_index/packages',['resource/balance_sim']),
            ('share/balance_sim',['package.xml'])]
        +[
            (f'share/balance_sim/{d}',[f for f in glob(f'{d}/*') if Path(f).is_file()]) 
            for d in ['config','description','launch','worlds','tools']],
    entry_points={
        'console_scripts':[
            'lqr_node=balance_sim.controller:main',
            'startup=balance_sim.startup:main',
            'interaction_panel=balance_sim.panel:main']})
