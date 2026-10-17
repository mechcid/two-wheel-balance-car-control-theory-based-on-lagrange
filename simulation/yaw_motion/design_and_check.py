"""旧入口兼容：转到新的七状态数学仿真；不再生成或覆盖 ROS 控制器配置。"""
from pathlib import Path
import runpy
import sys
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root))
if __name__=='__main__':
    sys.argv=[str(root/'run_all.py'),'--scenario','turning']
    runpy.run_path(str(root/'run_all.py'),run_name='__main__')
