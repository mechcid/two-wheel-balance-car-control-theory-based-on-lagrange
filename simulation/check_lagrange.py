"""兼容旧校验入口；完整验收实现已迁到 theory/validate.py。"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'theory'))
from validate import main
if __name__=='__main__': main()
