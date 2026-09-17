"""复用已验证数值仿真的参数、Q/R；不另造一套车辆数据。"""
from pathlib import Path
import importlib.util
from dataclasses import asdict
import json
import numpy as np
import base as b
ROOT=Path(__file__).resolve().parents[1]

def values():
    # 读取旧仿真文件的参数默认值，不执行 main()，也不读取过期输出矩阵。
    import sys
    spec=importlib.util.spec_from_file_location('balance_numeric_source',ROOT/'simulation/lqr_simulation.py')
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    p=asdict(module.PARAMS)
    # 转向模型额外需要 I_Bxx：从现有 Gazebo 物理参数读取（该键为简单数值）。
    physical=ROOT/'ros2_ws/src/balance_sim/config/physical.yaml'
    for line in physical.read_text().splitlines():
        if line.startswith('I_Bxx:'): p['I_Bxx']=float(line.split(':',1)[1])
    p['I_Lxx']=p['I_Lzz'];p['I_Rxx']=p['I_Rzz']
    return p,module.CONFIG

def substitutions():
    p,_=values()
    return {getattr(b,'lc' if k=='l' else k):v for k,v in p.items() if k!='psi0'}

def numeric(matrix,subs): return np.array(matrix.subs(subs),dtype=float)

def solve_lqr(A,B,Q,R):
    from scipy.linalg import solve_continuous_are
    ctr=np.hstack([np.linalg.matrix_power(A,i)@B for i in range(len(A))])
    rank=int(np.linalg.matrix_rank(ctr))
    P=solve_continuous_are(A,B,Q,R)
    K=np.linalg.solve(R,B.T@P)
    residual=A.T@P+P@A-P@B@K+Q
    poles=np.linalg.eigvals(A-B@K)
    assert np.max(poles.real)<0
    assert np.linalg.norm(residual,np.inf)<1e-6
    return dict(A=A,B=B,Q=Q,R=R,P=P,K=K,rank=rank,poles=poles,
                care_residual=float(np.linalg.norm(residual,np.inf)))

def report(result):
    np.set_printoptions(precision=8,suppress=True,linewidth=130)
    print('临时示例参数，不是实车测量值。')
    for key in ('A','B','Q','R','P','K','rank','poles','care_residual'): print(key,'=\n',result[key])
