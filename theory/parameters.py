"""实车 CAD 参数唯一入口；Q/R 沿用原示例配置，需重新求解增益。"""
from pathlib import Path
import importlib.util
import json
import numpy as np
import base as b
ROOT=Path(__file__).resolve().parents[1]

def values():
    # 读取旧仿真文件的控制权重默认值，不执行 main()，也不读取过期输出矩阵。
    import sys
    spec=importlib.util.spec_from_file_location('balance_numeric_source',ROOT/'simulation/lqr_simulation.py')
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    p=json.loads((ROOT/'theory/data/physical.json').read_text())['parameters']
    for name in ('B','L','R'):
        J=np.array([[p[f'I_{name}xx'],p.get(f'I_{name}xy',0),p.get(f'I_{name}xz',0)],
                    [p.get(f'I_{name}xy',0),p[f'I_{name}yy'],p.get(f'I_{name}yz',0)],
                    [p.get(f'I_{name}xz',0),p.get(f'I_{name}yz',0),p[f'I_{name}zz']]])
        eig=np.linalg.eigvalsh(J)
        if eig[0]<=0 or eig[-1]>eig[:2].sum(): raise ValueError(f'Invalid inertia: {name}')
    return p,module.CONFIG

def substitutions():
    p,_=values()
    return {getattr(b,'l' if k=='l' else k):v for k,v in p.items() if k!='psi0'}

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
    print('CAD 质量惯量及用户尺寸；理想模型计算，未作实车验收。')
    for key in ('A','B','Q','R','P','K','rank','poles','care_residual'): print(key,'=\n',result[key])
