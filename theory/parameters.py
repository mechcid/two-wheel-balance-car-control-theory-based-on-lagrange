"""共用配置的数值代入及连续 LQR；快捷离散设计见 control/。"""
from types import SimpleNamespace
import numpy as np
import base as b
from configuration import load_config

def values():
    cfg = load_config()
    # 保留旧调用形式；当前理论不再导入仿真模块。
    return dict(cfg['physical']), SimpleNamespace(
        q_weights=cfg['origin_static_balance']['Q'], r_weights=cfg['origin_static_balance']['R'])

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
