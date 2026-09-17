"""静止直立六状态 LQR。独立运行：python -m straight_balance.lqr（PYTHONPATH=theory）。"""
import numpy as np
import sympy as s
from straight_balance.lagrange import build,psi_0
from parameters import values,substitutions,numeric,solve_lqr,report

def symbolic():
    m=build()
    A=s.zeros(6);A[:3,3:]=s.eye(3)
    A[3:,:3]=-m['M'].inv()*m['K']  # 符号公式；数值计算下方用 solve。
    B=s.zeros(6,2);B[3:,:]=m['M'].inv()*m['B']
    return A.applyfunc(s.simplify),B.applyfunc(s.simplify)

def design(q_weights=None,r_weights=None):
    p,cfg=values(); sub=substitutions();sub[psi_0]=p['psi0'];m=build()
    M,C,K0,B0=[numeric(m[k],sub) for k in ('M','C','K','B')]
    A=np.block([[np.zeros((3,3)),np.eye(3)],[-np.linalg.solve(M,K0),-np.linalg.solve(M,C)]])
    B=np.vstack([np.zeros((3,2)),np.linalg.solve(M,B0)])
    Q=np.diag(cfg.q_weights if q_weights is None else q_weights)
    R=np.diag(cfg.r_weights if r_weights is None else r_weights)
    result=solve_lqr(A,B,Q,R);result.update(M=M,C=C,K0=K0,B0=B0,parameters=p)
    return result
if __name__=='__main__': report(design())
