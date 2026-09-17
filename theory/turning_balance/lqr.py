"""七状态转向跟踪。世界 Z=[x,y,psi,theta,v,omega,p]；不是全局定常AX+Bu。
恒定 v0/omega0 的参考轨迹上，旋转位置误差坐标可得到常系数 A_e,B_e。
CARE 求 K_e；世界坐标反馈增益为 K_world(t)=K_e H(psi_ref(t))。
"""
import numpy as np
import sympy as s
from turning_balance.lagrange import build,mW,Js,Jt,v,omega,p,theta,u
from base import q,psi,m_L,m_R,I_Lyy,I_Ryy,I_Lzz,I_Rzz
from parameters import values,substitutions,numeric,solve_lqr,report
DEFAULT_V0=.1
DEFAULT_OMEGA0=.4
Q_WEIGHTS=(10.,10.,5.,400.,5.,2.,40.)
R_WEIGHTS=(10.,10.)

def physical_subs():
    vals,_=values()
    for a,b in [('m_L','m_R'),('I_Lyy','I_Ryy'),('I_Lzz','I_Rzz')]:
        if vals[a]!=vals[b]:raise ValueError('转向推导当前采用左右相同轴对称车轮；不能静默平均不对称参数。')
    sub=substitutions();sub.update({mW:vals['m_L'],Js:vals['I_Lyy'],Jt:vals['I_Lzz']})
    return sub

def symbolic():
    m=build();v0,w0,pr=s.symbols('v_0 omega_0 psi_ref',real=True)
    M0=m['M_small'];h_theta=m['h_small'].diff(theta).subs({v:v0,omega:w0,p:0})
    acceleration=-M0.inv()*h_theta
    A=s.zeros(7);A[0,2]=-v0*s.sin(pr);A[0,4]=s.cos(pr)
    A[1,2]=v0*s.cos(pr);A[1,4]=s.sin(pr);A[2,5]=1;A[3,6]=1;A[4:,3]=acceleration
    B=s.zeros(7,2);B[4:,:]=M0.inv()*m['B']
    H=s.eye(7);H[:2,:2]=s.Matrix([[s.cos(pr),s.sin(pr)],[-s.sin(pr),s.cos(pr)]])
    Ae=(H*A*H.T+H.diff(pr)*w0*H.T).applyfunc(s.trigsimp)
    return dict(A=A,B=B,H=H,Ae=Ae,Be=B,v0=v0,w0=w0,psi_ref=pr)

def error_transform(heading):
    c,sn=np.cos(heading),np.sin(heading);H=np.eye(7);H[:2,:2]=[[c,sn],[-sn,c]];return H

def state_matrices(v0=DEFAULT_V0,w0=DEFAULT_OMEGA0,heading=0.):
    m=build();sub=physical_subs();M=numeric(m['M_small'],sub)
    htheta=numeric(m['h_small'].diff(theta).subs({v:v0,omega:w0,p:0}),sub).ravel()
    A=np.zeros((7,7));A[0,2]=-v0*np.sin(heading);A[0,4]=np.cos(heading)
    A[1,2]=v0*np.cos(heading);A[1,4]=np.sin(heading);A[2,5]=1;A[3,6]=1
    A[4:,3]=-np.linalg.solve(M,htheta)
    B=np.vstack([np.zeros((4,2)),np.linalg.solve(M,numeric(m['B'],sub))])
    H=error_transform(heading);Hdot=np.zeros((7,7));Hdot[:2,:2]=np.array([[0,w0],[-w0,0]])@H[:2,:2]
    Ae=H@A@H.T+Hdot@H.T
    return A,B,Ae,H

def design(v0=DEFAULT_V0,w0=DEFAULT_OMEGA0,q_weights=Q_WEIGHTS,r_weights=R_WEIGHTS):
    if abs(v0)<1e-10 and abs(w0)<1e-10:
        raise ValueError('静止七状态模型存在不可控横向位置零模态；请使用 straight_balance 六状态分支。')
    Aw,B,Ae,H=state_matrices(v0,w0)
    result=solve_lqr(Ae,B,np.diag(q_weights),np.diag(r_weights))
    result.update(A_world=Aw,H=H,v0=v0,w0=w0)
    return result

def reference(t,v0=DEFAULT_V0,w0=DEFAULT_OMEGA0,psi0=0.):
    heading=psi0+w0*t
    if abs(w0)>1e-12: x=v0/w0*(np.sin(heading)-np.sin(psi0));y=v0/w0*(np.cos(psi0)-np.cos(heading))
    else:x=v0*t*np.cos(psi0);y=v0*t*np.sin(psi0)
    return np.array([x,y,heading,0.,v0,w0,0.])

if __name__=='__main__':report(design())
