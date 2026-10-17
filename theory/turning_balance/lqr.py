"""精确动力学 -> 可行匀速参考 -> 完整雅可比 -> 七状态旋转误差 LQR。
只在可行的恒定 v0/omega0 参考上使用常增益。质心采用 [0,0,l]。
"""
from functools import lru_cache
import numpy as np
import sympy as s
from scipy.optimize import brentq
import base as b
from turning_balance.lagrange import build,mW,Js,Jt,v,omega,p,theta,u
from parameters import values,substitutions,numeric,solve_lqr,report
DEFAULT_V0=.1
DEFAULT_OMEGA0=.4
Q_WEIGHTS=(10.,10.,5.,400.,5.,2.,40.)
R_WEIGHTS=(10.,10.)

def physical_subs():
    vals,_=values()
    for left,right in [('m_L','m_R'),('I_Lyy','I_Ryy'),('I_Lzz','I_Rzz'),('I_Lxx','I_Lzz'),('I_Rxx','I_Rzz')]:
        if vals[left]!=vals[right]:raise ValueError('转向推导采用左右相同轴对称车轮；不能静默平均参数。')
    sub=substitutions();sub.update({mW:vals['m_L'],Js:vals['I_Lyy'],Jt:vals['I_Lzz']})
    return sub

@lru_cache(None)
def exact_functions():
    m=build();sub=physical_subs()
    args=(theta,v,omega,p)
    return (s.lambdify(theta,m['M'].subs(sub),'numpy',cse=True),
            s.lambdify(args,m['h'].subs(sub),'numpy',cse=True),
            numeric(m['B'],sub),
            s.lambdify(theta,m['M'].diff(theta).subs(sub),'numpy',cse=True),
            s.lambdify(args,m['h'].jacobian(s.Matrix(args)).subs(sub),'numpy',cse=True))

def trim(v0=DEFAULT_V0,w0=DEFAULT_OMEGA0):
    """求直立附近恒定速度解。返回副本，避免调用者修改缓存/共享数组。"""
    if not np.isfinite([v0,w0]).all():raise ValueError('参考速度必须有限')
    _,hf,B,_,_=exact_functions();vals,_=values();r=vals['r'];d=vals['d']
    def residual(th):
        h=np.asarray(hf(th,v0,w0,0.)).ravel()
        return h[2]+r*h[0]
    # 定义局部参考范围，找不到根时明确拒绝，而不是把不可行参考交给 LQR。
    th=brentq(residual,-.3,.3,xtol=1e-14)
    h=np.asarray(hf(th,v0,w0,0.)).ravel()
    total=r*h[0];difference=2*r*h[1]/d
    tau=np.array([(total-difference)/2,(total+difference)/2])
    err=float(np.max(abs(B@tau-h)))
    if err>1e-10:raise ValueError(f'参考动力学残差过大: {err}')
    return dict(theta=th,tau=tau,residual=err)

def rhs(z,tau):
    mf,hf,B,_,_=exact_functions()
    x,y,heading,th,vv,w,pp=z
    acc=np.linalg.solve(mf(th),B@np.asarray(tau)-np.asarray(hf(th,vv,w,pp)).ravel())
    return np.r_[vv*np.cos(heading),vv*np.sin(heading),w,pp,acc]

@lru_cache(None)
def symbolic():
    """保留逆算子与分块形式，避免展开完整惯量的巨大有理式。"""
    m=build();v0,w0,pr,tr=s.symbols('v_0 omega_0 psi_ref theta_ref',real=True)
    ref={theta:tr,v:v0,omega:w0,p:0}
    Mr=m['M'].subs(ref);hr=m['h'].subs(ref)
    # 匀速可行参考满足 udot_r=0，因此 d(M^-1)/dq 乘残差的项为零。
    da=-s.Inverse(s.MatrixSymbol('M_r',3,3))*m['h'].jacobian(s.Matrix([theta,v,omega,p])).subs(ref)
    top=s.zeros(4);top[0,2]=-v0*s.sin(pr);top[1,2]=v0*s.cos(pr)
    kin=s.Matrix([[s.cos(pr),0,0],[s.sin(pr),0,0],[0,1,0],[0,0,1]])
    A=s.BlockMatrix([[top,kin],[s.BlockMatrix([[s.zeros(3,3),da[:,0:1]]]),da[:,1:4]]])
    B=s.BlockMatrix([[s.zeros(4,2)],[s.Inverse(s.MatrixSymbol('M_r',3,3))*m['B']]])
    H=s.eye(7);H[:2,:2]=s.Matrix([[s.cos(pr),s.sin(pr)],[-s.sin(pr),s.cos(pr)]])
    Ae=H*A*H.T+H.diff(pr)*w0*H.T
    return dict(A=A,B=B,H=H,Ae=Ae,Be=H*B,v0=v0,w0=w0,psi_ref=pr,
                theta_ref=tr,M_ref=Mr,h_ref=hr,trim_equation=hr[2]+b.r*hr[0])

def error_transform(heading):
    c,sn=np.cos(heading),np.sin(heading);H=np.eye(7);H[:2,:2]=[[c,sn],[-sn,c]];return H

def state_matrices(v0=DEFAULT_V0,w0=DEFAULT_OMEGA0,heading=0.):
    ref=trim(v0,w0);th=ref['theta'];mf,hf,Btau,dmf,dhf=exact_functions()
    M=mf(th);acc=np.linalg.solve(M,Btau@ref['tau']-np.asarray(hf(th,v0,w0,0.)).ravel())
    # 隐函数微分 M da = -dh - dM a；保留全部 theta,v,omega,p 导数。
    partial=-np.asarray(dhf(th,v0,w0,0.));partial[:,0]-=dmf(th)@acc
    A=np.zeros((7,7));A[0,2]=-v0*np.sin(heading);A[0,4]=np.cos(heading)
    A[1,2]=v0*np.cos(heading);A[1,4]=np.sin(heading);A[2,5]=1;A[3,6]=1
    A[4:,3:]=np.linalg.solve(M,partial)
    B=np.vstack([np.zeros((4,2)),np.linalg.solve(M,Btau)])
    H=error_transform(heading);Hdot=np.zeros((7,7));Hdot[:2,:2]=np.array([[0,w0],[-w0,0]])@H[:2,:2]
    Ae=H@A@H.T+Hdot@H.T
    return A,B,Ae,H

def design(v0=DEFAULT_V0,w0=DEFAULT_OMEGA0,q_weights=Q_WEIGHTS,r_weights=R_WEIGHTS):
    if abs(v0)<1e-10 and abs(w0)<1e-10:
        raise ValueError('静止七状态模型存在不可控横向位置零模态；请使用 straight_balance 六状态分支。')
    Aw,B,Ae,H=state_matrices(v0,w0)
    result=solve_lqr(Ae,B,np.diag(q_weights),np.diag(r_weights));ref=trim(v0,w0)
    result.update(A_world=Aw,H=H,v0=v0,w0=w0,theta_ref=ref['theta'],tau_ref=ref['tau'],trim_residual=ref['residual'])
    return result

@lru_cache(None)
def reference_pitch(v0,w0):return trim(v0,w0)['theta']

def reference(t,v0=DEFAULT_V0,w0=DEFAULT_OMEGA0,psi0=0.):
    heading=psi0+w0*t
    if abs(w0)>1e-12:x=v0/w0*(np.sin(heading)-np.sin(psi0));y=v0/w0*(np.cos(psi0)-np.cos(heading))
    else:x=v0*t*np.cos(psi0);y=v0*t*np.sin(psi0)
    return np.array([x,y,heading,reference_pitch(v0,w0),v0,w0,0.])

if __name__=='__main__':
    result=design();report(result)
    print('theta_ref =',result['theta_ref'],'tau_ref =',result['tau_ref'],'trim residual =',result['trim_residual'])
