"""显式拉格朗日算子与非完整约束投影；不使用凯恩方法。"""
import sympy as s
from base import q,qd,qdd,clean
B6=s.Matrix([[0,0],[0,0],[0,0],[-1,-1],[1,0],[0,1]])
tau=s.Matrix(s.symbols('tau_L tau_R',real=True))

def derive(T,V,Acon,S,u):
    L=T-V
    momentum=s.Matrix([s.diff(L,z) for z in qd])
    Lq=s.Matrix([s.diff(L,z) for z in q])
    M6=clean(momentum.jacobian(qd))
    h6=clean(momentum.jacobian(q)*qd-Lq)
    # 第一类: M6 qdd + h6 = B6 tau + Acon.T lambda; Acon qdd + Adot qd=0。
    augmented=M6.row_join(-Acon.T).col_join(Acon.row_join(s.zeros(3)))
    adot_qd=(Acon*qd).jacobian(q)*qd
    rhs=(B6*tau-h6).col_join(-adot_qd)
    allowed=S*u; sub=dict(zip(qd,allowed)); bias=allowed.jacobian(q)*allowed
    M=clean(S.T*M6*S)
    h=clean(S.T*(M6*bias+h6.subs(sub,simultaneous=True)))
    B=clean(S.T*B6)
    # 第二类/无乘子形式: d(S.T p)/dt - S.T Lq - Sdot.T p = S.T Q。
    # qdot=S(q)u 是速度约束，不能把 u 误当三个全局坐标的导数后直接套普通EL。
    p_allowed=momentum.subs(sub,simultaneous=True)
    projected_momentum=clean(S.T*p_allowed)
    Sdot=S.applyfunc(lambda e:sum(s.diff(e,z)*v for z,v in zip(q,allowed)))
    M_second=clean(projected_momentum.jacobian(u))
    h_second=clean(projected_momentum.jacobian(q)*allowed
                   -S.T*Lq.subs(sub,simultaneous=True)-Sdot.T*p_allowed)
    assert clean(Acon*S)==s.zeros(3)
    assert clean(M-M_second)==s.zeros(3)
    assert clean(h-h_second)==s.zeros(3,1)
    return dict(T=T,V=V,L=L,M6=M6,h6=h6,Acon=Acon,S=S,u=u,B6=B6,
                augmented_matrix=augmented,augmented_rhs=rhs,bias=bias,
                M=M,h=h,B=B,M_second=M_second,h_second=h_second,
                momentum=momentum,projected_momentum=projected_momentum,Sdot=Sdot)
