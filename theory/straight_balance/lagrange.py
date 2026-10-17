"""六坐标二阶能量 -> 第一类/无乘子第二类 -> 三变量局部矩阵。
保留旧仿真世界 x 约定；名义朝向 psi_0 任意，但 cos(psi_0) 不可为0。
"""
from functools import lru_cache
import sympy as s
from base import *
from lagrange_utils import derive
psi_0=s.Symbol('psi_0',real=True)

@lru_cache(None)
def build():
    _,T_exact,V_exact=energies()
    rest={psi:psi_0,theta:0,theta_L:0,theta_R:0}
    M_at_rest=clean(s.hessian(T_exact,qd).subs(rest))
    T2=clean((qd.T*M_at_rest*qd)[0]/2)
    V2=m_B*g*l*(1-theta**2/2)
    # 局部一阶模型应使用在同一参考点冻结的约束；非线性约束仍在 base 可查。
    A0=constraints()[1].subs(rest)
    c=s.cos(psi_0)
    S=s.Matrix([[1,0,0],[s.tan(psi_0),0,0],[0,1,0],[0,0,1],
                [1/(r*c),-d/(2*r),0],[1/(r*c),d/(2*r),0]])
    u=s.Matrix(s.symbols('u_x u_psi u_theta',real=True))
    model=derive(T2,V2,A0,S,u)
    stiffness=s.diag(0,0,-m_B*g*l)
    assert clean(model['h']-stiffness*s.Matrix([x,psi,theta]))==s.zeros(3,1)
    model.update(T_exact=T_exact,V_exact=V_exact,C=s.zeros(3),K=stiffness,
                 z=s.Matrix([x,psi,theta]))
    return model

# 保留已有数值仿真校验需要的最终符号矩阵名字。
def __getattr__(name):
    keys={'M_linear':'M','C_linear':'C','K_linear':'K','B_linear':'B'}
    if name in keys:return build()[keys[name]]
    raise AttributeError(name)
if __name__=='__main__':
    for key in ('M','C','K','B'): print(key,'=',build()[key])
