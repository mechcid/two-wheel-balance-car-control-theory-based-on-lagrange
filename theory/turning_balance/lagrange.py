"""任意 psi 的六坐标精确动力学及小俯仰近似，轮子轴对称且左右相同。
底层运动学只从 base 取得；先正确求导再近似，保留 theta*omega**2。
"""
from functools import lru_cache
import sympy as s
from base import *
from lagrange_utils import derive
mW,Js,Jt=s.symbols('m_W J_s J_t',positive=True)
v,omega,p=s.symbols('v omega p',real=True)
u=s.Matrix([v,omega,p])
wheel_symmetry={m_L:mW,m_R:mW,I_Lxx:Jt,I_Lzz:Jt,I_Rxx:Jt,I_Rzz:Jt,I_Lyy:Js,I_Ryy:Js}

@lru_cache(None)
def build():
    _,T,V=energies(); T=clean(T.subs(wheel_symmetry))
    S=s.Matrix([[s.cos(psi),0,0],[s.sin(psi),0,0],[0,1,0],[0,0,1],
                [1/r,-d/(2*r),0],[1/r,d/(2*r),0]])
    model=derive(T,V,constraints()[1],S,u)
    # theta、p 同阶小量；v、omega 不按小量处理。
    eps=s.Symbol('epsilon')
    def first(e):return s.expand(e.subs({theta:eps*theta,p:eps*p},simultaneous=True)).series(eps,0,2).removeO().subs(eps,1)
    Msmall=model['M'].applyfunc(first); hsmall=model['h'].applyfunc(first)
    a=m_B+2*mW+2*Js/r**2; c=m_B*l; e=I_Byy+m_B*l**2
    J=I_Bzz+2*Jt+d**2*mW/2+d**2*Js/(2*r**2)
    D=I_Bxx-I_Bzz+m_B*l**2
    expected=s.Matrix([[a,0,c*s.cos(theta)],[0,J+D*s.sin(theta)**2,0],[c*s.cos(theta),0,e]])
    assert clean(model['M']-expected)==s.zeros(3)
    expected_h=s.Matrix([-c*s.sin(theta)*(p**2+omega**2),
        (2*D*s.sin(theta)*s.cos(theta)*p+c*s.sin(theta)*v)*omega,
        -D*s.sin(theta)*s.cos(theta)*omega**2-c*g*s.sin(theta)])
    assert clean(model['h']-expected_h)==s.zeros(3,1)
    # 能量功率恒等式独立于控制器，防止漏掉转向耦合项。
    Tr=clean(T.subs(dict(zip(qd,S*u)),simultaneous=True)); E=Tr+V
    ud=s.Matrix(s.symbols('dv domega dp'))
    Edot=s.diff(E,theta)*p+(s.Matrix([s.diff(E,z) for z in u]).T*ud)[0]
    assert clean(Edot-(u.T*(model['M']*ud+model['h']))[0])==0
    model.update(T_reduced=Tr,M_small=Msmall,h_small=hsmall,
                 coefficients=s.Matrix([a,c,e,J,D]))
    return model
if __name__=='__main__':
    for key in ('M','h','B','M_small','h_small'): print(key,'=',build()[key])
