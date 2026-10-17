"""复用现有六维矩阵，核对 world-x 速度消元和静止点线性化；不重建能量或EL。"""
from functools import lru_cache
import sympy as s
from base import q,qd,psi,theta,r,d,clean,constraints
from turning_balance.lagrange import build as turning_build, v,omega,p,wheel_symmetry
from straight_balance.lagrange import build as straight_build,psi_0

def clean(a):
    # world-x 消元含 sec/tan 有理式，保留适用于该表达式的匹配化简；
    # 基础刚体动力学使用 base.clean 的 Fu 多项式三角化简。
    def scalar(e): return s.trigsimp(s.expand(e))
    return a.applyfunc(scalar) if isinstance(a,s.MatrixBase) else scalar(a)

@lru_cache(None)
def build_bridge():
    model=turning_build();static=straight_build()
    dz=s.Matrix([qd[0],qd[2],qd[3]])
    A=constraints()[1]
    solved=s.solve(list(A*qd),[qd[1],qd[4],qd[5]],dict=True)[0]
    allowed=qd.subs(solved,simultaneous=True)
    N=clean(allowed.jacobian(dz))
    Ndot=clean(N.diff(psi)*dz[1]);bias=clean(Ndot*dz)
    assert clean(A*N)==s.zeros(3)
    subs=dict(zip(qd,N*dz))
    Mr=clean(N.T*model['M6']*N)
    hr=clean(N.T*(model['M6']*bias+model['h6'].subs(subs,simultaneous=True)))
    Br=clean(N.T*model['B6'])
    a,c,e,J,Dcoef=model['coefficients'];dx,dpsi,dtheta=dz
    expected_M=s.Matrix([[a/s.cos(psi)**2,0,c*s.cos(theta)/s.cos(psi)],
        [0,J+Dcoef*s.sin(theta)**2,0],[c*s.cos(theta)/s.cos(psi),0,e]])
    expected_h=s.Matrix([
        a*s.tan(psi)/s.cos(psi)**2*dx*dpsi-c*s.sin(theta)/s.cos(psi)*(dtheta**2+dpsi**2),
        (2*Dcoef*s.sin(theta)*s.cos(theta)*dtheta+c*s.sin(theta)/s.cos(psi)*dx)*dpsi,
        c*s.cos(theta)/s.cos(psi)*s.tan(psi)*dx*dpsi-Dcoef*s.sin(theta)*s.cos(theta)*dpsi**2+model['V'].diff(theta)])
    expected_M += model['delta_M']
    expected_h += model['delta_h'].subs({omega:dpsi,p:dtheta})
    assert clean(Mr-expected_M)==s.zeros(3)
    assert clean(hr-expected_h)==s.zeros(3,1)

    # 与现有前向速度投影等价；D只变换三个速度，不定义新的位置坐标。
    D=s.diag(1/s.cos(psi),1,1);Ddot=D.diff(psi)*dz[1]
    assert clean(N-model['S']*D)==s.zeros(6,3)
    hcheck=D.T*(model['M']*Ddot*dz+model['h'].subs({v:dz[0]/s.cos(psi),omega:dz[1],p:dz[2]},simultaneous=True))
    assert clean(hr-hcheck)==s.zeros(3,1)
    rest={theta:0,psi:psi_0,**dict.fromkeys(dz,0)}
    M0=clean(Mr.subs(rest));C0=clean(hr.jacobian(dz).subs(rest))
    K0=clean(hr.jacobian(s.Matrix([q[0],psi,theta])).subs(rest));B0=clean(Br.subs(rest))
    for key,value in [('M',M0),('C',C0),('K',K0),('B',B0)]:
        assert clean(value-static[key].subs(wheel_symmetry))==s.zeros(*value.shape)
    # 整体残差的 epsilon 一阶系数，独立核对文档的逐项舍弃结果。
    eps=s.Symbol('epsilon',real=True)
    delta=s.Matrix(s.symbols('delta_x delta_psi delta_theta',real=True))
    acc=s.Matrix(s.symbols('ddelta_x ddelta_psi ddelta_theta',real=True))
    inputs=s.Matrix(s.symbols('tau_L tau_R',real=True))
    scale={psi:psi_0+eps*delta[1],theta:eps*delta[2],**{z:eps*z for z in dz}}
    residual=Mr.subs(scale,simultaneous=True)*(eps*acc)+hr.subs(scale,simultaneous=True)-Br.subs(scale,simultaneous=True)*(eps*inputs)
    linear=clean(residual.diff(eps).subs(eps,0))
    assert clean(linear-(M0*acc+C0*dz+K0*delta-B0*inputs))==s.zeros(3,1)
    N0=N.subs(psi,psi_0)
    assert clean(N0-static['S'])==s.zeros(6,3)
    assert clean(static['Acon']*N0)==s.zeros(3)
    # 增广方程的加速度约束与代入形式完全一致。
    adot=A.diff(psi)*dz[1]
    assert clean(A*bias+adot*N*dz)==s.zeros(3,1)
    return dict(A=A,N=N,Ndot=Ndot,bias=bias,dz=dz,Mr=Mr,hr=hr,Br=Br,
                M0=M0,C0=C0,K0=K0,B0=B0,N0=N0,static=static,exact=model)
