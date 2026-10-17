"""唯一的三刚体运动学源。R_parent_child 把 child 分量变成 parent 分量。
N 原点位于名义车轴高度平面；地面 z=-r。l 是质心高度，行程使用 s。
所有 q/qd 都是独立符号，时间微分用 Jacobian 链式法则显式实现。
"""
from functools import lru_cache
import sympy as s
q=s.Matrix(s.symbols('x y psi theta theta_L theta_R', real=True))
x,y,psi,theta,theta_L,theta_R=q
# dx 等是内部广义速度符号；笔记本通过 symbol_names 显示为对应坐标的上点形式。
qd=s.Matrix(s.symbols('dx dy dpsi dtheta dtheta_L dtheta_R', real=True))
qdd=s.Matrix(s.symbols('ddx ddy ddpsi ddtheta ddtheta_L ddtheta_R', real=True))
r,d,l,g=s.symbols('r d l g', positive=True)
m_L,m_R,m_B=s.symbols('m_L m_R m_B', positive=True)
I_Lxx,I_Lyy,I_Lzz,I_Rxx,I_Ryy,I_Rzz,I_Bxx,I_Byy,I_Bzz=s.symbols(
    'I_Lxx I_Lyy I_Lzz I_Rxx I_Ryy I_Rzz I_Bxx I_Byy I_Bzz', positive=True)

# 实际惯量矩阵的非对角元素（CAD 正惯性积须先取反）。
I_Bxy,I_Bxz,I_Byz=s.symbols("I_Bxy I_Bxz I_Byz", real=True)

def clean(a):
    # 先展开，再用 Fu 三角恒等变换，避免默认匹配化简对完整惯量大式做高维因式分解。
    def scalar(e): return s.trigsimp(s.expand(e),method='fu')
    return a.applyfunc(scalar) if isinstance(a,s.MatrixBase) else scalar(a)
def ry(a): return s.Matrix([[s.cos(a),0,s.sin(a)],[0,1,0],[-s.sin(a),0,s.cos(a)]])
def rz(a): return s.Matrix([[s.cos(a),-s.sin(a),0],[s.sin(a),s.cos(a),0],[0,0,1]])
def homogeneous(R,p): return R.row_join(p).col_join(s.Matrix([[0,0,0,1]]))
def skew(w): return s.Matrix([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
def dt(a):
    """沿 qdot=qd、qdd 的时间全导数；可用于标量或矩阵。"""
    def scalar(e):
        return sum(s.diff(e,z)*rate for z,rate in zip(q,qd))+sum(s.diff(e,z)*rate for z,rate in zip(qd,qdd))
    return a.applyfunc(scalar) if isinstance(a,s.MatrixBase) else scalar(a)

@lru_cache(None)
def kinematics():
    Rnc=rz(psi)
    edges={('N','C'):(Rnc,s.Matrix([x,y,0])),
           ('C','B'):(ry(theta),ry(theta)*s.Matrix([0,0,l])),
           ('C','WL'):(ry(theta_L),s.Matrix([0,d/2,0])),
           ('C','WR'):(ry(theta_R),s.Matrix([0,-d/2,0]))}
    adjacent={f'{a}_{b}':homogeneous(R,p) for (a,b),(R,p) in edges.items()}
    transforms={'C':adjacent['N_C']}
    for body in ('B','WL','WR'):
        transforms[body]=clean(transforms['C']*adjacent['C_'+body])
    positions={k:T[:3,3] for k,T in transforms.items()}
    rotations={k:T[:3,:3] for k,T in transforms.items()}
    velocities={k:clean(p.jacobian(q)*qd) for k,p in positions.items()}
    omega_hat={k:clean(dt(R)*R.T) for k,R in rotations.items()}
    omegas={k:s.Matrix([W[2,1],W[0,2],W[1,0]]) for k,W in omega_hat.items()}
    relative_omega={f'{a}_{b}':clean(dt(R)*R.T) for (a,b),(R,p) in edges.items()}
    return dict(adjacent=adjacent,T=transforms,R=rotations,p=positions,v=velocities,
                omega=omegas,omega_hat=omega_hat,relative_omega_hat=relative_omega)

@lru_cache(None)
def energies():
    """逐刚体用 N 系 v、omega 计算；惯量也先转到 N 系。"""
    k=kinematics(); parts={}
    bodies={'B':(m_B,s.Matrix([[I_Bxx,I_Bxy,I_Bxz],[I_Bxy,I_Byy,I_Byz],[I_Bxz,I_Byz,I_Bzz]])),
            'WL':(m_L,s.diag(I_Lxx,I_Lyy,I_Lzz)),
            'WR':(m_R,s.diag(I_Rxx,I_Ryy,I_Rzz))}
    for name,(mass,Ib) in bodies.items():
        R=k['R'][name]; v=k['v'][name]; w=k['omega'][name]
        parts[name]=clean((mass*(v.T*v)[0]+(w.T*R*Ib*R.T*w)[0])/2)
    V=clean(sum(mass*g*k['p'][name][2] for name,(mass,_) in bodies.items()))
    return parts,clean(sum(parts.values())),V

@lru_cache(None)
def constraints():
    k=kinematics(); ex=k['R']['C'][:,0]; ey=k['R']['C'][:,1]
    contact={name:clean(k['v'][name]+k['omega'][name].cross(s.Matrix([0,0,-r]))) for name in ('WL','WR')}
    expressions=clean(s.Matrix([(ey.T*k['v']['C'])[0],(ex.T*contact['WL'])[0],(ex.T*contact['WR'])[0]]))
    return expressions,expressions.jacobian(qd),contact
