# -*- coding: utf-8 -*-
"""
生成 sympy_scaffold.ipynb —— 平衡车完整模型 v3 推导 notebook。

内容与 sympy_scaffold.py 的 v3 模型一致:
  六坐标 (x,y,ψ,θ,θ_L,θ_R) + 三非完整约束 -> KanesMethod 约化到 (x,ψ,θ)
代码格顺序执行共享命名空间;Markdown 格说明约定与含义。
生成后请用 .venv kernel 就地执行:
    .venv/bin/python -m jupyter nbconvert --to notebook --execute --inplace sympy_scaffold.ipynb
"""
import nbformat as nbf

nb = nbf.v4.new_notebook()
nb.metadata = {
    "kernelspec": {"display_name": "Python 3 (lagarain .venv)",
                   "language": "python", "name": "python3"},
    "language_info": {"name": "python"},
}

cells = []
md = lambda s: cells.append(nbf.v4.new_markdown_cell(s))
code = lambda s: cells.append(nbf.v4.new_code_cell(s))

# =====================================================================
md(r"""
# 平衡车完整模型 `(x, ψ, θ)` 约化 —— SymPy v3 推导

对象:**左右轮 + 车体**三刚体自平衡车(不计电机转子惯量,轮子纯滚动无滑)。
最终交付**三条约化方程**,供与手写拉格朗日推导逐项对账。

## 六个广义坐标
| 坐标 | 含义 |
|---|---|
| $x, y$ | 车轴中点 $C_o$ 在惯性系 $N$ 的水平位移 |
| $\psi$ | 车轴系 $C$ 相对 $N$ 的**偏航**(绕 $N.z$) |
| $\theta$ | 车体 $B$ 相对 $C$ 的**俯仰**(绕车轴 $C.y$) |
| $\theta_L, \theta_R$ | 左右轮相对 $C$ 的**滚动**(绕 $C.y$) |

系: $N$ 惯性,$C$ 车轴,$L/R$ 轮,$B$ 车体;原点 $C_o$=轴心中点, $L_o=C_o+\tfrac{d}{2}C.y$, $R_o=C_o-\tfrac{d}{2}C.y$, $B_o=C_o+l\,B.z$;接地点 $C_L=C_L$轮心$-r N.z$。

## 参数
$r$ 轮半径, $d$ 轮距, $l=C_o\to B_o$, $g$;质量 $m_L,m_R,m_B$;惯量 $I_{Lxx},I_{Lyy},I_{Lzz}$($I_L$ 系主轴)等;轮直径惯量相等时 $I_{Lxx}=I_{Lzz}$。

## 三个独立非完整约束($6\to 3$)
1. **无侧滑** $\;v(C_o)\cdot C.y=0$(车体侧向,注意是 **$C.y$** 非 $N.y$ → 允许转弯)
2. **左轮纯滚** $\;v(C_L)\cdot C.x=0$
3. **右轮纯滚** $\;v(C_R)\cdot C.x=0$

独立广义坐标取 $(x,\psi,\theta)$;$y,\theta_L,\theta_R$ 由约束决定(约束层为速度约束,非完整)。
**注意**:这里 $x$ 是**世界 $N.x$ 位移**(非车头方向位移/弧长),因此 $\dot y=\dot x\tan\psi$ 被无侧滑绑定,$\cos\psi$ 分母随之出现——这是坐标选取的固有后果,不是建模缺陷。
""")

# =====================================================================
code(r"""
import sympy as sm
import sympy.physics.mechanics as me
from IPython.display import display, Latex
from sympy import init_printing
init_printing()

def disp(name, expr):
    # 以 LaTeX 显示  name = expr
    display(Latex(f"${name} = {sm.latex(expr)}$"))

# ---------- 参数 ----------
r, d, l, g = sm.symbols('r d l g', positive=True)      # 轮半径/轮距/Co->Bo/重力
m_L, m_R, m_B = sm.symbols('m_L m_R m_B', positive=True)
I_Lxx, I_Lyy, I_Lzz = sm.symbols('I_Lxx I_Lyy I_Lzz')   # 左轮(绕自身质心, L 系主轴)
I_Rxx, I_Ryy, I_Rzz = sm.symbols('I_Rxx I_Ryy I_Rzz')   # 右轮
I_Bxx, I_Byy, I_Bzz = sm.symbols('I_Bxx I_Byy I_Bzz')   # 车体

# ---------- 六个广义坐标 ----------
x, y, psi, theta, thL, thR = me.dynamicsymbols('x y psi theta theta_L theta_R')

# ---------- 坐标系与原点 ----------
N = me.ReferenceFrame('N'); C = me.ReferenceFrame('C')
B = me.ReferenceFrame('B'); L = me.ReferenceFrame('L'); R = me.ReferenceFrame('R')
C.orient_axis(N, N.z, psi)      # C 相对 N: 偏航
B.orient_axis(C, C.y, theta)    # B 相对 C: 俯仰
L.orient_axis(C, C.y, thL)      # 左轮相对 C: 滚动
R.orient_axis(C, C.y, thR)      # 右轮

No = me.Point('No'); No.set_vel(N, 0)                   # N 原点(地面, 固定)
Co = me.Point('Co'); Lo = me.Point('Lo'); Ro = me.Point('Ro'); Bo = me.Point('Bo')
Co.set_pos(No, x*N.x + y*N.y + r*N.z)                   # 轴心中点, 离地 r
Co.set_vel(N, x.diff()*N.x + y.diff()*N.y)
Lo.set_pos(Co,  d/2*C.y); Lo.v2pt_theory(Co, N, C)      # 左轮中心 +d/2 C.y
Ro.set_pos(Co, -d/2*C.y); Ro.v2pt_theory(Co, N, C)      # 右轮中心 -d/2 C.y
Bo.set_pos(Co,  l*B.z);   Bo.v2pt_theory(Co, N, B)      # 车体质心 Bo = Co + l·B.z

# 接地点(瞬时接触的"材料点": 速度必须按轮刚体算, 不能当固定点)
CL = me.Point('CL'); CR = me.Point('CR')
CL.set_pos(Lo, -r*N.z)
CR.set_pos(Ro, -r*N.z)
CL.set_vel(N, Lo.vel(N) + L.ang_vel_in(N).cross(CL.pos_from(Lo)))
CR.set_vel(N, Ro.vel(N) + R.ang_vel_in(N).cross(CR.pos_from(Ro)))

# ---------- 刚体 + 惯量 + 能量 ----------
I_Ld = me.inertia(L, I_Lxx, I_Lyy, I_Lzz)
I_Rd = me.inertia(R, I_Rxx, I_Ryy, I_Rzz)
I_Bd = me.inertia(B, I_Bxx, I_Byy, I_Bzz)
wheel_L = me.RigidBody('wheel_L', Lo, L, m_L, (I_Ld, Lo))
wheel_R = me.RigidBody('wheel_R', Ro, R, m_R, (I_Rd, Ro))
body_B  = me.RigidBody('body_B',  Bo, B, m_B, (I_Bd, Bo))

T = wheel_L.kinetic_energy(N) + wheel_R.kinetic_energy(N) + body_B.kinetic_energy(N)
V = m_L*g*r + m_R*g*r + m_B*g*(r + l*sm.cos(theta))     # 轮心高 r, 质心高 r + l cosθ

disp(r'\omega_{B/N}', B.ang_vel_in(N))
disp(r'\omega_{B/N}\,\big|_{N}', B.ang_vel_in(N).express(N))
""")

# =====================================================================
md(r"""
## 1. 三个非完整约束与差速关系

把约束显式求出来。无侧滑给出 $\dot y$ 与 $\dot x$ 的关系;两条纯滚约束解出 $\dot\theta_L,\dot\theta_R$,得到差速驱动关系。下接自检:**直行** $\dot\theta_L=\dot\theta_R=1/r$;**原地转向** $\dot\theta_L=-d/(2r),\ \dot\theta_R=d/(2r)$。
""")

# =====================================================================
code(r"""
lat   = sm.simplify(Co.vel(N).dot(C.y))      # ① 无侧滑: v(Co)·C.y
rollL = sm.simplify(CL.vel(N).dot(C.x))      # ② 左轮纯滚: v(CL)·C.x
rollR = sm.simplify(CR.vel(N).dot(C.x))      # ③ 右轮纯滚: v(CR)·C.x
disp('\\,v(C_o)\\cdot C.y', lat)
disp('\\,v(C_L)\\cdot C.x', rollL)
disp('\\,v(C_R)\\cdot C.x', rollR)
disp('v_f = v(C_o)\\cdot C.x \\;(\\text{沿车头前进})', sm.simplify(Co.vel(N).dot(C.x)))

# 左/右接地点侧向滑移与 ① 相同 -> 三条约束相互独立(无冗余)
print('左右轮接地点侧向速度与无侧滑①之差(应全 0, 佐证独立性):',
      sm.simplify(CL.vel(N).dot(C.y) - lat), ',',
      sm.simplify(CR.vel(N).dot(C.y) - lat))
""")
# =====================================================================
code(r"""
# 用滚动约束解出轮转速 (差速关系)
spin = sm.solve([rollL, rollR], [thL.diff(), thR.diff()])
disp(r'\dot\theta_L', sm.simplify(spin[thL.diff()]))
disp(r'\dot\theta_R', sm.simplify(spin[thR.diff()]))

# 运动学自检
st = {psi: 0, y: 0, theta: 0,
      x.diff(): 1, y.diff(): 0, psi.diff(): 0, theta.diff(): 0}   # 直行
sp = {psi: 0, y: 0, theta: 0,
      x.diff(): 0, y.diff(): 0, psi.diff(): 1, theta.diff(): 0}   # 原地转向
print('直行   ẋ=1  → θ̇_L =', sm.simplify(spin[thL.diff()].subs(st).subs({thL:0, thR:0})),
      '  θ̇_R =', sm.simplify(spin[thR.diff()].subs(st).subs({thL:0, thR:0})),
      '   期望 1/r')
print('原地转 ψ̇=1 → θ̇_L =', sm.simplify(spin[thL.diff()].subs(sp).subs({thL:0, thR:0})),
      '  θ̇_R =', sm.simplify(spin[thR.diff()].subs(sp).subs({thL:0, thR:0})),
      '   期望 -d/(2r), d/(2r)')
""")

# =====================================================================
md(r"""
## 2. KanesMethod:滚动的非完整动力学

滚动约束是**速度层(非完整)**约束,不能直接进 `LagrangesMethod`(它只吃完整约束),改用 **KanesMethod**。

**循环自转技巧**——让车轮模型不出现自转坐标 $\theta_L,\theta_R$,但保留自转能量:
轮是轴对称的(直径惯量 $I_{xx}=I_{zz}$),自转是**循环坐标**,动能里只以 $u_{spin}$ 的平方出现。
做法:轮 frame 取向固定于 $C$(绕 $C.y$ 转 0 角,只为了给取向路径),但**角速度**写为
$$\omega_{wheel}=u_\psi\,N.z+u_{spin}\,C.y.$$
于是自转以**广义速度** $u_L,u_R$ 进能量,不进坐标/位形。

广义速度:独立 $u_{ind}=(\dot x,\dot\psi,\dot\theta)$,依赖 $u_{dep}=(\dot y,\dot\theta_L,\dot\theta_R)$;
三条速度约束(无侧滑 + 两轮纯滚)正好消掉三个依赖速度。$M_r$ 是 6×6 Kane 质量阵消去 $u_{dep}'$ 后的 3×3 约化阵。
""")

# =====================================================================
code(r"""
ux_, up_, ut_ = me.dynamicsymbols('u_x u_psi u_theta')      # 独立: ẋ, ψ̇, θ̇
uy_, ul_, ur_ = me.dynamicsymbols('u_y u_L u_R')            # 依赖: ẏ, θ̇_L, θ̇_R

Cc7 = me.ReferenceFrame('Cc7'); Bc7 = me.ReferenceFrame('Bc7')
Cc7.orient_axis(N, N.z, psi);  Bc7.orient_axis(Cc7, Cc7.y, theta)
Cc7.set_ang_vel(N, up_*N.z)
Bc7.set_ang_vel(N, up_*N.z + ut_*Cc7.y)

# 轮 frame: 取向路径=与 C 差 0 角; 自转只进角速度(循环坐标不进位形)
Wl7 = me.ReferenceFrame('Wl7'); Wr7 = me.ReferenceFrame('Wr7')
Wl7.orient_axis(Cc7, Cc7.y, 0); Wr7.orient_axis(Cc7, Cc7.y, 0)
Wl7.set_ang_vel(N, up_*N.z + ul_*Cc7.y)
Wr7.set_ang_vel(N, up_*N.z + ur_*Cc7.y)

Co7 = me.Point('Co7'); Co7.set_vel(N, ux_*N.x + uy_*N.y)
Lo7 = me.Point('Lo7'); Ro7 = me.Point('Ro7'); Bo7 = me.Point('Bo7')
Lo7.set_pos(Co7,  d/2*Cc7.y); Lo7.v2pt_theory(Co7, N, Cc7)
Ro7.set_pos(Co7, -d/2*Cc7.y); Ro7.v2pt_theory(Co7, N, Cc7)
Bo7.set_pos(Co7,  l*Bc7.z);   Bo7.v2pt_theory(Co7, N, Bc7)
CL7 = me.Point('CL7'); CR7 = me.Point('CR7')
CL7.set_pos(Lo7, -r*N.z)
CR7.set_pos(Ro7, -r*N.z)
CL7.set_vel(N, Lo7.vel(N) + Wl7.ang_vel_in(N).cross(CL7.pos_from(Lo7)))
CR7.set_vel(N, Ro7.vel(N) + Wr7.ang_vel_in(N).cross(CR7.pos_from(Ro7)))

# 轮轴对称: 直径惯量相等 (I_Lxx=I_Lzz), 自转/极轴 = 绕车轴 (I_Lyy)
wL7 = me.RigidBody('wheel_L7', Lo7, Wl7, m_L, (me.inertia(Wl7, I_Lxx, I_Lyy, I_Lxx), Lo7))
wR7 = me.RigidBody('wheel_R7', Ro7, Wr7, m_R, (me.inertia(Wr7, I_Rxx, I_Ryy, I_Rxx), Ro7))
bB7 = me.RigidBody('body_7',   Bo7, Bc7, m_B, (me.inertia(Bc7, I_Bxx, I_Byy, I_Bzz), Bo7))

# 同一组三条约束, 解出依赖速度(应与上节差速关系一致)
lat7   = Co7.vel(N).dot(Cc7.y)
rollL7 = CL7.vel(N).dot(Cc7.x)
rollR7 = CR7.vel(N).dot(Cc7.x)
dep7 = sm.solve([lat7, rollL7, rollR7], [uy_, ul_, ur_], dict=True)[0]
disp('\\dot y\\,(u_y)', sm.simplify(dep7[uy_]))
disp('\\dot\\theta_L\\,(u_L)', sm.simplify(dep7[ul_]))
disp('\\dot\\theta_R\\,(u_R)', sm.simplify(dep7[ur_]))

sub_st = {psi: 0, theta: 0, up_: 0, ut_: 0, ux_: 1}
sub_sp = {psi: 0, theta: 0, ux_: 0, up_: 1, ut_: 0}
print('直行:   u_L =', sm.simplify(dep7[ul_].subs(sub_st)),
      '  u_R =', sm.simplify(dep7[ur_].subs(sub_st)), '   期望 1/r')
print('原地转: u_L =', sm.simplify(dep7[ul_].subs(sub_sp)),
      '  u_R =', sm.simplify(dep7[ur_].subs(sub_sp)), '   期望 -d/(2r), d/(2r)')
""")

# =====================================================================
code(r"""
kd7 = [x.diff()-ux_, psi.diff()-up_, theta.diff()-ut_]
km7 = me.KanesMethod(N, q_ind=[x, psi, theta], u_ind=[ux_, up_, ut_], kd_eqs=kd7,
                     u_dependent=[uy_, ul_, ur_],
                     velocity_constraints=[lat7, rollL7, rollR7],
                     bodies=[wL7, wR7, bB7])
Gv = -g*N.z
km7.kanes_equations(loads=[(Lo7, m_L*Gv), (Ro7, m_R*Gv), (Bo7, m_B*Gv)])   # 仅重力

def reduced(M6, f6):
    # 解加速度约束 u_dep' = P·u_ind' + p, 代回前3行 -> 约化 (M_r, f_r).
    u_ind = [ux_, up_, ut_]; u_dep = [uy_, ul_, ur_]; u_all = u_ind + u_dep
    Mind, Mdep = M6[:3, :], M6[3:, :]
    f_ind, f_dep = f6[:3, :], f6[3:, :]
    acc_ind = sm.Matrix([u_.diff() for u_ in u_ind])
    acc_all = sm.Matrix([u_.diff() for u_ in u_all])
    dep_sym = [u_.diff() for u_ in u_dep]
    sol = sm.solve(Mdep*acc_all - f_dep, dep_sym, dict=True)[0]
    P = sm.Matrix(3, 3, lambda i, j: sol[dep_sym[i]].coeff(acc_ind[j]))
    p = sm.Matrix(3, 1, lambda i, _: sol[dep_sym[i]].subs({a: 0 for a in acc_ind}))
    return (sm.trigsimp(Mind[:, :3] + Mind[:, 3:]*P),
            sm.trigsimp(f_ind - Mind[:, 3:]*p))

Mr, fr_g = reduced(km7.mass_matrix, km7.forcing)

print('Mr 对称:', sm.simplify(Mr - Mr.T) == sm.zeros(3))
M00 = sm.simplify(Mr[0, 0].subs({psi: 0, theta: 0}))
print('直立直线锚 M00 =', M00)
print('锚点差:', sm.simplify(M00 - (m_B + m_L + m_R + (I_Lyy + I_Ryy)/r**2)), '   (应为 0)')
Mr
""")

# =====================================================================
md(r"""
## 3. 约化方程的结构:重力只在 $\theta$ 方向

消去依赖速度后的三方程写成 $M_r(\psi,\theta)\,(\ddot x,\ddot\psi,\ddot\theta)^T = f_r$。
把约束解(如 $\dot y=\dot x\tan\psi$)代进 $f_r$、$u_*\to$ 真实导数,得三条**显式方程**:
每条都整理成 `表达式 = 0`,可直接与手稿逐项对照。全式同存于 `reduced_eom_v3.txt`。

**结构要点** 供对照:重力项只应出现在 $\theta$ 方程($-m_B g l\sin\theta$ 那类);$x,\psi$ 方程只有速度耦合
(陀螺 $(\cdots)\dot\psi\dot\theta\sin2\theta$、离心项等)。$\psi$ 是循环坐标。
""")

# =====================================================================
code(r"""
# 依赖速度解 + 广义速度 -> 真实坐标导数
depS = {uy_: sm.simplify(dep7[uy_]), ul_: sm.simplify(dep7[ul_]), ur_: sm.simplify(dep7[ur_])}
D1v  = {ux_: x.diff(), up_: psi.diff(), ut_: theta.diff()}
MrD = sm.Matrix(3, 3, lambda i, j: Mr[i, j].subs(depS).subs(D1v))
frD = sm.Matrix(3, 1, lambda i, _: fr_g[i].subs(depS).subs(D1v))
aD  = sm.Matrix([x.diff().diff(), psi.diff().diff(), theta.diff().diff()])

# 静止时右端应 = −∂V/∂q = (0, 0, +m_B g l sinθ)
rest = {ux_: 0, up_: 0, ut_: 0, uy_: 0, ul_: 0, ur_: 0}
fr_rest = sm.trigsimp(fr_g.subs(rest))
print('静止(u=0) fr_g =', fr_rest.T, '   期望 (0,0,+m_B g l sinθ);  差:',
      sm.trigsimp(fr_rest - sm.Matrix([0, 0, m_B*g*l*sm.sin(theta)])).T)

eqs3 = []
labs = [r'x\text{-方程}', r'\psi\text{-方程}', r'\theta\text{-方程}']
for i in range(3):
    e = sm.expand(sum(MrD[i, j]*aD[j] for j in range(3)) - frD[i])
    eqs3.append(e)
    disp(labs[i], sm.Eq(e, 0))
print('含 g 的方程:', ['x','psi','theta'][[e.has(g) for e in eqs3].index(True)],
      ' (重力项应只在 θ 方程)')
""")

# =====================================================================
md(r"""
## 4. 电机扭矩 $\to$ 三个方向的广义力

轮毂电机对**轮对**施加扭矩:对左/右轮各 $\tau_L,\tau_R$(绕 $C.y$),反作用等大反向作用到车体。
因自转坐标已被约化,不能直接在 $\theta_L$ 上加载;改用**力偶**:$+\tau$ 在轮 frame、$-\tau$ 在车体上(绕同一轴),广义力自动落在正确的独立坐标上。
""")

# =====================================================================
code(r"""
tau_L, tau_R = sm.symbols('tau_L tau_R')
grav_loads = [(Lo7, m_L*Gv), (Ro7, m_R*Gv), (Bo7, m_B*Gv)]
mot_loads  = [(Wl7, tau_L*Cc7.y), (Wr7, tau_R*Cc7.y), (Bc7, -(tau_L + tau_R)*Cc7.y)]

km7m = me.KanesMethod(N, q_ind=[x, psi, theta], u_ind=[ux_, up_, ut_], kd_eqs=kd7,
                      u_dependent=[uy_, ul_, ur_],
                      velocity_constraints=[lat7, rollL7, rollR7],
                      bodies=[wL7, wR7, bB7])
km7m.kanes_equations(loads=grav_loads + mot_loads)
Mr_m, fr_m = reduced(km7m.mass_matrix, km7m.forcing)
print('Mr 不随电机载荷变:', Mr_m == Mr)

Q = sm.trigsimp(fr_m - fr_g)
# Q 应线性于 (τ_L, τ_R)
Bq = sm.Matrix(3, 2, lambda i, j: sm.trigsimp(Q[i].coeff(tau_L if j == 0 else tau_R)))
print('含 τ 的平方/高次项残余:', any(sm.degree(Q[i], tau_L) > 1 or sm.degree(Q[i], tau_R) > 1
                                      for i in range(3)))
for i, nm in enumerate(['x', 'psi', 'theta']):
    disp('Q_{' + nm + '}', sm.trigsimp(Bq[i, 0])*tau_L + sm.trigsimp(Bq[i, 1])*tau_R)
print('说明: 电机是内力, 作用在被约化掉的轮自转坐标上; 在世界 N.x 为广义坐标时前进不出显式 Q_x,',
      '由 θ 耦合间接产生。若改用弧长 s 为坐标, Q_s=(τ_L+τ_R)/r 会显式出现。')
""")

# =====================================================================
md(r"""
## 5. 收尾检查与后续

- **运动学自检**:直行 $\dot\theta_L=\dot\theta_R=1/r$;原地转向 $\pm d/(2r)$ ✓
- **约化质量阵**实对称 ✓;**直立直线锚点** $M_{00}=\frac{I_{Lyy}+I_{Ryy}}{r^2}+m_B+m_L+m_R$ 精确回归 ✓
- **独立数值对账**(`check_full_multi.py`):全六坐标拉格朗日 + d'Alembert 乘子 vs 本约化,
  重力 ~1e-14、加电机 ~2.75e-14,含左右不对称轮 ✓;电机映射另见 `test_motor.py`
- 与手稿对账时注意**符号约定**:$x$=世界 $N.x$;$\theta$ 正方向 = 绕 $C.y$ 右手(由 $B.z$ 相对 $C$ 转 $\theta$ 确定);Kane 的 $f_r$ 含 $-\partial V/\partial q$ 那类记号,左右轮不对称时 $M_{01}\ne0$。
""")

# =====================================================================
code(r"""
# 左右轮相同的特例(通常机器), 看 Mr 塌缩成多干净
m, Ix, Iy = sm.symbols('m Ix Iy')
Ibx, Iby, Ibz = sm.symbols('Ibx Iby Ibz')
Msym = sm.simplify(Mr.subs({m_L: m, m_R: m, I_Lxx: Ix, I_Rxx: Ix,
                            I_Lyy: Iy, I_Ryy: Iy,
                            I_Bxx: Ibx, I_Byy: Iby, I_Bzz: Ibz}))
Msym
""")

nb["cells"] = cells

import pathlib
out = pathlib.Path(__file__).resolve().parent / "sympy_scaffold.ipynb"
out.write_text(nbf.writes(nb), encoding="utf-8")
print("wrote", out)
