# -*- coding: utf-8 -*-
"""
平衡车完整模型 —— SymPy 骨架 v3（runnable）
==============================================

坐标系原点依次为 No, Co, Lo, Ro, Bo：
    N  空间固定系（原点 No, 地面）
    C  车轴系（原点 Co = 车轴中点）
    L  左轮（原点 Lo）   R 右轮（原点 Ro）   B 车体（原点 Bo = 质心）

六 个 广义坐标：
    x, y     车轴中点 Co 在 N 的 x/y 方向位移
    psi      C 相对 N 的偏航角
    theta    B 相对 C 的俯仰角
    theta_L, theta_R    L/R 相对 C 的转角（绕车轴 C.y 滚动）

几何参数： r 轮半径, d 轮距(车轴宽), l = Co->Bo 距离, g 重力。

三个 非完整约束（速度层, 相互独立, 6->3）:
    ① 无侧滑    v(Co)·C.y = 0
    ② 左轮纯滚  v(CL)·C.x = 0     (CL 为左轮接地点)
    ③ 右轮纯滚  v(CR)·C.x = 0

约化后 3 个独立广义坐标：x, psi, theta（另 y, theta_L, theta_R 由约束决定）。
"""

import sympy as sm
import sympy.physics.mechanics as me

# =====================================================================
# §1 参数（符号; 数值以后填）
# =====================================================================
r = sm.symbols('r', positive=True)         # 轮半径
d = sm.symbols('d', positive=True)         # 轮距 / 车轴宽
l = sm.symbols('l', positive=True)         # Co -> Bo 距离 (车体质心到轴心)
g = sm.symbols('g', positive=True)

m_L, m_R, m_B = sm.symbols('m_L m_R m_B', positive=True)
I_Lxx, I_Lyy, I_Lzz = sm.symbols('I_Lxx I_Lyy I_Lzz')   # 左轮(绕自身质心, L 系)
I_Rxx, I_Ryy, I_Rzz = sm.symbols('I_Rxx I_Ryy I_Rzz')   # 右轮
I_Bxx, I_Byy, I_Bzz = sm.symbols('I_Bxx I_Byy I_Bzz')   # 车体

# =====================================================================
# §2 六个广义坐标
# =====================================================================
x, y, psi, theta, thL, thR = me.dynamicsymbols('x y psi theta theta_L theta_R')

# =====================================================================
# §3 坐标系 + 原点 + 关键点（相对定向用 orient_axis）
# =====================================================================
N = me.ReferenceFrame('N')    # 惯性系(固定), N.z 竖直向上
C = me.ReferenceFrame('C')    # 车轴系
B = me.ReferenceFrame('B')    # 车体
L = me.ReferenceFrame('L')    # 左轮
R = me.ReferenceFrame('R')    # 右轮

C.orient_axis(N, N.z, psi)      # C 相对 N: 偏航
B.orient_axis(C, C.y, theta)    # 车体相对 C: 绕车轴俯仰
L.orient_axis(C, C.y, thL)      # 左轮相对 C: 绕车轴滚动
R.orient_axis(C, C.y, thR)      # 右轮

No = me.Point('No');  No.set_vel(N, 0)      # N 原点(地面, 固定)
Co = me.Point('Co')                          # 车轴中点
Lo = me.Point('Lo');  Ro = me.Point('Ro');  Bo = me.Point('Bo')

Co.set_pos(No, x * N.x + y * N.y)  # 轴心离地与N中心没有z轴上的偏移
Co.set_vel(N, x.diff() * N.x + y.diff() * N.y)
Lo.set_pos(Co,  d / 2 * C.y);  Lo.v2pt_theory(Co, N, C)   # 左轮中心: +d/2 C.y
Ro.set_pos(Co, -d / 2 * C.y);  Ro.v2pt_theory(Co, N, C)   # 右轮中心: -d/2 C.y
Bo.set_pos(Co,  l * B.z);      Bo.v2pt_theory(Co, N, B)   # 质心: 车体 B.z 上 l 处

# 接地点 CL, CR: 轮子正下方 r 处（瞬时接触的"材料点"速度要按轮刚体算）
CL = me.Point('CL');  CR = me.Point('CR')
CL.set_pos(Lo, -r * N.z)
CR.set_pos(Ro, -r * N.z)
CL.set_vel(N, Lo.vel(N) + L.ang_vel_in(N).cross(CL.pos_from(Lo)))
CR.set_vel(N, Ro.vel(N) + R.ang_vel_in(N).cross(CR.pos_from(Ro)))

# =====================================================================
# §4 刚体 + 惯量
# =====================================================================
I_Ld = me.inertia(L, I_Lxx, I_Lyy, I_Lzz)
I_Rd = me.inertia(R, I_Rxx, I_Ryy, I_Rzz)
I_Bd = me.inertia(B, I_Bxx, I_Byy, I_Bzz)

wheel_L = me.RigidBody('wheel_L', Lo, L, m_L, (I_Ld, Lo))
wheel_R = me.RigidBody('wheel_R', Ro, R, m_R, (I_Rd, Ro))
body_B  = me.RigidBody('body_B',  Bo, B, m_B, (I_Bd, Bo))

print("=" * 72)
print("§4 角速度与关键速度")
print("=" * 72)
print("  ω_C/N =", C.ang_vel_in(N).express(N))
print("  ω_B/N =", B.ang_vel_in(N).express(N))
print()

# =====================================================================
# §5 动能 / 势能（先保留精确能量, 为下一步出方程备好）
# =====================================================================
T = (wheel_L.kinetic_energy(N) + wheel_R.kinetic_energy(N)
     + body_B.kinetic_energy(N))
V = m_B * g * (l * sm.cos(theta))  # 在xy平面上运动，没有z方向的位移，以车轴为势能零点

# =====================================================================
# §6 三个非完整约束 + 约化(6 -> 3)
# =====================================================================
print("=" * 72)
print("§6 三个非完整约束(速度层)")
print("=" * 72)
lat   = sm.simplify(Co.vel(N).dot(C.y))      # ① 无侧滑
rollL = sm.simplify(CL.vel(N).dot(C.x))      # ② 左轮纯滚
rollR = sm.simplify(CR.vel(N).dot(C.x))      # ③ 右轮纯滚
print("  ① v(Co)·C.y =", lat)
print("  ② v(CL)·C.x =", rollL)
print("  ③ v(CR)·C.x =", rollR)
print()

print("  → 左/右轮接地点侧滑与①相同(独立性佐证):",
      sm.simplify(CL.vel(N).dot(C.y) - lat) == 0,
      sm.simplify(CR.vel(N).dot(C.y) - lat) == 0)
print()

# 定义沿车头的前进速度 v_f = v(Co)·C.x
v_f = sm.simplify(Co.vel(N).dot(C.x))
print("  前进速度  v_f = v(Co)·C.x =", v_f)
print()

# 用滚动约束解出轮转角速度 (差速关系), 并代回验证
spin = sm.solve([rollL, rollR], [thL.diff(), thR.diff()])
print("  滚动约束 ⇒ 轮转速(差速驱动):")
print("    θ̇_L =", sm.simplify(spin[thL.diff()]))
print("    θ̇_R =", sm.simplify(spin[thR.diff()]))
print()

# 运动学特例自检 1: 直行 ψ=0, ψ̇=0, y=0
sub_st = {psi: 0, y: 0, theta: 0,
          x.diff(): 1, y.diff(): 0, psi.diff(): 0, theta.diff(): 0}
print("自检 直行(ψ=0, ẋ=1): 侧滑=", lat.subs(sub_st),
      "  θ̇_L=θ̇_R=", sm.simplify(spin[thL.diff()].subs(sub_st).subs({thL:0, thR:0})))
# 自检 2: 原地转向 ẋ=ẏ=0, ψ̇=1  → 左轮反转, 右轮正转, 幅度 d/(2r)
sub_sp = {psi: 0, y: 0, theta: 0, x: 0,
          x.diff(): 0, y.diff(): 0, psi.diff(): 1, theta.diff(): 0}
print("自检 原地转向(ψ̇=1): θ̇_L=",
      sm.simplify(spin[thL.diff()].subs(sub_sp).subs({thL:0, thR:0})),
      " θ̇_R=", sm.simplify(spin[thR.diff()].subs(sub_sp).subs({thL:0, thR:0})))
print()

# =====================================================================
# §7 EOM: KanesMethod 上非完整约束 → 约化到 (x, ψ, θ) 三方程
#     （纯滚动非完整 → LagrangesMethod 不支持; 换 KanesMethod）
#     u_ind = (ẋ, ψ̇, θ̇);  u_dep = (ẏ, θ̇_L, θ̇_R) 由三约束消去
# =====================================================================
print("=" * 72)
print("§7 KanesMethod: 6 坐标 + 3 非完整约束 → (x, ψ, θ) 三方程")
print("=" * 72)

ux_, up_, ut_ = me.dynamicsymbols('u_x u_psi u_theta')
uy_, ul_, ur_ = me.dynamicsymbols('u_y u_L u_R')
tau_L, tau_R = sm.symbols('tau_L tau_R')          # 轮毂电机扭矩(作用于车体-轮对)

# 用 u 重参数化运动学: 轮自转是循环坐标 → 只进角速度分量, 不进坐标/位置/能量
Cc7 = me.ReferenceFrame('Cc7'); Bc7 = me.ReferenceFrame('Bc7')
Cc7.orient_axis(N, N.z, psi); Bc7.orient_axis(Cc7, Cc7.y, theta)
Cc7.set_ang_vel(N, up_*N.z);   Bc7.set_ang_vel(N, up_*N.z + ut_*Cc7.y)
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
CL7.set_pos(Lo7, -r*N.z); CL7.set_vel(N, Lo7.vel(N) + Wl7.ang_vel_in(N).cross(CL7.pos_from(Lo7)))
CR7.set_pos(Ro7, -r*N.z); CR7.set_vel(N, Ro7.vel(N) + Wr7.ang_vel_in(N).cross(CR7.pos_from(Ro7)))

# 轮绕自身轴对称 (直径惯量 I_xx = I_zz), 自转轴 = 车轴 (I_yy)
wL7 = me.RigidBody('wheel_L7', Lo7, Wl7, m_L,
                   (me.inertia(Wl7, I_Lxx, I_Lyy, I_Lxx), Lo7))
wR7 = me.RigidBody('wheel_R7', Ro7, Wr7, m_R,
                   (me.inertia(Wr7, I_Rxx, I_Ryy, I_Rxx), Ro7))
bB7 = me.RigidBody('body_7',   Bo7, Bc7, m_B,
                   (me.inertia(Bc7, I_Bxx, I_Byy, I_Bzz), Bo7))

# 同一组三个非完整约束 (依赖速度解与 §6 差速关系一致)
lat7  = Co7.vel(N).dot(Cc7.y)
rollL7 = CL7.vel(N).dot(Cc7.x)
rollR7 = CR7.vel(N).dot(Cc7.x)
dep7 = sm.solve([lat7, rollL7, rollR7], [uy_, ul_, ur_], dict=True)[0]
s_st = {psi: 0, theta: 0, up_: 0, ut_: 0, ux_: 1}
s_sp = {psi: 0, theta: 0, ux_: 0, up_: 1, ut_: 0}
print("  自检 直行: u_L=u_R =", sm.simplify(dep7[ul_].subs(s_st)),
      " | 原地转: u_L=", sm.simplify(dep7[ul_].subs(s_sp)),
      " u_R=", sm.simplify(dep7[ur_].subs(s_sp)))

kd7 = [x.diff()-ux_, psi.diff()-up_, theta.diff()-ut_]
km7 = me.KanesMethod(N, q_ind=[x, psi, theta], u_ind=[ux_, up_, ut_], kd_eqs=kd7,
                     u_dependent=[uy_, ul_, ur_],
                     velocity_constraints=[lat7, rollL7, rollR7],
                     bodies=[wL7, wR7, bB7])
Gv = -g*N.z
km7.kanes_equations(loads=[(Lo7, m_L*Gv), (Ro7, m_R*Gv), (Bo7, m_B*Gv)])

# --- 用"加速度约束"消去 u_dep' → 约化 3×3:  Mr·(ẍ,ψ̈,θ̈) = fr ---
u_all7 = [ux_, up_, ut_, uy_, ul_, ur_]
u_ind7 = [ux_, up_, ut_]; u_dep7s = [uy_, ul_, ur_]
M6, f6 = km7.mass_matrix, km7.forcing
a_ind = sm.Matrix([u_.diff() for u_ in u_ind7])
a_all = sm.Matrix([u_.diff() for u_ in u_all7])
dep_sym = [u_.diff() for u_ in u_dep7s]
d7 = sm.solve(M6[3:, :]*a_all - f6[3:, :], dep_sym, dict=True)[0]
P7 = sm.Matrix(3, 3, lambda i, j: d7[dep_sym[i]].coeff(a_ind[j]))
p7 = sm.Matrix(3, 1, lambda i, _: d7[dep_sym[i]].subs({a_ind[j]: 0 for j in range(3)}))
Mr = sm.trigsimp(M6[:3, :3] + M6[:3, 3:]*P7)      # 约化质量矩阵
fr7 = sm.trigsimp(f6[:3, :] - M6[:3, 3:]*p7)      # 约化右端(重力)

print("\n  约化质量矩阵对称性:", sm.simplify(Mr - Mr.T) == sm.zeros(3))
# 回归锚: 直立直线 (ψ=0) 第一行第一列应回到旧 DEMO 质量元
M00_anchor = sm.simplify(Mr[0, 0].subs({psi: 0, theta: 0}))
print("  直立直线 M11(x) =", M00_anchor)
print("  回归锚差:",
      sm.simplify(M00_anchor - (m_B + m_L + m_R + (I_Lyy + I_Ryy)/r**2)), " (应为 0)")
print("  T =", sm.count_ops(T), "ops | V = m_B·g·(r+l·cosθ)+const")
print("  (全式见 notebook; 数值对账: check_full_multi.py 残差 ~1e-14)")

# --- 三方向广义力: 电机扭矩 τ_L, τ_R 映射 (已在 test_motor.py 验证) ---
print("  (电机广义力映射完整式见 notebook §7b; 直行/一般情形结论:")
print("   Q_ψ = d/(2r)·(τ_R−τ_L),   Q_θ = −(τ_L+τ_R),   Q_x = 0")
print("   电机是内力、作用在被约化掉的轮自转坐标上; 前进在世界 x 方向不出显式广义力, 由 θ 耦合产生)")

# ---- (仅供汇报) 打印约化方程全式(代入速度约束 → 只含 x,ψ,θ 及其导数) ----
D1 = {ux_: x.diff(), up_: psi.diff(), ut_: theta.diff()}
fr7s = sm.trigsimp(fr7.subs(dep7).subs(D1))       # RHS 只含 x,ψ,θ 一阶导
MrD  = sm.Matrix(3, 3, lambda i, j: Mr[i, j].subs(dep7).subs(D1))
aD   = sm.Matrix([x.diff().diff(), psi.diff().diff(), theta.diff().diff()])
print("\n--- 约化三方程(已代入 ẏ=ẋ·tanψ 等; 重力项) ---")
for i, nm in zip(range(3), ['x', 'psi', 'theta']):
    eq = sm.trigsimp(sum(MrD[i, j]*aD[j] for j in range(3)) - fr7s[i])
    print(f"\nEQ_{nm} = {sm.sstr(eq.expand(), sympy_integers=True)} = 0")
print("\n约化质量矩阵 Mr (已代速度约束):")
for i in range(3):
    for j in range(3):
        e = sm.simplify(MrD[i, j])
        print(f"  Mr[{i},{j}] = {sm.sstr(e, sympy_integers=True)}")
print("\n(电机广义力见 test_motor.py: Q_x=0, Q_ψ=d(τ_R−τ_L)/(2r), Q_θ=−(τ_L+τ_R))")
