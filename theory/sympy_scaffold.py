# -*- coding: utf-8 -*-
"""
平衡车完整模型 —— SymPy 拉格朗日推导
==============================================

坐标系原点依次为 No, Co, Lo, Ro, Bo：
    N  空间固定系（原点 No, 车轴高度参考平面）
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

保留六个位置坐标，以 x、psi、theta 的导数参数化三个独立速度。
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

No = me.Point('No');  No.set_vel(N, 0)      # N 原点(车轴高度参考平面, 固定)
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
# §3a 相邻坐标系的旋转矩阵与齐次变换矩阵
# =====================================================================
# 约定 ^P R_F 把 F 系列分量变为 P 系列分量：v_P = R_PF*v_F。
# ^P H_F = [R_PF, p_PF; 0,1]，p_PF 是从 Po 指向 Fo、用 P 基表示的位矢。
# 在 SymPy 中 R_PF 对应 P.dcm(F)，不是 F.dcm(P)。
# H 用来表示齐次变换，避免覆盖动能变量 T。
t = me.dynamicsymbols._t

def matrix_simplify(mat):
    return mat.applyfunc(sm.trigsimp)

def skew(v):
    # [v]_cross*w = v cross w
    return sm.Matrix([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def unskew(mat):
    return sm.Matrix([mat[2, 1], mat[0, 2], mat[1, 0]])

def homogeneous(rot, pos):
    return rot.row_join(pos).col_join(sm.Matrix([[0, 0, 0, 1]]))

# 显式逐元素写出矩阵，独立于 ReferenceFrame 的自动定向结果。
R_NC = sm.Matrix([[sm.cos(psi), -sm.sin(psi), 0],
                  [sm.sin(psi), sm.cos(psi), 0], [0, 0, 1]])
R_CL = sm.Matrix([[sm.cos(thL), 0, sm.sin(thL)], [0, 1, 0],
                  [-sm.sin(thL), 0, sm.cos(thL)]])
R_CR = sm.Matrix([[sm.cos(thR), 0, sm.sin(thR)], [0, 1, 0],
                  [-sm.sin(thR), 0, sm.cos(thR)]])
R_CB = sm.Matrix([[sm.cos(theta), 0, sm.sin(theta)], [0, 1, 0],
                  [-sm.sin(theta), 0, sm.cos(theta)]])
p_NC = sm.Matrix([x, y, 0])
p_CL = sm.Matrix([0, d/2, 0])
p_CR = sm.Matrix([0, -d/2, 0])
# B 原点是质心 Bo，不是轴心 Co；这一平移在 C 中随 theta 变化。
p_CB = matrix_simplify(R_CB * sm.Matrix([0, 0, l]))
H_NC = homogeneous(R_NC, p_NC)
H_CL = homogeneous(R_CL, p_CL)
H_CR = homogeneous(R_CR, p_CR)
H_CB = homogeneous(R_CB, p_CB)
edges = [('N', 'C', N, C, R_NC, H_NC),
         ('C', 'L', C, L, R_CL, H_CL),
         ('C', 'R', C, R, R_CR, H_CR),
         ('C', 'B', C, B, R_CB, H_CB)]

# =====================================================================
# §3b 相邻系相对角速度：旋转矩阵求导
# =====================================================================
# ^P Omega_(F/P) = dot(R_PF)*R_PF.T；^F Omega_(F/P) = R_PF.T*dot(R_PF)。
# 第一种用父系基表示，第二种用子系基表示；都是相对角速度。
relative_angular = {}
for parent, child, Pf, Ff, rot, transform in edges:
    Omega_parent = matrix_simplify(rot.diff(t) * rot.T)
    Omega_child = matrix_simplify(rot.T * rot.diff(t))
    relative_angular[(parent, child)] = {
        'Omega_parent': Omega_parent, 'omega_parent': unskew(Omega_parent),
        'Omega_child': Omega_child, 'omega_child': unskew(Omega_child)}

# =====================================================================
# §3c 变换链与各刚体相对于 N 的角速度矩阵
# =====================================================================
R_NL = matrix_simplify(R_NC * R_CL)
R_NR = matrix_simplify(R_NC * R_CR)
R_NB = matrix_simplify(R_NC * R_CB)
H_NL = matrix_simplify(H_NC * H_CL)
H_NR = matrix_simplify(H_NC * H_CR)
H_NB = matrix_simplify(H_NC * H_CB)
rotations_N = {'C': R_NC, 'L': R_NL, 'R': R_NR, 'B': R_NB}
transforms_N = {'C': H_NC, 'L': H_NL, 'R': H_NR, 'B': H_NB}
Omega_N = {name: matrix_simplify(rot.diff(t)*rot.T)
           for name, rot in rotations_N.items()}
omega_N = {name: unskew(mat) for name, mat in Omega_N.items()}
# 用角速度递推公式独立核对 dot(R)*R.T 的结果。
omega_recursive_N = {'C': relative_angular[('N', 'C')]['omega_parent']}
for name in ['L', 'R', 'B']:
    omega_recursive_N[name] = matrix_simplify(
        omega_N['C'] + R_NC*relative_angular[('C', name)]['omega_parent'])

# =====================================================================
# §3d 各原点/质心在 N 下的位置、速度列矩阵与速度雅可比
# =====================================================================
q_kin = sm.Matrix([x, y, psi, theta, thL, thR])
origin_points = {'C': Co, 'L': Lo, 'R': Ro, 'B': Bo}
positions_N = {name: trans[:3, 3] for name, trans in transforms_N.items()}
# 路径一：齐次变换取出的 N 系位置列矩阵，对时间直接求导。
velocities_N = {name: matrix_simplify(pos.diff(t)) for name, pos in positions_N.items()}
# 路径二：v_F = v_C + R_NC*(Omega_C/N^C*p_CF + dot(p_CF))。
# dot(p_CF) 是 C 中分量的导数，Bo 的这一项不可省略。
Omega_C_in_C = relative_angular[('N', 'C')]['Omega_child']
velocities_recursive_N = {'C': p_NC.diff(t)}
for name, p_CF in [('L', p_CL), ('R', p_CR), ('B', p_CB)]:
    velocities_recursive_N[name] = matrix_simplify(
        velocities_N['C'] + R_NC*(Omega_C_in_C*p_CF + p_CF.diff(t)))
# 同时给出 v = J_v*qdot、omega = J_w*qdot，所有分量均在 N 下。
Jv_N = {name: pos.jacobian(q_kin) for name, pos in positions_N.items()}
Jw_N = {name: w.jacobian(q_kin.diff(t)) for name, w in omega_N.items()}

# =====================================================================
# §3e 接触材料点速度：用角速度反对称矩阵作叉乘
# =====================================================================
# 当前接触位置对应的轮上材料点，每一瞬间可能不同。
# 不对“圆心下方最低位置”的轨迹求导来替代材料点速度。
contact_offset_N = sm.Matrix([0, 0, -r])
v_CL_matrix = matrix_simplify(velocities_N['L'] + Omega_N['L']*contact_offset_N)
v_CR_matrix = matrix_simplify(velocities_N['R'] + Omega_N['R']*contact_offset_N)

# =====================================================================
# §3f 矩阵展开与 Mechanics 对象法的逐项等价检查
# =====================================================================
kinematic_checks = {}
def check_matrix(name, actual, expected):
    residual = (actual-expected).applyfunc(sm.simplify)
    kinematic_checks[name] = residual == sm.zeros(*residual.shape)

for parent, child, Pf, Ff, rot, transform in edges:
    check_matrix('R_'+parent+child, rot, Pf.dcm(Ff))
    check_matrix('orthogonal_'+parent+child, rot.T*rot, sm.eye(3))
    check_matrix('omega_relative_'+parent+child,
                 relative_angular[(parent, child)]['omega_parent'],
                 Ff.ang_vel_in(Pf).to_matrix(Pf))
for name, frame in [('C', C), ('L', L), ('R', R), ('B', B)]:
    check_matrix('position_'+name, positions_N[name], origin_points[name].pos_from(No).to_matrix(N))
    check_matrix('velocity_'+name, velocities_N[name], origin_points[name].vel(N).to_matrix(N))
    check_matrix('velocity_recursive_'+name, velocities_recursive_N[name], velocities_N[name])
    check_matrix('velocity_jacobian_'+name, Jv_N[name]*q_kin.diff(t), velocities_N[name])
    check_matrix('omega_'+name, omega_N[name], frame.ang_vel_in(N).to_matrix(N))
    check_matrix('omega_recursive_'+name, omega_recursive_N[name], omega_N[name])
check_matrix('contact_L', v_CL_matrix, CL.vel(N).to_matrix(N))
check_matrix('contact_R', v_CR_matrix, CR.vel(N).to_matrix(N))
assert all(kinematic_checks.values()), kinematic_checks

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
T_exact = (wheel_L.kinetic_energy(N) + wheel_R.kinetic_energy(N)
     + body_B.kinetic_energy(N))
T = T_exact  # 本节显示精确能量，§5a 末切换后续推导使用的能量。
V_exact = m_B * g * (l * sm.cos(theta))  # 在xy平面上运动，没有z方向的位移，以车轴为势能零点

# =====================================================================
# §5a 静止直立点的一阶速度与二阶动能（后续共同输入）
# =====================================================================
# 平衡点：theta=0，psi=psi_0，theta_L=theta_R=0，各速度为0。
# x/y 平衡位置任意；轮角零位沿用初始对齐约定，不假设轮惯量轴对称。
# 以下导数代表平衡点附近的扰动速度；不适用于有限速度转弯的近似。
psi_0 = sm.symbols('psi_0', real=True)
equilibrium_angles = {psi: psi_0, theta: 0, thL: 0, thR: 0}
ex_0 = sm.Matrix([sm.cos(psi_0), sm.sin(psi_0), 0])
ey_0 = sm.Matrix([-sm.sin(psi_0), sm.cos(psi_0), 0])
ez_0 = sm.Matrix([0, 0, 1])
v_C_first = sm.Matrix([x.diff(t), y.diff(t), 0])
v_L_first = v_C_first - d/2*psi.diff(t)*ex_0
v_R_first = v_C_first + d/2*psi.diff(t)*ex_0
v_B_first = v_C_first + l*theta.diff(t)*ex_0
w_L_first = psi.diff(t)*ez_0 + thL.diff(t)*ey_0
w_R_first = psi.diff(t)*ez_0 + thR.diff(t)*ey_0
w_B_first = psi.diff(t)*ez_0 + theta.diff(t)*ey_0
R_0 = R_NC.subs({psi: psi_0})
I_L_N0 = R_0*sm.diag(I_Lxx, I_Lyy, I_Lzz)*R_0.T
I_R_N0 = R_0*sm.diag(I_Rxx, I_Ryy, I_Rzz)*R_0.T
I_B_N0 = R_0*sm.diag(I_Bxx, I_Byy, I_Bzz)*R_0.T
half = sm.Rational(1, 2)
T2_L = half*m_L*v_L_first.dot(v_L_first) + half*(w_L_first.T*I_L_N0*w_L_first)[0]
T2_R = half*m_R*v_R_first.dot(v_R_first) + half*(w_R_first.T*I_R_N0*w_R_first)[0]
T2_B = half*m_B*v_B_first.dot(v_B_first) + half*(w_B_first.T*I_B_N0*w_B_first)[0]
T_quadratic = matrix_simplify(sm.Matrix([T2_L+T2_R+T2_B]))[0]
# 按速度乘积整理的同一表达式，适合直接对账。
v_forward_first = x.diff(t)*sm.cos(psi_0) + y.diff(t)*sm.sin(psi_0)
T_quadratic_compact = (
    half*(m_L+m_R+m_B)*(x.diff(t)**2+y.diff(t)**2)
    + d/2*(m_R-m_L)*psi.diff(t)*v_forward_first
    + m_B*l*theta.diff(t)*v_forward_first
    + half*(I_Lzz+I_Rzz+I_Bzz+d**2/4*(m_L+m_R))*psi.diff(t)**2
    + half*(I_Byy+m_B*l**2)*theta.diff(t)**2
    + half*I_Lyy*thL.diff(t)**2 + half*I_Ryy*thR.diff(t)**2)
# 精确动能本就是速度二次型；静止点的二阶展开等于其速度Hessian在该点的值。
qd_kin = q_kin.diff(t)
T2_from_exact = half*(qd_kin.T*sm.hessian(T_exact, qd_kin).subs(equilibrium_angles)*qd_kin)[0]
assert sm.simplify(sm.expand_trig(T_quadratic-T2_from_exact)) == 0
assert sm.simplify(sm.expand_trig(T_quadratic-T_quadratic_compact)) == 0
# 配套势能保留二阶项，两个后续分支统一使用这套近似能量。
V_quadratic = m_B*g*l*(1-theta**2/2)
L_quadratic = T_quadratic_compact - V_quadratic
T = T_quadratic_compact
V = V_quadratic

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
# §7 共同准备：近似能量的六坐标拉格朗日算子
# =====================================================================
t = me.dynamicsymbols._t
q = sm.Matrix([x, y, psi, theta, thL, thR])
qd = q.diff(t)
qdd = qd.diff(t)
lagrangian = sm.trigsimp(T - V)
# 这三项与纸面求导逐项对应。
dL_dqd = sm.Matrix([sm.diff(lagrangian, v) for v in qd])
d_dt_dL_dqd = dL_dqd.diff(t)
dL_dq = sm.Matrix([sm.diff(lagrangian, z) for z in q])
EL = (d_dt_dL_dqd - dL_dq).applyfunc(sm.trigsimp)
M6 = EL.jacobian(qdd)
h6 = EL.subs(dict.fromkeys(qdd, 0))

# =====================================================================
# §8 分支一：第一类拉格朗日方程（六方向 + 三个约束乘子）
# =====================================================================
constraints = sm.Matrix([lat, rollL, rollR])
A = constraints.jacobian(qd)
lam = sm.Matrix(sm.symbols('lambda_1:4'))
# 电机正转矩沿 C.y 作用于轮，反转矩作用于车体。
tau_L, tau_R = sm.symbols('tau_L tau_R')
motor_power = tau_L*(thL.diff(t)-theta.diff(t)) + tau_R*(thR.diff(t)-theta.diff(t))
Q6 = sm.Matrix([sm.diff(motor_power, v) for v in qd])
Q_constraint = A.T*lam
Q6_total = Q6 + Q_constraint
eom_multiplier = EL - Q6 - A.T*lam  # 每项 = 0
acc_bias = A.diff(t)*qd
augmented_matrix = M6.row_join(-A.T).col_join(A.row_join(sm.zeros(3)))
augmented_rhs = (Q6-h6).col_join(-acc_bias)
# 解此 9×9 线性系统即可同时得到六个加速度和三个乘子。

# =====================================================================
# §9 分支二：第二类无乘子形式（保留非完整修正，三个独立方向）
# =====================================================================
# 使用 z=(x,psi,theta)，依赖速度由三条原始滚动约束解出。
# 不是把约化后的 L 直接套普通欧拉–拉格朗日公式。
z = sm.Matrix([x, psi, theta])
ind = z.diff(t)
acc_ind = ind.diff(t)
dep = sm.solve(list(constraints), [y.diff(t), thL.diff(t), thR.diff(t)])
qd_allowed = qd.subs(dep, simultaneous=True).applyfunc(sm.trigsimp)
S = qd_allowed.jacobian(ind)
acc_allowed = qd_allowed.diff(t).subs(dep, simultaneous=True)
bias = acc_allowed.subs(dict.fromkeys(acc_ind, 0))
L_reduced = lagrangian.subs(dep, simultaneous=True)
p_reduced = sm.Matrix([sm.diff(L_reduced, v) for v in ind])
Lred_z = sm.Matrix([sm.diff(L_reduced, coordinate) for coordinate in z])
# 普通三坐标算子，仅为展示遗漏项；不能单独作为最终左端。
EL_naive = p_reduced.diff(t) - Lred_z
# 正确修正：EL_correct = EL_naive + Lred_z - S.T*L_q - dot(S).T*p_full。
p_full_allowed = dL_dqd.subs(dep, simultaneous=True)
Lq_allowed = dL_dq.subs(dep, simultaneous=True)
Sdot_allowed = S.diff(t).subs(dep, simultaneous=True)
nonholonomic_correction = (Lred_z-S.T*Lq_allowed-Sdot_allowed.T*p_full_allowed).applyfunc(sm.trigsimp)
EL_second = (EL_naive+nonholonomic_correction).applyfunc(sm.trigsimp)
# 直接在功率中代入速度约束，不先投影六方向力。
power_reduced = sm.expand(motor_power.subs(dep, simultaneous=True))
Q_second = sm.Matrix([sm.diff(power_reduced, v) for v in ind]).applyfunc(sm.trigsimp)
M_second = EL_second.jacobian(acc_ind).applyfunc(sm.trigsimp)
h_second = EL_second.subs(dict.fromkeys(acc_ind, 0)).applyfunc(sm.trigsimp)
eom_second = EL_second-Q_second

# =====================================================================
# §10 两分支合流：相同加速度、重建关系与局部一阶模型
# =====================================================================
# 分支一乘 S.T 消去乘子，供两条计算路径交叉检验。
M_projected = (S.T*M6*S).applyfunc(sm.trigsimp)
h_projected = (S.T*(M6*bias+h6.subs(dep, simultaneous=True))).applyfunc(sm.trigsimp)
Q_projected = (S.T*Q6).applyfunc(sm.trigsimp)
assert (M_second-M_projected).applyfunc(sm.simplify) == sm.zeros(3)
assert (h_second-h_projected).applyfunc(sm.simplify) == sm.zeros(3,1)
assert (Q_second-Q_projected).applyfunc(sm.simplify) == sm.zeros(3,1)
# 统一接口，后续仿真解线性系统，不展开符号逆矩阵。
M_reduced, h_reduced, Q_reduced = M_second, h_second, Q_second
eom_reduced = eom_second
# 近似能量+原始约束仍含非线性；以下才是静止点一致的一阶模型。
# z_delta=(delta_x,delta_psi,delta_theta)，名义转矩和速度为零。
rest = {psi: psi_0, theta: 0, tau_L: 0, tau_R: 0}
rest.update(dict.fromkeys(list(ind)+list(acc_ind), 0))
M_linear = M_reduced.subs(rest).applyfunc(sm.trigsimp)
K_linear = h_reduced.jacobian(z).subs(rest).applyfunc(sm.trigsimp)
C_linear = h_reduced.jacobian(ind).subs(rest).applyfunc(sm.trigsimp)
B_linear = Q_reduced.jacobian([tau_L,tau_R]).subs(rest).applyfunc(sm.trigsimp)
# M_linear*delta_zdd + C_linear*delta_zd + K_linear*delta_z = B_linear*tau。
# 重建 y/theta_L/theta_R 时使用 dep 并给出初值；世界x参数化要求cos(psi)!=0。
if __name__ == '__main__':
    print('两个分支均使用近似动能、势能；三方向质量矩阵、偏置、广义力符号核对一致。')
    print('Q_reduced =', Q_reduced.T)
