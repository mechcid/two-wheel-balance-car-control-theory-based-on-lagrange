# -*- coding: utf-8 -*-
"""独立对账: 六坐标全模型 + 拉格朗日乘子 (d'Alembert)
   vs  KanesMethod 约化 (x,psi,theta)
在随机位形比较广义加速度 (x'', psi'', theta'')."""
import sympy as sm
import sympy.physics.mechanics as me
import numpy as np

r, d, l, g = sm.symbols('r d l g', positive=True)
m_L, m_R, m_B = sm.symbols('m_L m_R m_B', positive=True)
I_Lx, I_Ly = sm.symbols('I_Lx I_Ly'); I_Rx, I_Ry = sm.symbols('I_Rx I_Ry')
I_Bx, I_By, I_Bz = sm.symbols('I_Bx I_By I_Bz')

# ============ A) 六坐标全模型 (含自转坐标, 真轮体) ============
x, y, psi, theta, thL, thR = me.dynamicsymbols('x y psi theta thL thR')
N = me.ReferenceFrame('N'); C = me.ReferenceFrame('C')
Lf = me.ReferenceFrame('Lf'); Rf = me.ReferenceFrame('Rf'); B = me.ReferenceFrame('B')
C.orient_axis(N, N.z, psi)
Lf.orient_axis(C, C.y, thL); Rf.orient_axis(C, C.y, thR)
B.orient_axis(C, C.y, theta)

No = me.Point('No'); No.set_vel(N, 0)
Co = me.Point('Co'); Co.set_pos(No, x*N.x + y*N.y + r*N.z)
Co.set_vel(N, x.diff()*N.x + y.diff()*N.y)
Lo = me.Point('Lo'); Ro = me.Point('Ro'); Bo = me.Point('Bo')
Lo.set_pos(Co,  d/2*C.y); Lo.v2pt_theory(Co, N, C)
Ro.set_pos(Co, -d/2*C.y); Ro.v2pt_theory(Co, N, C)
Bo.set_pos(Co,  l*B.z);   Bo.v2pt_theory(Co, N, B)
CL = me.Point('CL'); CR = me.Point('CR')
CL.set_pos(Lo, -r*N.z); CL.set_vel(N, Lo.vel(N) + Lf.ang_vel_in(N).cross(CL.pos_from(Lo)))
CR.set_pos(Ro, -r*N.z); CR.set_vel(N, Ro.vel(N) + Rf.ang_vel_in(N).cross(CR.pos_from(Ro)))

I_Ld = me.inertia(Lf, I_Lx, I_Ly, I_Lx)   # 轮横向相等
I_Rd = me.inertia(Rf, I_Rx, I_Ry, I_Rx)
I_Bd = me.inertia(B, I_Bx, I_By, I_Bz)
wheel_L = me.RigidBody('wheel_L', Lo, Lf, m_L, (I_Ld, Lo))
wheel_R = me.RigidBody('wheel_R', Ro, Rf, m_R, (I_Rd, Ro))
body_B  = me.RigidBody('body_B',  Bo, B, m_B, (I_Bd, Bo))
T = (wheel_L.kinetic_energy(N) + wheel_R.kinetic_energy(N) + body_B.kinetic_energy(N))
V = m_L*g*r + m_R*g*r + m_B*g*(r + l*sm.cos(theta))
lag = me.LagrangesMethod(T - V, [x, y, psi, theta, thL, thR])
lag.form_lagranges_equations()
Mfull = lag.mass_matrix; bfull = lag.forcing

qs6 = [x, y, psi, theta, thL, thR]; qds6 = [q.diff() for q in qs6]
c1 = Co.vel(N).dot(C.y); c2 = CL.vel(N).dot(C.x); c3 = CR.vel(N).dot(C.x)
cs = [c1, c2, c3]
Phi = sm.Matrix(3, 6, lambda k, j: sm.diff(cs[k], qds6[j]))
dcdt = sm.Matrix(3, 1, lambda k, _: sum(sm.diff(cs[k], qm)*qmd for qm, qmd in zip(qs6, qds6)))

# ============ B) KanesMethod 约化 (同 sketch) ============
ux, up, ut = me.dynamicsymbols('ux up ut'); uy, ul, ur = me.dynamicsymbols('uy ul ur')
Cc = me.ReferenceFrame('Cc'); Bc = me.ReferenceFrame('Bc')
Cc.orient_axis(N, N.z, psi); Bc.orient_axis(Cc, Cc.y, theta)
Cc.set_ang_vel(N, up*N.z); Bc.set_ang_vel(N, up*N.z + ut*Cc.y)
WL = me.ReferenceFrame('WL'); WR = me.ReferenceFrame('WR')
WL.orient_axis(Cc, Cc.y, 0); WR.orient_axis(Cc, Cc.y, 0)
WL.set_ang_vel(N, up*N.z + ul*Cc.y); WR.set_ang_vel(N, up*N.z + ur*Cc.y)
Co2 = me.Point('Co2'); Co2.set_vel(N, ux*N.x + uy*N.y)
Lo2 = me.Point('Lo2'); Ro2 = me.Point('Ro2'); Bo2 = me.Point('Bo2')
Lo2.set_pos(Co2,  d/2*Cc.y); Lo2.v2pt_theory(Co2, N, Cc)
Ro2.set_pos(Co2, -d/2*Cc.y); Ro2.v2pt_theory(Co2, N, Cc)
Bo2.set_pos(Co2,  l*Bc.z);   Bo2.v2pt_theory(Co2, N, Bc)
CL2 = me.Point('CL2'); CR2 = me.Point('CR2')
CL2.set_pos(Lo2, -r*N.z); CL2.set_vel(N, Lo2.vel(N) + WL.ang_vel_in(N).cross(CL2.pos_from(Lo2)))
CR2.set_pos(Ro2, -r*N.z); CR2.set_vel(N, Ro2.vel(N) + WR.ang_vel_in(N).cross(CR2.pos_from(Ro2)))
wL = me.RigidBody('wL', Lo2, WL, m_L, (me.inertia(WL, I_Lx, I_Ly, I_Lx), Lo2))
wR = me.RigidBody('wR', Ro2, WR, m_R, (me.inertia(WR, I_Rx, I_Ry, I_Rx), Ro2))
bB = me.RigidBody('bB', Bo2, Bc, m_B, (me.inertia(Bc, I_Bx, I_By, I_Bz), Bo2))
lat2 = Co2.vel(N).dot(Cc.y); rl2 = CL2.vel(N).dot(Cc.x); rr2 = CR2.vel(N).dot(Cc.x)
km = me.KanesMethod(N, q_ind=[x, psi, theta], u_ind=[ux, up, ut],
                    kd_eqs=[x.diff()-ux, psi.diff()-up, theta.diff()-ut],
                    u_dependent=[uy, ul, ur], velocity_constraints=[lat2, rl2, rr2],
                    bodies=[wL, wR, bB])
gv = -g*N.z
km.kanes_equations(loads=[(Lo2, m_L*gv), (Ro2, m_R*gv), (Bo2, m_B*gv)])
Mk, fk = km.mass_matrix, km.forcing
uk_all = [ux, up, ut, uy, ul, ur]; acc_k = [u.diff() for u in uk_all]

# ============ 数值对账 ============
sub_state = {x: 0.0, y: 0.0, psi: 0.0, theta: 0.0, thL: 0.0, thR: 0.0,
             x.diff(): 0.0, y.diff(): 0.0, psi.diff(): 0.0, theta.diff(): 0.0,
             thL.diff(): 0.0, thR.diff(): 0.0}
sub_state2 = {x: 0.0, psi: 0.0, theta: 0.0, ux: 0.0, up: 0.0, ut: 0.0,
              uy: 0.0, ul: 0.0, ur: 0.0}
p = {m_L: 0.40, m_R: 0.65, m_B: 2.10, I_Lx: 0.0022, I_Ly: 0.0041,
     I_Rx: 0.0030, I_Ry: 0.0056, I_Bx: 0.01, I_By: 0.02, I_Bz: 0.012,
     r: 0.1, d: 0.3, l: 0.4, g: 9.81}
Mfull_f = sm.lambdify((qs6, qds6), Mfull.subs(p), 'numpy')
bfull_f = sm.lambdify((qs6, qds6), bfull.subs(p), 'numpy')
Phi_f   = sm.lambdify((qs6, qds6), Phi.subs(p), 'numpy')
dcdt_f  = sm.lambdify((qs6, qds6), dcdt.subs(p), 'numpy')
Mk_f    = sm.lambdify(([x, psi, theta], uk_all), Mk.subs(p), 'numpy')
fk_f    = sm.lambdify(([x, psi, theta], uk_all), fk.subs(p), 'numpy')

rng = np.random.default_rng(3)
maxerr = 0.0
for trial in range(8):
    # 独立速度: ẋ=vx, ψ̇=wp, θ̇=wt
    vx = float(rng.uniform(-1, 1)); wp = float(rng.uniform(-1, 1)); wt = float(rng.uniform(-1, 1))
    ps = float(rng.uniform(-0.9, 0.9)); th = float(rng.uniform(-0.5, 0.5))  # cosψ 不近零
    st_full = [x, y, psi, theta, thL, thR]
    sp_full = [x.diff(), y.diff(), psi.diff(), theta.diff(), thL.diff(), thR.diff()]
    # 用约束在数值上解出依赖速度 (y', θ̇L, θ̇R)
    Phi0 = np.array(Phi_f([0.0, 0.0, ps, th, 0.0, 0.0],
                          [0.0]*3 + [0.0, 0.0, 0.0]), dtype=float).reshape(3, 6)
    Phi_ind = Phi0[:, [0, 2, 3]]; Phi_dep = Phi0[:, [1, 4, 5]]
    dep = np.linalg.solve(Phi_dep, -Phi_ind @ np.array([vx, wp, wt]))
    vy, wL_, wR_ = dep
    qv = np.array([0.0, 0.0, ps, th, 0.0, 0.0])
    qdv = np.array([vx, vy, wp, wt, wL_, wR_])
    Mf = np.array(Mfull_f(qv, qdv), dtype=float).reshape(6, 6)
    bf = np.array(bfull_f(qv, qdv), dtype=float).reshape(6, 1)
    Pf = np.array(Phi_f(qv, qdv), dtype=float).reshape(3, 6)
    dcf = np.array(dcdt_f(qv, qdv), dtype=float).reshape(3, 1)
    Ablk = np.block([[Mf, -Pf.T], [Pf, np.zeros((3, 3))]])
    RHSv = np.block([[bf], [-dcf]])
    xlam = np.linalg.solve(Ablk, RHSv)
    qdd_full = xlam[:6].ravel()
    # Kane 数值解
    qr = np.array([0.0, ps, th]); ukr = np.array([vx, wp, wt, vy, wL_, wR_])
    Mk0 = np.array(Mk_f(qr, ukr), dtype=float).reshape(6, 6)
    fk0 = np.array(fk_f(qr, ukr), dtype=float).reshape(6, 1)
    acc = np.linalg.solve(Mk0, fk0).ravel()
    # full: x'', psi'', theta'' 对应 idx 0,2,3; kane 前 3
    full_red = qdd_full[[0, 2, 3]]
    kane_red = acc[:3]
    err = np.max(np.abs(full_red - kane_red))
    maxerr = max(maxerr, err)
    print(f"trial {trial}: ψ={ps:+.2f} θ={th:+.2f} | full x\",ψ\",θ\"={np.round(full_red,8)}"
          f"  kane={np.round(kane_red,8)}  err={err:.2e}")

print("\n最大误差:", f"{maxerr:.2e}", " → ", "一致 ✓" if maxerr < 1e-9 else "不一致 ✗")

# ============ C) 电机扭矩一致性: 全模型乘子 vs Kane(带扭矩) ============
tau_L, tau_R = sm.symbols('tau_L tau_R')
Qfull_motor = sm.Matrix([0, 0, 0, -(tau_L + tau_R), tau_L, tau_R])  # x,y,ψ,θ,θL,θR
# Kane 电机载荷
kml = me.KanesMethod(N, q_ind=[x, psi, theta], u_ind=[ux, up, ut],
                     kd_eqs=[x.diff()-ux, psi.diff()-up, theta.diff()-ut],
                     u_dependent=[uy, ul, ur], velocity_constraints=[lat2, rl2, rr2],
                     bodies=[wL, wR, bB])
motl = [(WL, tau_L*Cc.y), (WR, tau_R*Cc.y), (Bc, -(tau_L + tau_R)*Cc.y)]
kml.kanes_equations(loads=[(Lo2, m_L*gv), (Ro2, m_R*gv), (Bo2, m_B*gv)] + motl)
Mkl, fkl = kml.mass_matrix, kml.forcing
Mkl_f = sm.lambdify(([x, psi, theta], uk_all), Mkl.subs(p), 'numpy')
fkl_f = sm.lambdify(([x, psi, theta], uk_all, [tau_L, tau_R]), fkl.subs(p), 'numpy')

maxerr2 = 0.0
rng2 = np.random.default_rng(11)
for trial in range(6):
    vx = float(rng2.uniform(-1, 1)); wp = float(rng2.uniform(-1, 1)); wt = float(rng2.uniform(-1, 1))
    tL = float(rng2.uniform(-1, 1)); tR = float(rng2.uniform(-1, 1))
    ps = float(rng2.uniform(-0.9, 0.9)); th = float(rng2.uniform(-0.5, 0.5))
    Phi0 = np.array(Phi_f([0, 0, ps, th, 0, 0], [0.0]*3 + [0.0, 0.0, 0.0]), dtype=float).reshape(3, 6)
    dep = np.linalg.solve(Phi0[:, [1, 4, 5]], -Phi0[:, [0, 2, 3]] @ np.array([vx, wp, wt]))
    vy, wLL, wRR = dep
    qv = np.array([0.0, 0.0, ps, th, 0.0, 0.0]); qdv = np.array([vx, vy, wp, wt, wLL, wRR])
    Mf = np.array(Mfull_f(qv, qdv), dtype=float).reshape(6, 6)
    bf = np.array(bfull_f(qv, qdv), dtype=float).reshape(6, 1)
    Qm = np.array([0, 0, 0, -(tL + tR), tL, tR], dtype=float).reshape(6, 1)
    Pf = np.array(Phi_f(qv, qdv), dtype=float).reshape(3, 6)
    dcf = np.array(dcdt_f(qv, qdv), dtype=float).reshape(3, 1)
    Ablk = np.block([[Mf, -Pf.T], [Pf, np.zeros((3, 3))]])
    RHSv = np.block([[bf + Qm], [-dcf]])
    qdd_full = np.linalg.solve(Ablk, RHSv)[:6].ravel()
    qr = np.array([0.0, ps, th]); ukr = np.array([vx, wp, wt, vy, wLL, wRR])
    Mk0 = np.array(Mkl_f(qr, ukr), dtype=float).reshape(6, 6)
    fk0 = np.array(fkl_f(qr, ukr, [tL, tR]), dtype=float).reshape(6, 1)
    acc = np.linalg.solve(Mk0, fk0).ravel()
    full_red = qdd_full[[0, 2, 3]]; kane_red = acc[:3]
    err2 = np.max(np.abs(full_red - kane_red)); maxerr2 = max(maxerr2, err2)
    print(f"[motor] trial {trial}: τL={tL:+.2f} τR={tR:+.2f} | err={err2:.2e}")
print("电机情形最大误差:", f"{maxerr2:.2e}", "→", "一致 ✓" if maxerr2 < 1e-9 else "不一致 ✗")
