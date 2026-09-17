# -*- coding: utf-8 -*-
"""电机扭矩 τ_L,τ_R -> (x,ψ,θ) 三方向广义力 Q = B(q)·[τL;τR]"""
import sympy as sm
import sympy.physics.mechanics as me

r, d, l, g = sm.symbols('r d l g', positive=True)
m_L, m_R, m_B = sm.symbols('m_L m_R m_B', positive=True)
I_Lx, I_Ly = sm.symbols('I_Lx I_Ly'); I_Rx, I_Ry = sm.symbols('I_Rx I_Ry')
I_Bx, I_By, I_Bz = sm.symbols('I_Bx I_By I_Bz')
tau_L, tau_R = sm.symbols('tau_L tau_R')

x, psi, theta = me.dynamicsymbols('x psi theta')
ux, up, ut = me.dynamicsymbols('ux up ut'); uy, ul, ur = me.dynamicsymbols('uy ul ur')
N = me.ReferenceFrame('N'); C = me.ReferenceFrame('C'); B = me.ReferenceFrame('B')
C.orient_axis(N, N.z, psi); B.orient_axis(C, C.y, theta)
C.set_ang_vel(N, up*N.z); B.set_ang_vel(N, up*N.z + ut*C.y)
WL = me.ReferenceFrame('WL'); WR = me.ReferenceFrame('WR')
WL.orient_axis(C, C.y, 0); WR.orient_axis(C, C.y, 0)
WL.set_ang_vel(N, up*N.z + ul*C.y); WR.set_ang_vel(N, up*N.z + ur*C.y)
Co = me.Point('Co'); Co.set_vel(N, ux*N.x + uy*N.y)
Lo = me.Point('Lo'); Ro = me.Point('Ro'); Bo = me.Point('Bo')
Lo.set_pos(Co,  d/2*C.y); Lo.v2pt_theory(Co, N, C)
Ro.set_pos(Co, -d/2*C.y); Ro.v2pt_theory(Co, N, C)
Bo.set_pos(Co,  l*B.z);   Bo.v2pt_theory(Co, N, B)
CL = me.Point('CL'); CR = me.Point('CR')
CL.set_pos(Lo, -r*N.z); CL.set_vel(N, Lo.vel(N) + WL.ang_vel_in(N).cross(CL.pos_from(Lo)))
CR.set_pos(Ro, -r*N.z); CR.set_vel(N, Ro.vel(N) + WR.ang_vel_in(N).cross(CR.pos_from(Ro)))
wL = me.RigidBody('wL', Lo, WL, m_L, (me.inertia(WL, I_Lx, I_Ly, I_Lx), Lo))
wR = me.RigidBody('wR', Ro, WR, m_R, (me.inertia(WR, I_Rx, I_Ry, I_Rx), Ro))
bB = me.RigidBody('bB', Bo, B, m_B, (me.inertia(B, I_Bx, I_By, I_Bz), Bo))
lat = Co.vel(N).dot(C.y); rl = CL.vel(N).dot(C.x); rr = CR.vel(N).dot(C.x)
km = me.KanesMethod(N, q_ind=[x, psi, theta], u_ind=[ux, up, ut],
                    kd_eqs=[x.diff()-ux, psi.diff()-up, theta.diff()-ut],
                    u_dependent=[uy, ul, ur], velocity_constraints=[lat, rl, rr],
                    bodies=[wL, wR, bB])
gv = -g*N.z
grav = [(Lo, m_L*gv), (Ro, m_R*gv), (Bo, m_B*gv)]
mot  = [(WL, tau_L*C.y), (WR, tau_R*C.y), (B, -(tau_L + tau_R)*C.y)]

def reduced(M6, f6):
    u_all = [ux, up, ut, uy, ul, ur]
    u_ind = [ux, up, ut]; u_dep = [uy, ul, ur]
    Mind, Mdep = M6[:3, :], M6[3:, :]
    f_ind, f_dep = f6[:3, :], f6[3:, :]
    acc_all = sm.Matrix([u.diff() for u in u_all])
    acc_ind = sm.Matrix([u.diff() for u in u_ind])
    dep_syms = [u.diff() for u in u_dep]
    d0 = sm.solve(Mdep*acc_all - f_dep, dep_syms, dict=True)[0]
    P = sm.Matrix(3, 3, lambda i, j: d0[dep_syms[i]].coeff(acc_ind[j]))
    p = sm.Matrix(3, 1, lambda i, _: d0[dep_syms[i]].subs({acc_ind[j]: 0 for j in range(3)}))
    Mr = sm.trigsimp(Mind[:, :3] + Mind[:, 3:]*P)
    fr = sm.trigsimp(f_ind - Mind[:, 3:]*p)
    return Mr, fr

km.kanes_equations(loads=grav)
Mr, fr_g = reduced(km.mass_matrix, km.forcing)
km.kanes_equations(loads=grav + mot)
Mr2, fr_m = reduced(km.mass_matrix, km.forcing)
print("Mr 相同:", Mr == Mr2)
Q = fr_m - fr_g
# Q 应线性于 (tau_L, tau_R)
Qtau = sm.Matrix(3, 2, lambda i, j: sm.trigsimp(Q[i].coeff(tau_L if j == 0 else tau_R)))
print("Q 中 tau^2 残余非零? ", any(sm.degree(Q[i], tau_L) > 1 or sm.degree(Q[i], tau_R) > 1 for i in range(3)))

print("\nB(q) 广义力映射 Q = B·[τ_L, τ_R]:")
for i, nm in enumerate(['x', 'psi', 'theta']):
    print(f"  Q_{nm} = {Qtau[i,0]}·τ_L + {Qtau[i,1]}·τ_R")

# 直行特例 ψ=0: 期望 Q_x=(τL+τR)/r, Q_ψ = (?)·(τR-τL), Q_θ=-(τL+τR) 级
s0 = {psi: 0, theta: 0, ux: 0, up: 0, ut: 0}
print("\nψ=0, θ=0 特例:")
for i, nm in enumerate(['x', 'psi', 'theta']):
    bL = sm.simplify(Qtau[i, 0].subs(s0)); bR = sm.simplify(Qtau[i, 1].subs(s0))
    print(f"  Q_{nm} = {bL}·τ_L + {bR}·τ_R")
