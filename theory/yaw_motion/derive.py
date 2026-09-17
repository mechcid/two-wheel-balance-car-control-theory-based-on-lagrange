"""Exact three-body energy -> six-coordinate Lagrange -> nonholonomic projection.
No ordinary Euler-Lagrange operator on an incorrectly reduced Lagrangian.
"""
import sympy as s
from pathlib import Path
import json
q=s.Matrix(s.symbols('x y psi theta alpha_L alpha_R', real=True))
x,y,psi,th,al,ar=q
qd=s.Matrix(s.symbols('dx dy w p spin_L spin_R',real=True))
dx,dy,w,p,wl,wr=qd
r,d,lc,g,mB,mW,Js,Jt,Ix,Iy,Iz=s.symbols('r d l_c g m_B m_W J_s J_t I_x I_y I_z',positive=True)
half=s.Rational(1,2)
# Symmetric wheels, same assumption as current numerical/Gazebo example.
fwd=dx*s.cos(psi)+dy*s.sin(psi)
lat=-dx*s.sin(psi)+dy*s.cos(psi)
T=half*mW*((dx-d*w*s.cos(psi)/2)**2+(dy-d*w*s.sin(psi)/2)**2)
T+=half*mW*((dx+d*w*s.cos(psi)/2)**2+(dy+d*w*s.sin(psi)/2)**2)
T+=half*Js*(wl**2+wr**2)+Jt*w**2
T+=half*mB*(dx**2+dy**2+lc*lc*p*p+lc*lc*s.sin(th)**2*w*w+2*lc*s.cos(th)*p*fwd+2*lc*s.sin(th)*w*lat)
T+=half*(Iy*p*p+(Ix*s.sin(th)**2+Iz*s.cos(th)**2)*w*w)
V=mB*g*lc*s.cos(th)
L=T-V
M6=s.hessian(T,qd)
momentum=s.Matrix([s.diff(L,v) for v in qd])
h6=momentum.jacobian(q)*qd-s.Matrix([s.diff(L,v) for v in q])
v,om,pp=s.symbols('v omega p_rate',real=True)
u=s.Matrix([v,om,pp]);ud=s.Matrix(s.symbols('dv domega dp',real=True))
S=s.Matrix([[s.cos(psi),0,0],[s.sin(psi),0,0],[0,1,0],[0,0,1],[1/r,-d/(2*r),0],[1/r,d/(2*r),0]])
sub=dict(zip(qd,S*u)); bias=(S*u).jacobian(q)*(S*u)
clean=lambda a:a.applyfunc(lambda z:s.trigsimp(s.expand_trig(s.expand(z))))
M=clean(S.T*M6*S); h=clean(S.T*(M6*bias+h6.subs(sub,simultaneous=True)))
a=mB+2*mW+2*Js/r**2;c=mB*lc;e=Iy+mB*lc*lc
J=Iz+2*Jt+d*d*mW/2+d*d*Js/(2*r*r)
D=Ix-Iz+mB*lc*lc
expected_M=s.Matrix([[a,0,c*s.cos(th)],[0,J+D*s.sin(th)**2,0],[c*s.cos(th),0,e]])
expected_h=s.Matrix([-c*s.sin(th)*(pp**2+om**2),(2*D*s.sin(th)*s.cos(th)*pp+c*s.sin(th)*v)*om,-D*s.sin(th)*s.cos(th)*om**2-c*g*s.sin(th)])
assert clean(M-expected_M)==s.zeros(3)
assert clean(h-expected_h)==s.zeros(3,1)
B6=s.Matrix([[0,0],[0,0],[0,0],[-1,-1],[1,0],[0,1]])
B=clean(S.T*B6)
# Unconstrained multipliers eliminated by allowed virtual velocity projection.
Acon=s.Matrix([lat,fwd-d*w/2-r*wl,fwd+d*w/2-r*wr]).jacobian(qd)
assert clean(Acon*S)==s.zeros(3)
# Ideal power consistency: dE/dt = u.T B tau (no dissipation).
Tr=clean(s.Matrix([T.subs(sub,simultaneous=True)]))[0]
E=Tr+V
power_rate=s.diff(E,th)*pp+(s.Matrix([s.diff(E,z) for z in u]).T*ud)[0]
assert s.simplify(power_rate-(u.T*(M*ud+h))[0])==0
out=Path(__file__).parent
(out/'equations.json').write_text(json.dumps({'M':str(M),'h':str(h),'B':str(B),'M_latex':s.latex(expected_M),'h_latex':s.latex(expected_h),'checks':['six-coordinate projection','constraint nullspace','energy power identity']},indent=2))
print('PASS: exact Lagrange projection, nullspace and power identity.');print('M=',M);print('h=',h);print('B=',B)
