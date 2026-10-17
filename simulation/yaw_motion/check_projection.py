"""Independent 9x9 multiplier solve against implemented 3-speed model."""
from pathlib import Path
import runpy,sys,json
import numpy as np
import sympy as s
import yaml
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'ros2_ws/src/balance_yaw'))
from balance_yaw.dynamics import matrices
sys.path.insert(0,str(root/'theory'))
from turning_balance import lagrange as theory
model=theory.build()
ns={**model,'q':theory.q,'qd':theory.qd,
    'r':theory.r,'d':theory.d,'lc':theory.l,'g':theory.g,
    'mB':theory.m_B,'mW':theory.mW,'Js':theory.Js,'Jt':theory.Jt,
    'Ix':theory.I_Bxx,'Iy':theory.I_Byy,'Iz':theory.I_Bzz}
p=yaml.safe_load((root/'ros2_ws/src/balance_sim/config/physical.yaml').read_text());p['com_height']=p['l']
subs=dict(zip([ns[k] for k in ['r','d','lc','g','mB','mW','Js','Jt','Ix','Iy','Iz']], [p[k] for k in ['r','d','com_height','g','m_B','m_L','I_Lyy','I_Lzz','I_Bxx','I_Byy','I_Bzz']]))
q,qd=ns['q'],ns['qd']; A=ns['Acon'];S=ns['S'];uu=ns['u']
M6=s.lambdify((q,qd),ns['M6'].subs(subs),'numpy');h6=s.lambdify((q,qd),ns['h6'].subs(subs),'numpy')
Ac=s.lambdify((q,),A.subs(subs),'numpy');Af=s.lambdify((q,qd),(A*qd).jacobian(q).subs(subs)*qd,'numpy')
Sf=s.lambdify((q,),S.subs(subs),'numpy');Q=np.array(ns['B6']).astype(float)
rng=np.random.default_rng(16);worst=0.
for _ in range(30):
    qq=rng.uniform(-2,2,6);qq[3]=rng.uniform(-.15,.15);u=rng.uniform(-.6,.6,3);tau=rng.uniform(-.2,.2,2)
    SS=Sf(qq);vel=SS@u;AA=Ac(qq);MM=M6(qq,vel);hh=h6(qq,vel).ravel()
    saddle=np.block([[MM,-AA.T],[AA,np.zeros((3,3))]])
    full=np.linalg.solve(saddle,np.r_[Q@tau-hh,-Af(qq,vel).ravel()])[:6]
    reduced=np.array([np.cos(qq[2])*full[0]+np.sin(qq[2])*full[1],full[2],full[3]])
    m,h,b=matrices(qq[3],*u,p)
    expected=np.linalg.solve(m,b@tau-h)
    worst=max(worst,float(max(abs(reduced-expected))))
assert worst<1e-10
out=root/'simulation/yaw_motion/outputs/projection_check.json';out.write_text(json.dumps({'samples':30,'max_acceleration_difference':worst},indent=2));print(worst)
