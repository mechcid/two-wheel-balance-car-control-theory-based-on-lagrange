from pathlib import Path
import sys,json
import numpy as np
import yaml
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/balance_yaw'))
from balance_yaw.dynamics import linearize,rhs,reference,matrices
p=yaml.safe_load((ROOT/'ros2_ws/src/balance_sim/config/physical.yaml').read_text())
# New coordinate l is travel; old physical parameter l means COM height.
p['com_height']=p['l']
old=json.loads((ROOT/'simulation/outputs/results.json').read_text())
Q=np.diag(old['settings']['q_weights']);R=np.diag(old['settings']['r_weights'])
v0,w0=.10,.40
A,B=linearize(p,v0,w0);P=solve_continuous_are(A,B,Q,R);K=np.linalg.solve(R,B.T@P)
A0,B0=linearize(p,0,0);P0=solve_continuous_are(A0,B0,Q,R);K0=np.linalg.solve(R,B0.T@P0)
np.testing.assert_allclose(A0,old['matrices']['A'],atol=1e-12)
np.testing.assert_allclose(B0,old['matrices']['B'],atol=1e-12)
np.testing.assert_allclose(K0,old['matrices']['K_LQR'],atol=1e-9)
# Finite difference Jacobian around a moving relative equilibrium.
X0=np.array([0,1.1,0,v0,w0,0.]);eps=1e-6
J=np.column_stack([(rhs(X0+eps*np.eye(6)[i],[0,0],p)-rhs(X0-eps*np.eye(6)[i],[0,0],p))/(2*eps) for i in range(6)])
np.testing.assert_allclose(A,J,atol=1e-8)
# Exact symbolic derivation verifies projection and power; this checks runtime linearization.
rank=np.linalg.matrix_rank(np.hstack([np.linalg.matrix_power(A,i)@B for i in range(6)]))
assert rank==6 and np.max(np.linalg.eigvals(A-B@K).real)<0
out=ROOT/'simulation/yaw_motion/outputs';out.mkdir(exist_ok=True)
checks={}
for heading,rate,name in [(.7,.4,'turn'),(-1.1,-.4,'reverse_turn'),(1.2,0.,'fixed_heading')]:
    def f(t,X):return rhs(X,np.clip(-K@(X-reference(t,v0,rate,heading)),-1,1),p)
    ts=np.linspace(0,30,6001);sol=solve_ivp(f,(0,30),[0,heading,.03,0,0,0],t_eval=ts,rtol=1e-9,atol=1e-11)
    assert sol.success
    refs=np.array([reference(t,v0,rate,heading) for t in ts]);errors=sol.y.T-refs
    assert np.max(abs(errors[-400:]))<.01
    checks[name]={'final_error':errors[-1].tolist(),'peak_pitch':float(max(abs(sol.y[2])))}
    np.savetxt(out/f'{name}.csv',np.column_stack([ts,sol.y.T,refs]),delimiter=',',header='t,l,psi,theta,v,omega,p,l_ref,psi_ref,theta_ref,v_ref,omega_ref,p_ref',comments='')
config={'balance_yaw':{'ros__parameters':{'use_sim_time':True,'gain':K.ravel().tolist(),'control_frequency':200.,'torque_limit':1.,'forward_speed':v0,'yaw_rate':w0,'heading0':.7,'log_path':'/tmp/balance_yaw.csv','disturbance':False}}}
(ROOT/'ros2_ws/src/balance_yaw/config/yaw.yaml').write_text(yaml.safe_dump(config))
report={'parameters':p,'v0':v0,'omega0':w0,'Q':Q.tolist(),'R':R.tolist(),'A':A.tolist(),'B':B.tolist(),'K':K.tolist(),'rank':rank,'closed_poles':[[z.real,z.imag] for z in np.linalg.eigvals(A-B@K)],'numerical_tests':checks,'checks':['zero speed recovers old A B K','moving Jacobian finite difference','rank6','three exact nonlinear rollouts']}
(out/'design.json').write_text(json.dumps(report,indent=2,default=int));print(json.dumps(checks,indent=2));print('rank=',rank,'K=',K)
