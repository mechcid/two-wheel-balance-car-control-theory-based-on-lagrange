"""重构验收：运动学、两种拉格朗日、旧矩阵、七状态线性化与LQR。"""
import json
from pathlib import Path
import numpy as np
import sympy as s
from scipy.integrate import solve_ivp
import base as b
from origin_static_balance import lagrange as origin_static_balance
from turning_balance import lagrange as turn
from origin_static_balance.lqr import design as design_origin_static
from turning_balance.lqr import design,state_matrices,error_transform,physical_subs,reference,trim,rhs
from parameters import substitutions,numeric

def main():
    k=b.kinematics()
    for name,R in k['R'].items():
        assert b.clean(R.T*R-s.eye(3))==s.zeros(3)
        assert b.clean(k['omega_hat'][name]-b.skew(k['omega'][name]))==s.zeros(3)
    # 静止分支与旧数值函数核查由 simulation/check_matrix_source.py 执行。
    print('构建精确模型与静止模型',flush=True)
    st=origin_static_balance.build();moving=turn.build();rng=np.random.default_rng(17)
    errors={}
    for label,model,sub in [('origin_static_balance',st,{**substitutions(),origin_static_balance.psi_0:.37}),('turning',moving,physical_subs())]:
        print('校验乘子投影:',label,flush=True)
        S=model['S']; u=model['u']; allowed=dict(zip(b.qd,S*u))
        def fn(expr):return s.lambdify((b.q,u),expr.subs(allowed,simultaneous=True).subs(sub),'numpy',cse=True)
        aug=fn(model['augmented_matrix']);h=fn(model['h6']);adot=fn((model['Acon']*b.qd).jacobian(b.q)*b.qd)
        M=fn(model['M']);hr=fn(model['h']);B=fn(model['B']);Sf=fn(S);bias=fn(model['bias'])
        B6=np.array(model['B6'],float);worst=0.
        for _ in range(20):
            q=rng.uniform(-1,1,6);q[3]=rng.uniform(-.1,.1);vel=rng.uniform(-.5,.5,3);tau=rng.uniform(-.2,.2,2)
            full=np.linalg.solve(aug(q,vel),np.r_[B6@tau-h(q,vel).ravel(),-adot(q,vel).ravel()])[:6]
            red=np.linalg.solve(M(q,vel),B(q,vel)@tau-hr(q,vel).ravel())
            restored=Sf(q,vel)@red+bias(q,vel).ravel()
            worst=max(worst,float(np.max(abs(full-restored))))
        assert worst<1e-9
        errors[label+'_multiplier_projection_max_error']=worst
    print('校验 CAD 与参考解',flush=True)
    # CAD 正惯性积的符号与主轴方向必须同时匹配；只核特征值不够。
    sub=physical_subs()
    J=np.array([[sub[b.I_Bxx],sub[b.I_Bxy],sub[b.I_Bxz]],
                [sub[b.I_Bxy],sub[b.I_Byy],sub[b.I_Byz]],
                [sub[b.I_Bxz],sub[b.I_Byz],sub[b.I_Bzz]]])
    eig,axes=np.linalg.eigh(J)
    np.testing.assert_allclose(eig,np.array([2539.715,2748.923,4441.796])*1e-6,atol=5e-10,rtol=0)
    axis=np.array([-.324,.946,0.]);axis/=np.linalg.norm(axis)
    assert abs(axis@axes[:,0])>1-1e-6
    # 惯性积置零，必须恢复原精确转向方程。
    zero={b.I_Bxy:0,b.I_Bxz:0,b.I_Byz:0}
    assert b.clean(moving['delta_M'].subs(zero))==s.zeros(3)
    assert b.clean(moving['delta_h'].subs(zero))==s.zeros(3,1)
    # 左转、右转、倒车、直行及静止参考均验证三个动力学方程。
    for vv,ww in [(0,0),(.1,0),(.1,.4),(.1,-.4),(-.1,.4),(.3,1.2)]:
        tr=trim(vv,ww);z=reference(0,vv,ww,.7)
        assert np.max(abs(rhs(z,tr['tau'])[4:]))<1e-10
    print('求解 LQR',flush=True)
    result=design(); static=design_origin_static()
    assert result['rank']==7 and static['rank']==6
    # 用精确非线性动力学的数值微分，独立核对沿轨迹的七状态 A/B。
    mf=s.lambdify((b.theta,),moving['M'].subs(physical_subs()),'numpy')
    hf=s.lambdify((b.theta,turn.v,turn.omega,turn.p),moving['h'].subs(physical_subs()),'numpy')
    bf=numeric(moving['B'],physical_subs())
    def f(z,tau):
        x,y,psi,th,v,w,p=z
        acc=np.linalg.solve(mf(th),bf@tau-hf(th,v,w,p).ravel())
        return np.r_[v*np.cos(psi),v*np.sin(psi),w,p,acc]
    print('校验精确雅可比',flush=True)
    fd_error=0.
    for heading in (0.,.7,2.1):
        z=reference(0,psi0=heading);tau_r=result['tau_ref'];eps=1e-6
        eye=np.eye(7)*eps
        fdA=np.column_stack([(f(z+e,tau_r)-f(z-e,tau_r))/(2*eps) for e in eye])
        fdB=np.column_stack([(f(z,tau_r+e)-f(z,tau_r-e))/(2*eps) for e in np.eye(2)*eps])
        A,B,Ae,H=state_matrices(heading=heading)
        fd_error=max(fd_error,float(np.max(abs(fdA-A))),float(np.max(abs(fdB-B))))
        np.testing.assert_allclose(Ae,result['A'],atol=1e-12)
    assert fd_error<1e-7
    # LTV世界误差与旋转坐标误差在同一时刻应一致。
    e0=np.array([.01,.01,.01,.03,0,0,0.]); times=np.linspace(0,35,351)
    def world_rhs(t,z):
        A,B,Ae,H=state_matrices_fast(t)
        return (A-B@result['K']@H)@z
    # 循环中避免重新做符号替换。
    Ae=result['A'];B=result['B'];w=.4
    def state_matrices_fast(t):
        H=error_transform(w*t);Hd=np.zeros((7,7));Hd[:2,:2]=np.array([[0,w],[-w,0]])@H[:2,:2]
        A=H.T@(Ae-Hd@H.T)@H
        return A,B,Ae,H
    print('积分世界误差和旋转误差',flush=True)
    solw=solve_ivp(world_rhs,(0,35),e0,t_eval=times,rtol=1e-9,atol=1e-11,method='Radau')
    sole=solve_ivp(lambda t,e:(Ae-B@result['K'])@e,(0,35),e0,t_eval=times,rtol=1e-9,atol=1e-11,method='Radau')
    reconstructed=np.column_stack([error_transform(.4*t)@z for t,z in zip(times,solw.y.T)])
    equiv=float(np.max(abs(reconstructed-sole.y)))
    assert equiv<1e-7
    # 本次新增七状态反馈在精确模型上作小误差冒烟测试，不替代 Gazebo 验收。
    z0=reference(0)+e0
    def nonlinear_rhs(t,z):
        err=error_transform(.4*t)@(z-reference(t))
        return f(z,result['tau_ref']-result['K']@err)
    print('积分精确非线性闭环',flush=True)
    soln=solve_ivp(nonlinear_rhs,(0,35),z0,t_eval=times,rtol=1e-9,atol=1e-11,method='Radau')
    assert soln.success and solw.success and sole.success
    final_error=float(np.max(abs(soln.y[:,-1]-reference(35))))
    assert final_error<1e-3
    errors.update(theta_ref=result['theta_ref'],tau_ref=result['tau_ref'].tolist(),trim_residual=result['trim_residual'],origin_static_balance_rank=static['rank'],turning_rank=result['rank'],
                  finite_difference_error=fd_error,world_rotating_equivalence_error=equiv,
                  nonlinear_final_max_error=final_error,turning_care_residual=result['care_residual'],
                  turning_closed_poles=[str(x) for x in result['poles']])
    output=Path(__file__).parent/'validation.json';output.write_text(json.dumps(errors,indent=2))
    print(json.dumps(errors,indent=2));print('PASS: all theory checks')
if __name__=='__main__':main()
