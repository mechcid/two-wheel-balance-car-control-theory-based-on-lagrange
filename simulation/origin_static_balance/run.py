"""原点静止直立模型：开环、小扰动恢复及平移静止目标的位置调节回归。"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import *
from origin_static_balance.lqr import design
LABELS=['x','psi','theta','dx','dpsi','dtheta']

def run_all():
    model=design();A,B,K=model['A'],model['B'],model['K'];out=Path(__file__).parent/'outputs';summary={}
    # 初始扰动为0.03rad；前进/后退是固定位置目标，不是瞬时变速参考。
    for name,target,closed in [('open_loop',0,False),('balance',0,True),('forward',.1,True),('backward',-.1,True)]:
        ref=np.array([target,0,0,0,0,0.]);z0=np.array([0,0,.03,0,0,0.])
        def control(z):return -K@(z-ref) if closed else np.zeros(2)
        def rhs(t,z):return A@z+B@np.clip(control(z),-TORQUE_LIMIT,TORQUE_LIMIT)
        def cutoff(t,z):return .2-abs(z[2])
        cutoff.terminal=True;cutoff.direction=-1
        def jac(t,z):
            active=(abs(control(z))<TORQUE_LIMIT).astype(float)
            return A-B@np.diag(active)@K if closed else A
        t,z=integrate(rhs,z0,20 if closed else 2,event=None if closed else cutoff,jac=jac)
        refs=np.tile(ref,(len(t),1));raw=np.array([control(zz) for zz in z]);applied=np.clip(raw,-TORQUE_LIMIT,TORQUE_LIMIT)
        summary[name]=save_case(out,name,t,z,refs,z-refs,raw,applied,np.zeros_like(raw),LABELS,model,
           {'plant':'local_linear','open_loop_cutoff_rad':.2,'mass_matrix':model['M']})
        if closed:assert np.max(abs(z[-1]-ref))<1e-3,(name,z[-1])
        else:assert abs(z[-1,2])>.15,'开环未表现为预期的不稳定'
    return summary
if __name__=='__main__':print(json.dumps(run_all(),indent=2,default=serial))
