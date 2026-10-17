"""七状态跟踪：近似动力学主线 + 精确动力学对照；s只是积分记录，不增加反馈维数。"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import *
import sympy as sym
from turning_balance.lagrange import build,theta,v,omega,p
from turning_balance.lqr import design,reference,error_transform,physical_subs
LABELS=['x','y','psi','theta','v','omega','p','s']

def plants():
    m=build();sub=physical_subs();B=np.asarray(m['B'].subs(sub),float);out={}
    for name,mkey,hkey in [('small_pitch','M_small','h_small'),('exact','M','h')]:
        mf=sym.lambdify((theta,),m[mkey].subs(sub),'numpy',cse=True)
        hf=sym.lambdify((theta,v,omega,p),m[hkey].subs(sub),'numpy',cse=True)
        def rhs(z,tau,mf=mf,hf=hf):
            x,y,psi,th,vv,w,pp,travel=z
            acc=np.linalg.solve(mf(th),B@tau-np.asarray(hf(th,vv,w,pp)).ravel())
            return np.r_[vv*np.cos(psi),vv*np.sin(psi),w,pp,acc,vv]
        out[name]=rhs
    return out

def run_all():
    plant_models=plants();out=Path(__file__).parent/'outputs';summary={}
    scenarios=[('left_turn',.1,.4,.7,False),('right_turn',.1,-.4,-.6,False),
               ('reverse_turn',-.1,.4,1.,False),('fixed_heading',.1,0,1.2,False),
               ('turn_disturbance',.1,.4,.7,True)]
    for scenario,v0,w0,heading,pulse in scenarios:
        model=design(v0,w0);K=model['K']
        def ref(t):return np.r_[reference(t,v0,w0,heading),v0*t]
        def error(t,z):return error_transform(heading+w0*t)@(z[:7]-ref(t)[:7])
        def control(t,z):return -K@error(t,z)
        # 扰动定义为轮-车体关节上的附加力矩，使用同一B；不是凭空当作车体外力。
        def disturbance(t):return np.array([.025,.025]) if pulse and 8<=t<8.2 else np.zeros(2)
        z0=ref(0)+np.array([.01,-.01,.01,.03,0,0,0,0.]);runs={}
        for plant_name,plant in plant_models.items():
            def rhs(t,z):return plant(z,np.clip(control(t,z),-TORQUE_LIMIT,TORQUE_LIMIT)+disturbance(t))
            t,z=integrate(rhs,z0,35,stops=(8,8.2) if pulse else ())
            refs=np.array([ref(tt) for tt in t]);raw=np.array([control(tt,zz) for tt,zz in zip(t,z)])
            applied=np.clip(raw,-TORQUE_LIMIT,TORQUE_LIMIT);dist=np.array([disturbance(tt) for tt in t])
            err=z-refs  # CSV是世界误差，控制器使用旋转位置误差；s不在控制目标里。
            info=save_case(out,scenario+'/'+plant_name,t,z,refs,err,raw,applied,dist,LABELS,model,
                {'plant':plant_name,'v0':v0,'omega0':w0,'heading0':heading,
                 'controller_state':'[e_parallel,e_perp,e_psi,e_theta,e_v,e_omega,e_p]',
                 's_is_diagnostic_only':True,'peak_pitch_rad':float(np.max(abs(z[:,3])))})
            assert np.max(abs(err[-1,:7]))<1e-3,(scenario,plant_name,err[-1])
            assert np.max(abs(z[:,3]))<.15,(scenario,plant_name,'pitch outside comparison range')
            summary[scenario+'/'+plant_name]=info;runs[plant_name]=(t,z,refs)
            fig,ax=plt.subplots(figsize=(6,6));ax.plot(z[:,0],z[:,1],label='actual');ax.plot(refs[:,0],refs[:,1],'--',label='reference')
            ax.set(xlabel='x [m]',ylabel='y [m]',title=scenario+' / '+plant_name);ax.axis('equal');ax.grid(True);ax.legend();fig.tight_layout()
            fig.savefig(out/scenario/plant_name/'path.png',dpi=130);plt.close(fig)
        t,z,_=runs['exact'];small=runs['small_pitch'][1]
        comparison={'max_position_difference_m':float(np.max(np.linalg.norm(z[:,:2]-small[:,:2],axis=1))),
                    'max_pitch_difference_rad':float(np.max(abs(z[:,3]-small[:,3])))}
        (out/scenario/'comparison.json').write_text(json.dumps(comparison,indent=2))
        fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
        axes[0].plot(t,z[:,3],label='exact');axes[0].plot(t,small[:,3],'--',label='small pitch');axes[0].set_ylabel('theta [rad]');axes[0].legend()
        axes[1].plot(t,np.linalg.norm(z[:,:2]-small[:,:2],axis=1));axes[1].set_ylabel('position difference [m]');axes[1].set_xlabel('t [s]')
        for ax in axes:ax.grid(True)
        fig.tight_layout();fig.savefig(out/scenario/'comparison.png',dpi=130);plt.close(fig)
    return summary
if __name__=='__main__':print(json.dumps(run_all(),indent=2,default=serial))
