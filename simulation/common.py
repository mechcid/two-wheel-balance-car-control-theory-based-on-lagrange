"""两种场景共用的积分、CSV/JSON导出和绘图。仅数学仿真，不写 ROS 配置。"""
from pathlib import Path
import sys,os,json
import numpy as np
from scipy.integrate import solve_ivp
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'theory'))
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'simulation/outputs/.mplconfig'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
DT=.02
TORQUE_LIMIT=1.0  # 每个电机 N m；控制律的原始输出和限幅输出都记录。

def integrate(rhs,z0,duration,stops=(),event=None):
    """在外部扰动起止时刻分段，防止积分器跨过短脉冲。"""
    times=np.linspace(0,duration,round(duration/DT)+1);chunks=[];current=np.asarray(z0,float)
    for a,b in zip([0,*stops],[*stops,duration]):
        ts=times[(times>=a-1e-10)&(times<=b+1e-10)]
        ts=np.unique(np.r_[a,ts,b]);ts=np.clip(ts,a,b)
        sol=solve_ivp(rhs,(a,b),current,t_eval=ts,events=event,rtol=1e-9,atol=1e-11,max_step=DT)
        if not sol.success:raise RuntimeError(sol.message)
        chunk=np.column_stack((sol.t,sol.y.T))
        chunks.append(chunk if not chunks else chunk[1:]);current=sol.y[:,-1]
        if sol.status==1:break
    data=np.vstack(chunks)
    if not np.isfinite(data).all():raise ValueError('积分出现非有限数值')
    return data[:,0],data[:,1:]

def serial(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)

def save_case(out,name,t,z,ref,errors,raw,applied,disturbance,labels,design,extra=None):
    out=Path(out)/name;out.mkdir(parents=True,exist_ok=True)
    from parameters import values
    peak=np.max(abs(applied),axis=0)
    info={'parameters':values()[0],'torque_limit_Nm':TORQUE_LIMIT,'feedback':'continuous','output_step_s':DT,'samples':len(t),'end_time':float(t[-1]),'final_error':errors[-1].tolist(),
          'tail_max_errors':np.max(abs(errors[t>=t[-1]-min(2,t[-1])]),axis=0).tolist(),
          'peak_motor_torque_Nm':peak.tolist(),'saturation_fraction':float(np.mean(abs(raw)>TORQUE_LIMIT)),
          'rank':design['rank'],'closed_poles':[[x.real,x.imag] for x in design['poles']],
          'notice':'临时示例参数；数学模型仿真，不是 Gazebo 或实车验收。'}
    if extra:info.update(extra)
    head=['t']+labels+[v+'_ref' for v in labels]+['error_'+v for v in labels]+['tau_L_raw','tau_R_raw','tau_L','tau_R','dist_L','dist_R']
    np.savetxt(out/'trajectory.csv',np.column_stack([t,z,ref,errors,raw,applied,disturbance]),delimiter=',',header=','.join(head),comments='')
    matrices={k:design[k] for k in ('A','B','Q','R','P','K')}
    (out/'result.json').write_text(json.dumps({**info,'controller_matrices':matrices},indent=2,default=serial))
    fig,axes=plt.subplots(len(labels),1,figsize=(10,2*len(labels)),sharex=True)
    for i,ax in enumerate(axes):
        ax.plot(t,z[:,i],label='actual');ax.plot(t,ref[:,i],'--',label='reference');ax.set_ylabel(labels[i]+' ['+('m' if labels[i] in ('x','y','s') else 'm/s' if labels[i] in ('v','dx') else 'rad' if labels[i] in ('psi','theta') else 'rad/s')+']');ax.grid(True);ax.legend(loc='upper right')
    axes[-1].set_xlabel('t [s]');fig.tight_layout();fig.savefig(out/'states.png',dpi=130);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for i,ax in enumerate(axes):
        ax.plot(t,raw[:,i],label='requested');ax.plot(t,applied[:,i],'--',label='motor applied');ax.plot(t,disturbance[:,i],':',label='external joint torque')
        ax.axhline(TORQUE_LIMIT,color='gray',lw=.5);ax.axhline(-TORQUE_LIMIT,color='gray',lw=.5)
        ax.set_ylabel(['left [Nm]','right [Nm]'][i]);ax.grid(True);ax.legend()
    axes[-1].set_xlabel('t [s]');fig.tight_layout();fig.savefig(out/'torques.png',dpi=130);plt.close(fig)
    return info
