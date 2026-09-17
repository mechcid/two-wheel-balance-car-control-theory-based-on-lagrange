"""Read actual ROS logger data; plot states/torques and report explicit criteria.
Use project .venv Python (matplotlib), not necessarily ROS Python.
"""
import argparse,json,os
os.environ.setdefault('MPLCONFIGDIR','/tmp/balance_mpl')
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('csv',type=Path); args=ap.parse_args()
    a=np.genfromtxt(args.csv,delimiter=',',names=True)
    if a.size<20: raise ValueError('Not enough actual control samples')
    fig,axes=plt.subplots(3,2,figsize=(11,8),sharex=True)
    for ax,k,u in zip(axes.flat,['x','xdot','psi','psidot','theta','thetadot'],['m','m/s','rad','rad/s','rad','rad/s']):
        ax.plot(a['t'],a[k]);
        if np.max(np.abs(a[k])) < 1e-4: ax.set_ylim(-1e-4,1e-4)
        if args.csv.stem=='push': ax.axvspan(3,3.1,color='orange',alpha=.25)
        ax.set_ylabel(f'{k} [{u}]'); ax.grid(); ax.set_xlabel('Simulation time [s]')
    fig.suptitle(f'Gazebo physical simulation: {args.csv.stem}'); fig.tight_layout(); fig.savefig(args.csv.with_suffix('.states.png')); plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,4))
    for k in ['tau_L','tau_R']: ax.plot(a['t'],a[k],label=k)
    ax.set(xlabel='Simulation time [s]',ylabel='Commanded torque [N m]'); ax.grid(); ax.legend(); fig.tight_layout(); fig.savefig(args.csv.with_suffix('.torques.png')); plt.close(fig)
    tail=a[a['t']>a['t'][-1]-2]
    bounds=dict(x=.01,psi=.01,theta=.005,xdot=.02,psidot=.02,thetadot=.02)
    metrics={'source':'Gazebo ROS CSV, not linear solve_ivp','duration':float(a['t'][-1]),'tail_max_abs':{k:float(max(abs(tail[k]))) for k in bounds},'limits':bounds,'saturation_fraction':float(a['saturated'].mean()),'peak_torque':float(max(max(abs(a['tau_L'])),max(abs(a['tau_R'])))), 'max_sample_gap':float(max(np.diff(a['t']))), 'max_torque_step':float(max(max(abs(np.diff(a['tau_L']))),max(abs(np.diff(a['tau_R'])))))}
    metrics['converged']=bool(metrics['duration']>=14.9 and all(metrics['tail_max_abs'][k]<v for k,v in bounds.items()))
    args.csv.with_suffix('.metrics.json').write_text(json.dumps(metrics,indent=2)); print(json.dumps(metrics,indent=2))
if __name__=='__main__': main()
