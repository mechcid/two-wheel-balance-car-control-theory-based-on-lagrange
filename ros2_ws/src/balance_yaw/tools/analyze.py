"""Compare actual Gazebo travel/yaw with moving references; plot world path separately."""
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/balance_yaw_mpl')
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def analyze(path):
    a=np.genfromtxt(path,delimiter=',',names=True)
    names=['l','psi','theta','v','omega','p'];units=['m','rad','rad','m/s','rad/s','rad/s']
    errors=np.column_stack([a[n]-a[n+'_ref'] for n in names]);tail=a['t']>29;cruise=(a['t']>8)&(a['t']<19)
    tol=np.array([.01,.02,.01,.02,.03,.03])
    metrics={'duration':float(a['t'][-1]),'end_max_errors':dict(zip(names,np.max(abs(errors[tail]),axis=0).tolist())),
             'cruise_max_errors':dict(zip(names,np.max(abs(errors[cruise]),axis=0).tolist())),
             'peak_pitch':float(max(abs(a['theta']))),'peak_roll':float(max(abs(a['roll']))),
             'peak_torque':float(max(max(abs(a['tau_L'])),max(abs(a['tau_R'])))),
             'saturation_fraction':float(a['saturated'].mean()),'yaw_travel':float(a['psi'][-1]-a['psi'][0]),
             'max_sample_gap':float(max(np.diff(a['t'])))}
    metrics['passed']=bool(a['t'][-1]>31 and np.all(np.max(abs(errors[tail]),axis=0)<tol) and metrics['peak_pitch']<.12 and metrics['peak_roll']<.1 and metrics['saturation_fraction']<.01 and max(abs(errors[cruise,4]))<.06)
    fig,axes=plt.subplots(3,2,figsize=(12,8),sharex=True)
    for ax,n,unit in zip(axes.flat,names,units):
        ax.plot(a['t'],a[n],label='Gazebo');ax.plot(a['t'],a[n+'_ref'],'--',label='reference');ax.set_ylabel(f'{n} [{unit}]');ax.grid();ax.legend()
        if path.stem=='turn_push':ax.axvspan(12,12.1,color='orange',alpha=.3)
    for ax in axes[-1]:ax.set_xlabel('Simulation time [s]')
    fig.suptitle('Moving yaw: l = forward travel (not world x)');fig.tight_layout();fig.savefig(path.with_suffix('.states.png'));plt.close(fig)
    fig,ax=plt.subplots(1,2,figsize=(12,4.5))
    # Integrate commanded speed and heading solely to show the nominal path.
    dt=np.diff(a['t'],prepend=a['t'][0]);xref=np.cumsum(a['v_ref']*np.cos(a['psi_ref'])*dt);yref=np.cumsum(a['v_ref']*np.sin(a['psi_ref'])*dt)
    ax[0].plot(a['x_world'],a['y_world'],label='Gazebo path');ax[0].plot(xref,yref,'--',label='kinematic reference');ax[0].set(xlabel='world x [m]',ylabel='world y [m]');ax[0].axis('equal')
    ax[1].plot(a['t'],a['tau_L'],label='left');ax[1].plot(a['t'],a['tau_R'],label='right');ax[1].set(xlabel='time [s]',ylabel='torque [N m]')
    for z in ax:z.legend();z.grid()
    fig.tight_layout();fig.savefig(path.with_suffix('.path_torque.png'));plt.close(fig)
    metrics['final_xy_reference_offset_m']=float(np.hypot(a['x_world'][-1]-xref[-1],a['y_world'][-1]-yref[-1]))
    path.with_suffix('.metrics.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2));return metrics
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('csv',type=Path);arg=ap.parse_args();m=analyze(arg.csv)
    if not m['passed']:raise SystemExit('Acceptance failed')
