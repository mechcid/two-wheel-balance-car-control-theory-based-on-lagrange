"""Aggregate physical acceptance evidence from CSV, preserving failed contact comparison."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parents[3]
out=root/'results'
summary={}
for case in ['upright','pitch','push','yaw']:
    a=np.genfromtxt(out/f'{case}.csv',delimiter=',',names=True)
    metrics=json.loads((out/f'{case}.metrics.json').read_text())
    metrics['peak_pitch']=float(max(abs(a['theta'])))
    metrics['peak_yaw']=float(max(abs(a['psi'])))
    metrics['tail_torque_rms']=float(np.sqrt(np.mean(np.concatenate([a['tau_L'][a['t']>14],a['tau_R'][a['t']>14]])**2)))
    metrics['initial_state']=[float(a[k][0]) for k in ['x','psi','theta','xdot','psidot','thetadot']]
    metrics['actual_mean_rate_hz']=float(1/np.mean(np.diff(a['t'])))
    summary[case]=metrics
probes={}
for side in ['left','right']:
    a=np.genfromtxt(out/f'{side}_probe.csv',delimiter=',',names=True)
    # Fit initial 10--40 ms velocities to avoid first-frame startup latency.
    b=a[(a['t']>=.01)&(a['t']<=.04)]
    assert len(b)>=4
    slopes={k:float(np.polyfit(b['t'],b[k],1)[0]) for k in ['xdot','psidot','thetadot']}
    assert slopes['xdot']>0 and slopes['thetadot']<0
    assert slopes['psidot']*(-1 if side=='left' else 1)>0
    np.testing.assert_allclose(a['tau_L'],.1 if side=='left' else 0)
    np.testing.assert_allclose(a['tau_R'],.1 if side=='right' else 0)
    probes[side]=slopes
summary['independent_torque_probes']=probes
# One combined physical response plot for quick review; raw data remain unchanged.
fig,axes=plt.subplots(4,2,figsize=(12,11),sharex=True)
for row,case in enumerate(['upright','pitch','push','yaw']):
    a=np.genfromtxt(out/f'{case}.csv',delimiter=',',names=True)
    axes[row,0].plot(a['t'],a['theta'],label='pitch theta [rad]')
    axes[row,0].plot(a['t'],a['psi'],label='yaw psi [rad]')
    if max(max(abs(a['theta'])),max(abs(a['psi'])))<1e-4: axes[row,0].set_ylim(-1e-4,1e-4)
    axes[row,1].plot(a['t'],a['tau_L'],label='left torque')
    axes[row,1].plot(a['t'],a['tau_R'],'--',label='right torque')
    if max(max(abs(a['tau_L'])),max(abs(a['tau_R'])))<1e-4: axes[row,1].set_ylim(-1e-4,1e-4)
    for ax in axes[row]:
        ax.set_title(case); ax.grid(); ax.legend(loc='upper right'); ax.set_xlabel('Simulation time [s]')
        if case=='push': ax.axvspan(3,3.1,color='orange',alpha=.25)
    axes[row,0].set_ylabel('Angle [rad]'); axes[row,1].set_ylabel('Torque [N m]')
fig.suptitle('Gazebo physical simulation - unchanged LQR K - ideal narrow-wheel contact')
fig.tight_layout();fig.savefig(out/'acceptance.png',dpi=150);plt.close(fig)
(out/'acceptance.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
assert all(summary[k]['converged'] for k in ['upright','pitch','push','yaw'])
