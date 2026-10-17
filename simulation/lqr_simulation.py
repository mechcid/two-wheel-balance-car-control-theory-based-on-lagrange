"""Local upright balance-car simulation. Run from any cwd; outputs beside this file.
Matrix source: theory/straight_balance/lagrange.py, M_linear/C_linear/K_linear/B_linear.
World-x convention; psi0=0 aligns world x with nominal forward direction.
No saturation/nonlinear plant/hardware fidelity is claimed.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import json
import importlib.util
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are, expm
import os
# Keep plotting cache inside the project, not the system/user environment.
os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / "outputs" / ".mplconfig"))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ===== Editable parameters: DEMONSTRATION ONLY, not measured robot parameters =====
# dataclass 将一组带名字的数值组织成参数对象；frozen=True 防止运行中意外改值。
@dataclass(frozen=True)
class Parameters:
    r: float = 0.10                 # m
    d: float = 0.40                 # m, full track width
    l: float = 0.30                 # m, axle to body COM
    m_B: float = 2.0                # kg
    m_L: float = 0.50
    m_R: float = 0.50
    I_Lyy: float = 0.0025           # kg m^2, wheel axle moment
    I_Ryy: float = 0.0025
    I_Lzz: float = 0.00125          # wheel diameter moment at reference orientation
    I_Rzz: float = 0.00125
    I_Byy: float = 0.080
    I_Bzz: float = 0.060
    I_Bxy: float = 0.0
    I_Bxz: float = 0.0
    I_Byz: float = 0.0
    g: float = 9.81
    psi0: float = 0.0               # rad; world-x chart requires cos(psi0) != 0

@dataclass(frozen=True)
class Settings:
    # 仿真实验配置，与真实机器人参数分开：权重、初值、时长、积分精度。
    q_weights: tuple = (10., 5., 400., 5., 2., 40.)  # x,psi,theta,xdot,psidot,thetadot
    r_weights: tuple = (10., 10.)                    # tau_L,tau_R
    initial_state: tuple = (0., 0., 0.03, 0., 0., 0.)
    open_duration: float = 3.
    closed_duration: float = 15.
    open_pitch_limit: float = 0.20   # reporting cutoff, not a physical hard limit
    rtol: float = 1e-9
    atol: float = 1e-11

# 在启动时建立默认参数；修改类中的默认值后重新运行程序即可生效。
PARAMS = Parameters()
CONFIG = Settings()
OUTPUT = Path(__file__).resolve().parent / 'outputs'
LABELS = ['x [m]', 'psi [rad]', 'theta [rad]',
          'xdot [m/s]', 'psidot [rad/s]', 'thetadot [rad/s]']

def build_M0(p=PARAMS):
    """把质量、尺寸和惯量代入符号推导的最终公式，返回 3×3 质量矩阵。
    不重新推导动力学。这里仍是世界 x，psi0=0 时与名义前进方向一致。"""
    for name, value in asdict(p).items():
        if not np.isfinite(value) or (name not in ('psi0','I_Bxy','I_Bxz','I_Byz') and value <= 0):
            raise ValueError(f'Invalid parameter {name}={value}')
    c = np.cos(p.psi0)
    if abs(c) < 1e-6:
        raise ValueError('World-x parameterization singular near cos(psi0)=0')
    wheel_mass = (p.I_Lyy + p.I_Ryy)/p.r**2
    coupling = p.d/2*((p.I_Ryy-p.I_Lyy)/p.r**2 + p.m_R-p.m_L)
    return np.array([
        [(p.m_B+p.m_L+p.m_R+wheel_mass)/c**2, coupling/c, p.l*p.m_B/c],
        [coupling/c, p.I_Bzz+p.I_Lzz+p.I_Rzz + p.d**2/4*(wheel_mass+p.m_L+p.m_R), p.I_Byz],
        [p.l*p.m_B/c, p.I_Byz, p.I_Byy+p.l**2*p.m_B]])

def build_state_space(p=PARAMS):
    """将 M0*zdd+C0*zd+K0*z=B0*u 转成 Xdot=A*X+B*u。
    返回 M0,C0,K0,B0（机械方程）及 A(6×6),B(6×2)（状态方程）。"""
    M0 = build_M0(p)
    C0 = np.zeros((3,3))
    K0 = np.diag([0., 0., -p.g*p.l*p.m_B])
    B0 = np.array([[1/(p.r*np.cos(p.psi0))]*2,
                   [-p.d/(2*p.r), p.d/(2*p.r)], [-1., -1.]])
    # solve(M0,K0) 解的是 M0*Y=K0，避免显式构造 inv(M0)。
    # 上半块负责“位置导数=速度”，下半块负责“速度导数=加速度”。
    A = np.block([[np.zeros((3,3)), np.eye(3)],
                  [-np.linalg.solve(M0,K0), -np.linalg.solve(M0,C0)]])
    B = np.vstack([np.zeros((3,2)), np.linalg.solve(M0,B0)])
    return M0,C0,K0,B0,A,B

def check_system(M0,A,B):
    """积分前检查矩阵是否能用于计算，并检查开环模态与可控性。
    返回诊断字典、开环特征值、6×12 可控性矩阵；发现问题会停止。"""
    if M0.shape!=(3,3) or A.shape!=(6,6) or B.shape!=(6,2):
        raise ValueError('Unexpected matrix dimensions')
    if not all(np.isfinite(v).all() for v in (M0,A,B)):
        raise ValueError('Non-finite matrix')
    np.testing.assert_allclose(M0,M0.T,atol=1e-12)
    np.linalg.cholesky(M0)  # positive definite => invertible
    eig = np.linalg.eigvals(A)
    # [B, A@B, ..., A^5@B]：6状态、2输入，共6×12；@ 表示矩阵乘法。
    blocks=[B]
    for _ in range(5): blocks.append(A@blocks[-1])
    ctrb=np.hstack(blocks)
    rank=int(np.linalg.matrix_rank(ctrb))
    info=dict(mass_rank=int(np.linalg.matrix_rank(M0)),
              mass_determinant=float(np.linalg.det(M0)),
              mass_condition=float(np.linalg.cond(M0)),
              mass_eigenvalues=np.linalg.eigvalsh(M0).tolist(),
              controllability_rank=rank,
              controllability_singular_values=np.linalg.svd(ctrb,compute_uv=False).tolist(),
              open_unstable=bool(np.any(eig.real>1e-9)))
    if rank != 6: raise ValueError(f'Controllability rank {rank}/6')
    return info,eig,ctrb

def integrate(F,X0,duration,cfg,event=None):
    """公共积分器：给定常矩阵 F，数值求解 Xdot=F*X。
    返回时间数组(1001,)、状态历史(6,1001)、与矩阵指数的误差、是否触发终止事件。"""
    # 求解器反复调用右端函数得到当前导数，再自适应选择内部积分步长。
    # max_step=0.02 是内部步长上限，不是控制器采样周期。
    sol=solve_ivp(lambda t,X:F@X,(0.,duration),np.asarray(X0,dtype=float),
                  method='DOP853',dense_output=True,events=event,
                  rtol=cfg.rtol,atol=cfg.atol,max_step=0.02)
    if not sol.success: raise RuntimeError(sol.message)
    # dense_output 保存连续插值；以下1001点仅用于出图/导出，不是积分步数。
    t=np.linspace(0.,sol.t[-1],1001)
    X=sol.sol(t)
    if not np.isfinite(X).all(): raise FloatingPointError('Integration produced non-finite values')
    # Independent constant-matrix exponential check of the IVP integration.
    error=max(np.max(np.abs(X[:,j]-expm(F*t[j])@X[:,0])) for j in (0,250,500,750,1000))
    if error > 1e-7: raise AssertionError(f'Integration/expm mismatch {error}')
    return t,X,float(error),bool(sol.status==1)

def simulate_open_loop(A,cfg=CONFIG):
    """无控制实验：u=0，从小俯仰扰动开始求解 Xdot=A*X。
    俯仰达到展示阈值后提前结束，防止把局部模型外推至大角度。"""
    def limit(t,X): return cfg.open_pitch_limit-abs(X[2])
    # 事件值从正变负表示 |theta| 穿过阈值；求解器定位交点并停止。
    limit.terminal=True
    limit.direction=-1
    if abs(cfg.initial_state[2]) >= cfg.open_pitch_limit: raise ValueError('Initial pitch exceeds cutoff')
    return integrate(A,cfg.initial_state,cfg.open_duration,cfg,limit)

def design_lqr(A,B,cfg=CONFIG):
    """由 Q/R 权重计算常数反馈增益 K，并验证 Riccati 残差和闭环极点。
    返回 K(2×6),P(6×6),Q,R、闭环极点、残差、求解后端名称。"""
    # 最小化 integral(X.T@Q@X + u.T@R@u) dt；不是逐步试力矩或机器学习。
    Q=np.diag(cfg.q_weights); R=np.diag(cfg.r_weights)
    if Q.shape!=(6,6) or R.shape!=(2,2): raise ValueError('Q/R dimension mismatch')
    if min(cfg.q_weights)<0 or min(cfg.r_weights)<=0: raise ValueError('Q>=0 and R>0 required')
    if importlib.util.find_spec('control'):
        import control
        gain,P,_=control.lqr(A,B,Q,R)
        gain=np.asarray(gain); P=np.asarray(P); backend='python-control'
    else:
        # CARE: A.T@P+P@A-P@B@inv(R)@B.T@P+Q=0。
        # P是最优价值函数矩阵；反馈增益 gain=solve(R,B.T@P)。
        P=solve_continuous_are(A,B,Q,R)
        gain=np.linalg.solve(R,B.T@P); backend='scipy.linalg.solve_continuous_are'
    residual=A.T@P+P@A-P@B@np.linalg.solve(R,B.T@P)+Q
    relative=np.linalg.norm(residual)/max(1.,np.linalg.norm(Q))
    poles=np.linalg.eigvals(A-B@gain)
    if relative>1e-7 or not np.all(poles.real < -1e-8): raise AssertionError('LQR validation failed')
    return gain,P,Q,R,poles,float(relative),backend

def simulate_closed_loop(A,B,gain,cfg=CONFIG):
    """闭环实验：将理想连续反馈 u=-K*X 代回，积分 Xdot=(A-B*K)*X。
    返回时间、六状态历史、两轮力矩历史(2,1001)、积分核对误差。"""
    t,X,error,_=integrate(A-B@gain,cfg.initial_state,cfg.closed_duration,cfg)
    torque=-gain@X  # unsaturated ideal torque input; no hardware rating assumed
    # 这里只是当前示例的末时刻数值检查，非全程收敛/物理精度的定义；各状态单位不同。
    if np.max(np.abs(X[:,-1]))>1e-4: raise AssertionError('State did not settle within horizon')
    return t,X,torque,error

def plot_states(t,X,title,path):
    """把六状态画在三行两列中：左列位置/角度，右列对应速度。
    保存 PNG；不改变仿真数据。接近浮点零的曲线不放大到1e-16刻度。"""
    fig,axs=plt.subplots(3,2,figsize=(11,8),sharex=True)
    for row in range(3):
        for col in range(2):
            i=row+3*col
            axs[row,col].plot(t,X[i],lw=1.8)
            axs[row,col].set_ylabel(LABELS[i]);axs[row,col].grid(alpha=.3)
            if np.max(np.abs(X[i])) < 1e-12:
                axs[row,col].set_ylim(-0.001,0.001)
                axs[row,col].text(.5,.85,'Zero to numerical precision',ha='center',transform=axs[row,col].transAxes)
    for ax in axs[-1]: ax.set_xlabel('Time [s]')
    fig.suptitle(title+'\nDEMO parameters; local linear model')
    fig.tight_layout();fig.savefig(path,dpi=160);plt.close(fig)

def complex_list(values):
    """把复数特征值保存成 [实部,虚部]，便于写入 JSON。"""
    return [[float(z.real),float(z.imag)] for z in values]

def main():
    """运行完整流程：模型→检查→开环→LQR→闭环→验证→图表与数据文件。"""
    OUTPUT.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.size':10})
    M0,C0,K0,B0,A,B=build_state_space()
    info,open_poles,ctrb=check_system(M0,A,B)
    if not info['open_unstable']: raise AssertionError('Expected inverted-pendulum unstable mode')
    to,Xo,eo,cutoff=simulate_open_loop(A)
    if abs(Xo[2,-1])<=abs(CONFIG.initial_state[2]): raise AssertionError('Open pitch did not grow')
    gain,P,Q,R,closed_poles,care_error,backend=design_lqr(A,B)
    tc,Xc,u,ec=simulate_closed_loop(A,B,gain)
    # Tilt-only symmetric run does not excite yaw: test a mixed initial perturbation as well.
    mixed=(.02,.03,.02,0.,0.,0.)
    _,Xm,em,_=integrate(A-B@gain,mixed,CONFIG.closed_duration,CONFIG)
    assert np.max(np.abs(Xm[:,-1]))<1e-4
    plot_states(to,Xo,'Open loop: zero torque; stopped at pitch cutoff',OUTPUT/'open_loop.png')
    plot_states(tc,Xc,'Closed loop: continuous-time LQR',OUTPUT/'closed_loop.png')
    fig,ax=plt.subplots(figsize=(10,4))
    ax.plot(tc,u[0],label='tau_L');ax.plot(tc,u[1],'--',label='tau_R')
    ax.set(xlabel='Time [s]',ylabel='Torque [N m]',title='LQR torque (unsaturated, DEMO parameters)')
    ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(OUTPUT/'torques.png',dpi=160);plt.close(fig)
    # CSV 一行一个时间点；X.T 将“每行一个状态”转为“每行一个时刻”。
    np.savetxt(OUTPUT/'open_loop.csv',np.column_stack([to,Xo.T]),delimiter=',',header='t,x,psi,theta,xdot,psidot,thetadot',comments='')
    np.savetxt(OUTPUT/'closed_loop.csv',np.column_stack([tc,Xc.T,u.T]),delimiter=',',header='t,x,psi,theta,xdot,psidot,thetadot,tau_L,tau_R',comments='')
    matrices={k:v.tolist() for k,v in dict(M0=M0,C0=C0,K0=K0,B0=B0,A=A,B=B,K_LQR=gain,Q=Q,R=R,P=P,controllability_matrix=ctrb).items()}
    metrics=dict(info,open_poles=complex_list(open_poles),closed_poles=complex_list(closed_poles),
        open_final_time=float(to[-1]),open_stopped_at_cutoff=cutoff,open_final_pitch=float(Xo[2,-1]),
        closed_final_state=Xc[:,-1].tolist(),peak_abs_torque=np.max(np.abs(u),axis=1).tolist(),
        peak_abs_state=np.max(np.abs(Xc),axis=1).tolist(),care_relative_residual=care_error,
        integration_expm_error=max(eo,ec,em),mixed_final_state=Xm[:,-1].tolist(),lqr_backend=backend)
    report=dict(notice='DEMO ONLY: these are NOT actual robot parameters.',coordinate='World x; psi0=0 equals nominal forward direction',parameters=asdict(PARAMS),settings=asdict(CONFIG),matrices=matrices,metrics=metrics)
    (OUTPUT/'results.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    lines=['# LQR numerical validation','',report['notice'],'',
           'Source: theory/straight_balance/lagrange.py final matrices. World x convention. psi0=0.',
           'No saturation, delay, nonlinear plant, or measured actuator limits modeled.','',
           '## Parameters','```json',json.dumps(asdict(PARAMS),indent=2),'```']
    for name in ['M0','C0','K0','B0','A','B','K_LQR','Q','R']:
        lines += ['',f'## {name}','```text',np.array2string(np.asarray(matrices[name]),precision=8,suppress_small=True,max_line_width=120),'```']
    lines += ['', '## Diagnostics','```json',json.dumps(metrics,indent=2),'```',
              '', 'Open-loop divergence is truncated at |theta|=0.2 rad as a reporting cutoff.',
              'Closed-loop stability and convergence are verified only for this ideal local linear model.',
              'Peak torques are reported; hardware feasibility requires real motor ratings and real parameters.',
              'Mass condition number depends on coordinate scaling/units; it is not a hardware quality measure.']
    (OUTPUT/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
    print(report['notice'])
    for key in ['A','B','K_LQR']: print(key,'\n',np.asarray(matrices[key]))
    print(json.dumps(metrics,indent=2))
    print('Outputs:',OUTPUT)

# 直接运行本文件才执行 main；其他验证脚本 import 它时不会自动开始仿真。
if __name__=='__main__': main()
