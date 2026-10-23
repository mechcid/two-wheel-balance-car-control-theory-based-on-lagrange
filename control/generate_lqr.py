#!/usr/bin/env python3
"""读取统一配置，调用理论模型，导出可追溯的 LQR 设计。"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
import numpy as np
from scipy.linalg import expm, solve_discrete_are

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'theory'))
from configuration import DEFAULT_CONFIG, configure, load_config, cost_matrix
from parameters import solve_lqr

BALANCE_STATES = ['x', 'psi', 'theta', 'x_dot', 'psi_dot', 'theta_dot']
TURNING_ERRORS = ['e_parallel', 'e_left', 'e_psi', 'e_theta', 'e_v', 'e_omega', 'e_p']


def solve(A, B, Q, R, domain, dt):
    """离散权重表示每拍代价；对连续误差动力学做精确 ZOH 离散化。"""
    if domain == 'continuous':
        result = solve_lqr(A, B, Q, R)
        return dict(A=A, B=B, Q=Q, R=R, P=result['P'], K=result['K'],
                    poles=result['poles'], residual=result['care_residual'])
    n, m = B.shape
    block = np.zeros((n+m, n+m))
    block[:n, :n] = A
    block[:n, n:] = B
    transition = expm(block * dt)
    Ad, Bd = transition[:n, :n], transition[:n, n:]
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    residual = Ad.T @ P @ Ad - P - Ad.T @ P @ Bd @ K + Q
    relative = float(np.linalg.norm(residual, np.inf) / max(1., np.linalg.norm(P, np.inf), np.linalg.norm(Q, np.inf)))
    poles = np.linalg.eigvals(Ad - Bd @ K)
    if not np.isfinite(K).all() or np.max(np.abs(poles)) >= 1 or relative > 1e-8:
        raise ValueError('离散 LQR 未通过稳定性或 DARE 残差检查')
    return dict(A=Ad, B=Bd, Q=Q, R=R, P=P, K=K, poles=poles,
                residual=float(np.linalg.norm(residual, np.inf)), relative_residual=relative)


def generate(cfg):
    from origin_static_balance.lqr import state_matrices as balance_matrices
    from turning_balance.lqr import state_matrices as turn_matrices, trim
    domain, dt = cfg['design']['domain'], cfg['design']['sample_time']
    controllers = {}
    for name, n in [('origin_static_balance', 6), ('turning', 7)]:
        c = cfg[name]
        if not c['enabled']:
            continue
        if name == 'origin_static_balance':
            if cfg['physical']['psi0'] != 0:
                raise ValueError('快捷入口的 origin_static_balance 要求 psi0=0，以保持导出的前向状态约定')
            model = balance_matrices()
            A, B = model['A'], model['B']
            reference = dict(state=[0.] * 6, tau=[0., 0.], note='固定位置直立；可平移 x_ref 和世界原点，当前名义朝向为 0')
            order = BALANCE_STATES
            units = ['m', 'rad', 'rad', 'm/s', 'rad/s', 'rad/s']
        else:
            v0, w0 = c['v0'], c['omega0']
            if abs(v0) < 1e-10 and abs(w0) < 1e-10:
                raise ValueError('静止请启用 origin_static_balance；turning 七状态静止时横向位置不可控')
            _, B, A, _ = turn_matrices(v0, w0)
            ref = trim(v0, w0)
            reference = dict(v0=v0, omega0=w0, theta=ref['theta'], tau=ref['tau'],
                             state_at_t0=[0., 0., 0., ref['theta'], v0, w0, 0.],
                             world_state_order=['x', 'y', 'psi', 'theta', 'v', 'omega', 'p'],
                             trim_residual=ref['residual'])
            order = TURNING_ERRORS
            units = ['m', 'm', 'rad', 'rad', 'm/s', 'rad/s', 'rad/s']
        Q = cost_matrix(c['Q'], n, name+'.Q')
        R = cost_matrix(c['R'], 2, name+'.R', positive=True)
        result = solve(A, B, Q, R, domain, dt)
        poles = result.pop('poles')
        result.update(domain=domain, sample_time=dt if domain == 'discrete' else None,
                      A_continuous=A, B_continuous=B,
                      poles=[dict(real=float(z.real), imag=float(z.imag)) for z in poles],
                      state_error_order=order, state_error_units=units,
                      input_order=['tau_L', 'tau_R'], input_unit='N*m',
                      control_law='tau = tau_ref - K @ error', reference=reference,
                      cost_convention='sum per-step (error.T Q error + delta_tau.T R delta_tau)' if domain == 'discrete' else 'integral (error.T Q error + delta_tau.T R delta_tau) dt',
                      controllability_rank=int(np.linalg.matrix_rank(np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)]))))
        controllers[name] = result
    if not controllers:
        raise ValueError('至少启用一个控制场景')
    return controllers


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG, help='带注释的 TOML 配置')
    parser.add_argument('--output', type=Path, default=ROOT/'control/generated', help='输出目录')
    args = parser.parse_args()
    configure(args.config)
    cfg = load_config()
    controllers = generate(cfg)
    bundle = dict(schema_version=2, generated_at=datetime.now(timezone.utc).isoformat(),
                  config_path=str(args.config.resolve()),
                  config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                  config_snapshot=cfg, controllers=controllers,
                  limitation='局部理想模型；未验证执行器限幅、延迟、传感器、轮地接触或实车。')
    # 全部场景成功后才更新输出；controllers.json 是唯一完整机器接口。
    args.output.mkdir(parents=True, exist_ok=True)
    target = args.output/'controllers.json'
    temporary = target.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(bundle, ensure_ascii=False, indent=2, default=serial, allow_nan=False)+'\n')
    temporary.replace(target)
    lines = ['# LQR 生成结果', '', f"生成时间：{bundle['generated_at']}", '',
             f"设计类型：{cfg['design']['domain']}；配置 SHA256：`{bundle['config_sha256']}`", '',
             '完整矩阵、状态顺序、单位、参考值与配置快照见同目录 controllers.json。', '']
    for name, result in controllers.items():
        lines += [f'## {name}', '', f"采样周期：{result['sample_time']} s（null 表示连续设计）", '',
                  f"误差顺序：`{result['state_error_order']}`", '', 'K =', '', '```text',
                  np.array2string(result['K'], precision=12, max_line_width=160), '```', '',
                  f"Riccati 残差无穷范数：{result['residual']:.3g}", '']
        print(name, 'K =\n', result['K'])
    lines += [bundle['limitation'], '']
    (args.output/'summary.md').write_text('\n'.join(lines))
    print('完整结果：', target.resolve())


if __name__ == '__main__':
    main()
