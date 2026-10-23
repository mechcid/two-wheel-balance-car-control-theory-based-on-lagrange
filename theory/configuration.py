"""理论展示和快捷入口共用的配置；不依赖仿真程序。"""
from pathlib import Path
import tomllib
import numpy as np

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/lqr.toml'
_path = DEFAULT_CONFIG


def configure(path):
    """在构建模型前选择配置；切换配置时清除数值缓存。"""
    global _path
    import sys
    _path = Path(path).resolve()
    module = sys.modules.get('turning_balance.lqr')
    if module is not None:
        module.exact_functions.cache_clear()
        module.reference_pitch.cache_clear()


def cost_matrix(value, size, name, positive=False):
    matrix = np.asarray(value, dtype=float)
    if matrix.ndim == 1:
        matrix = np.diag(matrix)
    if matrix.shape != (size, size) or not np.isfinite(matrix).all():
        raise ValueError(f'{name}: 需要有限的 {size}×{size} 矩阵或长度 {size} 的对角列表')
    if not np.allclose(matrix, matrix.T, rtol=0, atol=1e-12):
        raise ValueError(f'{name}: 必须对称')
    eig = np.linalg.eigvalsh(matrix)
    if (eig[0] <= 0 if positive else eig[0] < -1e-12):
        raise ValueError(f'{name}: 必须' + ('正定' if positive else '半正定'))
    return matrix


def load_config():
    with _path.open('rb') as file:
        cfg = tomllib.load(file)
    expected = {'physical', 'design', 'origin_static_balance', 'turning'}
    if set(cfg) != expected:
        raise ValueError(f'配置顶层必须为 {sorted(expected)}')
    p = cfg['physical']
    required = {'r', 'd', 'l', 'm_B', 'm_L', 'm_R', 'g', 'psi0'}
    required |= {f'I_{body}{axis}' for body in ('B', 'L', 'R') for axis in ('xx', 'yy', 'zz')}
    required |= {'I_Bxy', 'I_Bxz', 'I_Byz'}
    if set(p) != required:
        raise ValueError(f'物理参数缺失 {required-set(p)}；未知参数 {set(p)-required}')
    if not all(np.isfinite(v) for v in p.values()):
        raise ValueError('物理参数必须是有限数值')
    if any(p[k] <= 0 for k in ('r', 'd', 'l', 'm_B', 'm_L', 'm_R', 'g')):
        raise ValueError('尺寸、质量和重力加速度必须为正')
    for body in ('B', 'L', 'R'):
        J = np.array([[p[f'I_{body}xx'], p.get(f'I_{body}xy', 0), p.get(f'I_{body}xz', 0)],
                      [p.get(f'I_{body}xy', 0), p[f'I_{body}yy'], p.get(f'I_{body}yz', 0)],
                      [p.get(f'I_{body}xz', 0), p.get(f'I_{body}yz', 0), p[f'I_{body}zz']]])
        eig = np.linalg.eigvalsh(J)
        if eig[0] <= 0 or eig[-1] > eig[:2].sum() + 1e-14:
            raise ValueError(f'刚体 {body} 的惯量不满足正定性或主惯量三角不等式')
    if set(cfg['design']) != {'domain', 'sample_time'}:
        raise ValueError('design 只允许 domain 和 sample_time')
    if cfg['design']['domain'] not in ('continuous', 'discrete'):
        raise ValueError('domain 必须是 continuous 或 discrete')
    dt = cfg['design']['sample_time']
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError('sample_time 必须为正且有限')
    for name, size in [('origin_static_balance', 6), ('turning', 7)]:
        item = cfg[name]
        keys = {'enabled', 'Q', 'R'} | ({'v0', 'omega0'} if name == 'turning' else set())
        if set(item) != keys or type(item['enabled']) is not bool:
            raise ValueError(f'{name}: 配置键应为 {sorted(keys)}，enabled 必须为布尔值')
        cost_matrix(item['Q'], size, name+'.Q')
        cost_matrix(item['R'], 2, name+'.R', positive=True)
    if not np.isfinite([cfg['turning']['v0'], cfg['turning']['omega0']]).all():
        raise ValueError('参考速度必须有限')
    return cfg
