"""入口回归：连续入口一致、离散采样闭环、完整权重、错误配置及缓存切换。"""
import copy
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'theory'))
spec = importlib.util.spec_from_file_location('generator', ROOT/'control/generate_lqr.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
from configuration import configure, DEFAULT_CONFIG, load_config, cost_matrix


class EntryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        configure(DEFAULT_CONFIG)
        cls.cfg = load_config()
        cls.discrete = generator.generate(cls.cfg)

    def tearDown(self):
        configure(DEFAULT_CONFIG)

    def test_continuous_matches_theory(self):
        from origin_static_balance.lqr import design as origin_static_balance
        from turning_balance.lqr import design as turning
        cfg = copy.deepcopy(self.cfg)
        cfg['design']['domain'] = 'continuous'
        results = generator.generate(cfg)
        # 日常配置允许用户调参；比较同一配置下的两个入口，不绑定旧默认 K。
        for name, theory in [('origin_static_balance', origin_static_balance()), ('turning', turning())]:
            np.testing.assert_allclose(results[name]['K'], theory['K'], rtol=1e-10, atol=1e-10)

    def test_discrete_zoh_against_integration_and_stability(self):
        for result in self.discrete.values():
            A, B = result['A_continuous'], result['B_continuous']
            x = np.linspace(-.01, .01, len(A))
            u = np.array([.001, -.002])
            end = solve_ivp(lambda t,z: A@z+B@u, [0,self.cfg['design']['sample_time']], x,
                            rtol=1e-11, atol=1e-13).y[:,-1]
            np.testing.assert_allclose(result['A']@x+result['B']@u, end, rtol=1e-9, atol=1e-11)
            closed = result['A']-result['B']@result['K']
            self.assertLess(max(abs(np.linalg.eigvals(closed))), 1)
            self.assertLess(np.linalg.norm(np.linalg.matrix_power(closed, 20000)@x), 1e-6)

    def test_full_matrices(self):
        cfg = copy.deepcopy(self.cfg)
        for name in ('origin_static_balance', 'turning'):
            Q = np.diag(cfg[name]['Q'])
            Q[0,1] = Q[1,0] = .2
            cfg[name]['Q'] = Q.tolist()
            cfg[name]['R'] = [[10.,1.], [1.,10.]]
        result = generator.generate(cfg)
        for name in result:
            self.assertFalse(np.allclose(result[name]['K'], self.discrete[name]['K']))

    def test_reject_invalid_weights_and_stationary_turn(self):
        with self.assertRaises(ValueError):
            cost_matrix([[1,2],[0,1]], 2, 'Q')
        with self.assertRaises(ValueError):
            cost_matrix([0,1], 2, 'R', positive=True)
        cfg = copy.deepcopy(self.cfg)
        cfg['turning'].update(v0=0., omega0=0.)
        with self.assertRaisesRegex(ValueError, '不可控'):
            generator.generate(cfg)

    def test_config_switch_refreshes_numeric_cache(self):
        from turning_balance.lqr import exact_functions, reference
        original = exact_functions()[0](0.)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'changed.toml'
            path.write_text(DEFAULT_CONFIG.read_text().replace('m_B = 0.831', 'm_B = 1.1').replace('v0 = 0.1', 'v0 = 0.2'))
            configure(path)
            self.assertFalse(np.allclose(original, exact_functions()[0](0.)))
            self.assertEqual(reference(0)[4], .2)
            configure(DEFAULT_CONFIG)
            np.testing.assert_allclose(original, exact_functions()[0](0.))


if __name__ == '__main__':
    unittest.main()
