import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"theory"))
"""拉格朗日求导、虚功映射及约束消元检查。"""
import contextlib, io
with contextlib.redirect_stdout(io.StringIO()):
    from sympy_scaffold import *
import numpy as np
assert (A*S).applyfunc(sm.trigsimp) == sm.zeros(3)
assert (M_reduced-M_reduced.T).applyfunc(sm.trigsimp) == sm.zeros(3)
expected = sm.Matrix([(tau_L+tau_R)/(r*sm.cos(psi)), d*(tau_R-tau_L)/(2*r), -tau_L-tau_R])
assert (Q_reduced-expected).applyfunc(sm.simplify) == sm.zeros(3,1)
# 以独立的 LagrangesMethod 自动求导核对逐项手写算子。
lm = me.LagrangesMethod(lagrangian, list(q))
assert (lm.form_lagranges_equations()-EL).applyfunc(sm.trigsimp) == sm.zeros(6,1)
# 非轴对称、左右不等参数：9×9乘子解与3×3约化解对比。
params={psi_0:.2,r:.2,d:.5,l:.4,g:9.81,m_L:.4,m_R:.65,m_B:2.1,
 I_Lxx:.003,I_Lyy:.004,I_Lzz:.0035,I_Rxx:.004,I_Ryy:.005,I_Rzz:.0045,
 I_Bxx:.1,I_Byy:.12,I_Bzz:.14}
args=list(q)+list(ind)+[tau_L,tau_R]
def fun(e):
    return sm.lambdify(args, e.subs(dep, simultaneous=True).subs(params), 'numpy', cse=True)
fM,ff,fMr,fhr,fQr=map(fun,[augmented_matrix,augmented_rhs,M_reduced,h_reduced,Q_reduced])
fS,fb=map(fun,[S,bias])
rng=np.random.default_rng(24)
err=0.
for _ in range(8):
    vals=rng.uniform(-.7,.7,len(args))
    full=np.linalg.solve(fM(*vals),ff(*vals)).ravel()[:6]
    red=np.linalg.solve(fMr(*vals),(fQr(*vals)-fhr(*vals))).ravel()
    restored=np.asarray(fS(*vals))@red+np.asarray(fb(*vals)).ravel()
    err=max(err,float(np.max(np.abs(full-restored))))
assert err < 1e-9, err
print('PASS: 拉格朗日算子、约束零空间、质量矩阵对称性、电机虚功映射')
print('PASS: 8组非轴对称参数状态下乘子/约化加速度最大差 =',err)

assert lagrangian == sm.trigsimp(L_quadratic)
assert (M_second-M_projected).applyfunc(sm.simplify)==sm.zeros(3)
assert (h_second-h_projected).applyfunc(sm.simplify)==sm.zeros(3,1)
assert (Q_second-Q_projected).applyfunc(sm.simplify)==sm.zeros(3,1)
assert C_linear.applyfunc(sm.simplify)==sm.zeros(3)
assert (K_linear-sm.diag(0,0,-m_B*g*l)).applyfunc(sm.simplify)==sm.zeros(3)
print("PASS: 近似能量主线、两分支符号一致、局部一阶模型")
