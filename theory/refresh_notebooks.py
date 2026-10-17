#!/usr/bin/env python3
"""一键生成并执行五本笔记本。源码 .py 是唯一计算源，ipynb 是可重建展示。
默认使用当前 Python 启动临时 kernel，无需安装新环境或注册 kernel。
仅当全部执行成功后才替换正式 ipynb；--no-execute 只生成、不验证。
"""
from pathlib import Path
import argparse,json,os,tempfile
import nbformat as nbf
from nbclient import NotebookClient
ROOT=Path(__file__).resolve().parent
md=nbf.v4.new_markdown_cell
code=nbf.v4.new_code_cell
SETUP=r'''from pathlib import Path
import sys
root=Path.cwd()
while not (root/'base.py').exists():
    if root==root.parent: raise RuntimeError('找不到 theory/base.py')
    root=root.parent
sys.path.insert(0,str(root))
import sympy as s
import numpy as np
from IPython.display import display, Math
# 只改 LaTeX 显示，不替换计算符号：dx -> x 上点，ddx -> x 双点。
from base import q, qd, qdd
latex_names = {}
for coordinate, velocity, acceleration in zip(q, qd, qdd):
    name = s.latex(coordinate)
    latex_names[velocity] = r'\dot{' + name + '}'
    latex_names[acceleration] = r'\ddot{' + name + '}'
s.init_printing(symbol_names=latex_names)
np.set_printoptions(precision=8,suppress=True,linewidth=130)
def show(label, expression):
    display(Math(label+' = '+s.latex(expression, symbol_names=latex_names)))
'''
COST=r'''## 从性能指标到最终增益
下列 $E$ 表示所选线性误差状态（静止分支就是 $X$，转向分支是旋转坐标中的七维误差）。

$$\dot E=A_eE+B_e\delta\tau,\qquad
J=\int_0^\infty(E^TQE+\delta\tau^TR\delta\tau)\,dt.$$

$Q\succeq0$ 惩罚各状态偏差，$R\succ0$ 惩罚电机力矩。权重有量纲依赖，不是车辆物理参数。
在可稳定、可检测的线性问题上，设最小剩余代价 $V(E)=E^TPE$。

$$0=\min_{\delta\tau}\{E^TQE+\delta\tau^TR\delta\tau+2E^TP(A_eE+B_e\delta\tau)\}.$$

对力矩求导：$2R\delta\tau+2B_e^TPE=0$，故

$$\delta\tau_*=-R^{-1}B_e^TPE=-K_e E.$$

代回并比较二次型系数，得到 CARE：

$$A_e^TP+PA_e-PB_eR^{-1}B_e^TP+Q=0.$$

选择使 $A_e-B_eK_e$ 稳定的对称解 $P$，于是 $J_{\min}(E_0)=E_0^TPE_0$。
这给出最优反馈，而不是把积分式对常矩阵 $K$ 的每个元素蛮力求极值。
实际程序用 `solve_continuous_are` 求 $P$，用 `solve(R, B.T@P)` 得到 $K$。
下方展示 $Q,R,P,K$ 的实际数值及残差。该最优性只针对所写的线性误差系统和无饱和二次代价。

参考：[MIT Underactuated Robotics — LQR](https://underactuated.mit.edu/lqr.html)。
'''

def make_base():
    cells=[md(r'''# 共用机器人运动学：base.py
阅读顺序：相邻位姿 → N系位姿 → 相邻角速度 → N系速度。
$R_{PC}$ 把子系 C 的分量变换到父系 P；$T_{PC}$ 的平移列也在父系中表示。
$N$ 原点位于名义车轴高度平面，故 $p_C^N=[x,y,0]^T$，地面高度为 $-r$。
$No,Co,Bo,Lo,Ro$ 分别是 N,C,B,WL,WR 的原点。左轮沿 $+C.y$、右轮沿 $-C.y$。
$l$ 是质心距车轴的高度；累计前向行程统一记为 $s$。
本文件不做小角度近似。修改 `base.py` 后统一重建所有笔记本。'''),code(SETUP),code('from base import *\nk=kinematics()\nshow("q",q)\nshow(r"\\dot q",qd)'),md('## 相邻坐标系位姿和角速度矩阵\n角速度矩阵 $[\\omega_{child/parent}^{parent}]_\\times=\\dot R R^T$，各项仍是精确三角函数。'),code("for name,T in k['adjacent'].items():\n    show('T_{'+name+'}',T)\n    show(r'[\\omega_{'+name+r'}]_{\\times}',k['relative_omega_hat'][name])"),md(r'''## 各刚体在 N 系的位置、速度和角速度
$$T_{NB}=T_{NC}T_{CB},\quad T_{NWL}=T_{NC}T_{CWL},\quad T_{NWR}=T_{NC}T_{CWR}.$$
$$v_i^N=\frac{\partial p_i^N}{\partial q}\dot q,
\quad[\omega_{i/N}^N]_\times=\dot R_{Ni}R_{Ni}^T.$$
下方是代码实际输出的展开矩阵；两个动力学场景均读取这些结果。'''),code("for name in ('C','B','WL','WR'):\n    show('T_{N'+name+'}',k['T'][name])\n    show('v_{'+name+'}^{N}',k['v'][name])\n    show(r'\\omega_{'+name+'}^{N}',k['omega'][name])\n    show(r'[\\omega_{'+name+r'}^{N}]_{\\times}',k['omega_hat'][name])"),md('## 接触点速度与三个独立约束\n接触点指当前接触位置的轮上材料点，速度用刚体公式计算，不能把持续变化的几何接触位置直接对时间求导。'),code("expressions,Acon,contact=constraints()\nfor name,value in contact.items():show('v_{contact,'+name+'}^{N}',value)\nshow('A_c\\dot q',expressions)\nshow('A_c',Acon)"),md(r'''## 能量接口
$$T_i=\tfrac12m_i(v_i^N)^Tv_i^N+\tfrac12(\omega_i^N)^T R_{Ni}I_iR_{Ni}^T\omega_i^N.$$
这里只显示逐刚体精确能量，近似方式由两个场景分别决定。'''),code("parts,T,V=energies()\nfor name,value in parts.items():show('T_{'+name+'}',value)\nshow('V',V)")]
    return cells

def make_lagrange(turning):
    scenario='turning_balance' if turning else 'straight_balance'
    intro=(r'''# 存在偏航运动：第一类与无乘子第二类拉格朗日方程
$\psi$ 与偏航速度不作小量；只把 $\theta,p=\dot\theta$ 作为俯仰小扰动。
本分支车轮轴对称且左右相同，$m_W=m_L=m_R$，$J_s=I_{Lyy}=I_{Ryy}$，$J_t=I_{Lxx}=I_{Lzz}=I_{Rxx}=I_{Rzz}$。
先由共用速度构造精确能量并求导，再展开小俯仰方程，避免丢失 $\theta\omega^2$。
可允许速度为 $u=[v,\omega,p]^T$，$v=\dot s$；$s$ 是累计前向行程（旧版写作 $l$，现已改名）。
$x,y$ 保持世界位置，$\dot x=v\cos\psi,\dot y=v\sin\psi$，不把 $v$ 冒充世界 $\dot x$。
''' if turning else r'''# 固定名义朝向：第一类与无乘子第二类拉格朗日方程
名义速度均为零，$\psi=\psi_0+\delta\psi$，$\theta=\delta\theta$。
不是把真实状态固定为目标，而是在该参考点保留能量的扰动二阶项，得到一阶动力学。
$x$ 保留现有数值仿真的**世界 x 位移扰动**；$\psi_0=0$ 时它就是名义前向位置。
$\cos\psi_0\ne0$ 是这个世界 x 速度参数化的适用条件。
此处不把偏航扰动强行设为0；保留小偏航控制通道。''')
    cells=[md(intro),code(SETUP),code(f'from {scenario}.lagrange import *\nfrom lagrange_utils import tau\nm=build()'),md('## 从共享运动学到本场景能量'),code("show('T',m['T']);show('V',m['V'])")]
    if not turning:cells += [md('保留精确表达式供对账；上面 T、V 是后续实际求导的二阶近似。'),code("show('T_{exact}',m['T_exact']);show('V_{exact}',m['V_exact'])")]
    cells += [md(r'''## 第一类：六个广义方向与三个约束反力
坐标顺序 $q=[x,y,\psi,\theta,\theta_L,\theta_R]^T$。

$$M_6\ddot q+h_6=B_6\tau+A_c^T\lambda,\quad A_c\dot q=0,$$
$$\begin{bmatrix}M_6&-A_c^T\\ A_c&0\end{bmatrix}
\begin{bmatrix}\ddot q\\\lambda\end{bmatrix}=
\begin{bmatrix}B_6\tau-h_6\\-\dot A_c\dot q\end{bmatrix}.$$

电机反作用力矩同时作用于车体：$Q_\theta=-(\tau_L+\tau_R)$，不能漏掉。
下面列出全部分块矩阵、偏置和最终9×9系数矩阵；$\lambda$ 的数值依赖约束行的归一化。'''),code(r"for key in ('M6','h6','B6','Acon','augmented_matrix','augmented_rhs'):show(key.replace('_',r'\_'),m[key])"),md(r'''## 第二类：消除乘子，保留非完整约束的修正项
选取可允许独立速度 $u$，使 $\dot q=S(q)u$、$A_c S=0$。
令 $p_q=\partial L/\partial\dot q$（这里的 $p_q$ 是动量，不是俯仰角速度 p）。

$$\frac{d}{dt}(S^Tp_q)-S^T\frac{\partial L}{\partial q}-\dot S^Tp_q=S^TB_6\tau.$$

这就是无乘子的拉格朗日—达朗贝尔方程，本文称“第二类形式”。它**不等于**先把速度约束代入 L 再直接对三个假想全局坐标套普通 Euler–Lagrange 算子。

$$M=S^TM_6S,\quad h=S^T(M_6\dot S u+h_6),\quad B_\tau=S^TB_6,$$
$$M\dot u+h=B_\tau\tau.$$

代码还从上面的动量公式独立构造 M_second、h_second，并与投影式逐项比较。'''),code("show('S',m['S']);show('u',m['u'])\nfor key in ('M','h','B'):show(key,m[key])\nassert clean(m['M']-m['M_second'])==s.zeros(3)\nassert clean(m['h']-m['h_second'])==s.zeros(3,1)\nprint('PASS：两种拉格朗日形式严格一致')")]
    if turning:cells += [md(r'''## 只对俯仰作一阶展开
令 $(\theta,p)\mapsto\epsilon(\theta,p)$；保留 $O(\epsilon)$，$v,\omega$ 不缩放。
这里指局部运动附近的加速度有界展开，不适用于俯仰加速度随 $1/\epsilon$ 发散的情况。

$$M_{small}\dot u+h_{small}=B_\tau\tau.$$

$h_{small}$ 含速度乘积，系统仍是非线性的，不能给任意转向运动使用一组固定世界坐标 A、B。
五个辅助系数依次是 $a,c,e,J,D$；$J$ 在此是偏航惯量，LQR 章节的 J 才是代价泛函。'''),code("show(s.latex(s.Matrix(s.symbols('a c e J D'))),m['coefficients'])\nshow('M_{small}',m['M_small']);show('h_{small}',m['h_small'])")]
    else:cells += [md(r'''## 最终局部矩阵
$$M_0\delta\ddot z+C_0\delta\dot z+K_0\delta z=B_0\tau,\quad z=[x,\psi,\theta]^T.$$
一阶局部模型的约束也冻结在同一静止参考点；不把局部能量模型假装成完整非线性模型。'''),code("for key,label in [('M','M_0'),('C','C_0'),('K','K_0'),('B','B_0')]:show(label,m[key])")]
    from bridge_cells import cells as reduction_cells
    insert_at = next(i for i, cell in enumerate(cells) if cell.cell_type == "markdown" and cell.source.startswith("## 第二类"))
    cells[insert_at:insert_at] = reduction_cells(turning)
    return cells

def make_lqr(turning):
    scenario='turning_balance' if turning else 'straight_balance'
    cells=[md('# '+('七状态转向轨迹误差 LQR' if turning else '六状态静止直立 LQR')),code(SETUP),code(f'from {scenario}.lqr import *\nresult=design()')]
    if turning:
        cells += [md(r'''## 世界状态与参考轨迹
$$Z=[x,y,\psi,\theta,v,\omega,p]^T,\quad
\dot Z=[v\cos\psi,v\sin\psi,\omega,p,(M^{-1}(B_\tau\tau-h))^T]^T.$$
参考选择已有实验的 $v_0=0.1\,m/s,\omega_0=0.4\,rad/s$，$\theta_r=p_r=0$，理想无耗散模型的 $\tau_r=0$。
$\psi_r=\psi_0+\omega_0t$，$x_r,y_r$ 为圆周轨迹；小量是**相对轨迹的误差**，不是世界位置或偏航角自身。

$$\delta\dot Z=A_w(t)\delta Z+B_w\delta\tau.$$

下面 A 的 `psi_ref` 随时间改变；不能把一个时刻的 A 当作整个轨迹的常矩阵。'''),code("sy=symbolic()\nshow('A_w(t)',sy['A']);show('B_w',sy['B'])"),md(r'''## 旋转位置误差得到常系数形式
这里沿用 $E$ 表示旋转后的误差（即推导中也常记作 $\eta$ 的向量）；$A_w,B_w$ 分别对应世界误差模型中的 $A,B_Z$。

**1. 定义参考航向坐标中的误差。** 世界误差为 $\delta Z=Z-Z_r$，旋转后的误差为

$$E=H(t)\delta Z=[e_{\parallel},e_{\perp},e_\psi,e_\theta,e_v,e_\omega,e_p]^T,$$

$$H(t)=\begin{bmatrix}C(\psi_r(t))&0_{2\times5}\\0_{5\times2}&I_5\end{bmatrix},\qquad
C(\psi_r)=\begin{bmatrix}\cos\psi_r&\sin\psi_r\\-\sin\psi_r&\cos\psi_r\end{bmatrix}.$$

因此前两项是位置误差在参考车头方向、参考左侧方向上的投影：

$$e_{\parallel}=\cos\psi_r\,\delta x+\sin\psi_r\,\delta y,$$
$$e_{\perp}=-\sin\psi_r\,\delta x+\cos\psi_r\,\delta y.$$

H 只旋转前两项，不旋转角度和速度误差。

**2. 对随时间变化的坐标变换求导。** 参考航向随时间变化，故 $H$ 不是常矩阵。使用乘积法则：

$$\dot E=\frac{d}{dt}(H\delta Z)=\dot H\delta Z+H\delta\dot Z.$$

代入世界坐标中的线性误差方程 $\delta\dot Z=A_w(t)\delta Z+B_w\delta\tau$，得到

$$\dot E=\dot H\delta Z+H A_w\delta Z+H B_w\delta\tau.$$

**3. 将右侧也改用 E 表示。** $C$ 是旋转矩阵，故 $H$ 是正交矩阵：

$$H H^T=H^T H=I_7,\qquad H^{-1}=H^T,\qquad \delta Z=H^T E.$$

将此关系代回上一式，并合并乘以 E 的两项：

$$\begin{aligned}
\dot E&=\dot H H^T E+H A_w H^T E+H B_w\delta\tau\\
&=(H A_w H^T+\dot H H^T)E+H B_w\delta\tau.
\end{aligned}$$

与目标形式 $\dot E=A_e E+B_e\delta\tau$ 对照，得到

$$\boxed{A_e=H A_w H^T+\dot H H^T,\qquad B_e=H B_w.}$$

其中 $H A_w H^T$ 是原误差动力学的坐标变换，$\dot H H^T$ 是误差坐标系自身转动带来的附加项；后者不能省略。

**4. 展开坐标系转动的附加项。** 此处 $\dot\psi_r=\omega_0$，所以

$$\dot C=\omega_0\begin{bmatrix}-\sin\psi_r&\cos\psi_r\\-\cos\psi_r&-\sin\psi_r\end{bmatrix},\qquad
\dot C C^T=\begin{bmatrix}0&\omega_0\\-\omega_0&0\end{bmatrix}.$$

$\dot H H^T$ 只有左上角的这个 $2\times2$ 块非零。结合 $H A_w H^T$，位置误差的前两行变为

$$\dot e_{\parallel}=\omega_0 e_{\perp}+e_v,\qquad
\dot e_{\perp}=-\omega_0 e_{\parallel}+v_0 e_\psi.$$

它们不再含绝对参考航向 $\psi_r$。另外，世界输入矩阵的结构为

$$B_w=\begin{bmatrix}0_{4\times2}\\M_0^{-1}B_\tau\end{bmatrix}.$$

H 只旋转 B_w 中原本为零的前两行，因此在当前模型中 $B_e=H B_w=B_w$。
对于此处恒定 $v_0,\omega_0$，其余动力学系数也恒定，因此 $A_e,B_e$ 是常矩阵，可用于求解固定增益 $K_e$。
$Q$ 的两个位置权重相同，因此这个选择也等价于世界位置误差的各向同性惩罚。'''),code("show('H',sy['H']);show('A_e',sy['Ae']);show('B_e',sy['Be'])")]
    else:
        cells += [md(r'''## 状态方程
$$X=[\delta x,\delta\psi,\delta\theta,\delta\dot x,\delta\dot\psi,\delta\dot\theta]^T,$$
$$\dot X=AX+B\tau,\quad A=\begin{bmatrix}0&I\\-M_0^{-1}K_0&-M_0^{-1}C_0\end{bmatrix},\quad B=\begin{bmatrix}0\\M_0^{-1}B_0\end{bmatrix}.$$
下面先展示符号矩阵，再代入当前项目的临时示例参数。数值实现使用 solve，不显式求逆。'''),code("As,Bs=symbolic();show('A',As);show('B',Bs)")]
    cells += [md(COST),code("from parameters import values\nprint('临时示例参数（不是实车值）：',values()[0])\nfor key in ('A','B','Q','R','P','K'):show(key,s.Matrix(result[key]).evalf(8))\nprint('可控性秩：',result['rank'])\nprint('闭环特征值：',result['poles'])\nprint('CARE残差：',result['care_residual'])"),md(r'''## 最优代价示例
只给初始俯仰误差 0.03 rad，其他误差为0；下面得到该线性无限时域问题的最小代价数值。'''),code("E0=np.zeros(len(result['A']));E0[3 if len(E0)==7 else 2]=.03\nshow('J_{min}',s.Float(E0@result['P']@E0))")]
    if turning:cells += [md(r'''## 世界坐标反馈与适用边界
$$\tau=\tau_r-K_eE=\tau_r-K_w(t)\delta Z,\quad K_w(t)=K_eH(t).$$
K_e 是上面的 2×7 常矩阵，每一行对应一侧电机、每列对应一种误差。

### 实现时必须区分两种误差向量

**固定增益 $K_e$ 对应旋转后的误差 E，不可直接乘世界坐标误差 $Z-Z_r$。** 两者的状态排列为：

$$\delta Z=[x-x_r,y-y_r,\delta\psi,\delta\theta,\delta v,\delta\omega,\delta p]^T,$$
$$E=[e_{\parallel},e_{\perp},\delta\psi,\delta\theta,\delta v,\delta\omega,\delta p]^T=H(\psi_r)\delta Z.$$

前两项分别为**参考车辆的前向位置误差、参考车辆左侧的横向位置误差**：

$$e_{\parallel}=\cos\psi_r(x-x_r)+\sin\psi_r(y-y_r),$$
$$e_{\perp}=-\sin\psi_r(x-x_r)+\cos\psi_r(y-y_r).$$

它们不是世界坐标中的 $\delta x,\delta y$。前向沿参考车头方向；参考前进速度为正时，它也是轨迹切向，横向则是法向。变换必须使用**参考航向 $\psi_r$**，与本笔记本的模型和增益保持一致；其余五项不做坐标旋转，排列也不能改变。

每个控制周期按以下流程计算（代码示意，`state` 来自实际状态估计，`t` 与参考轨迹使用同一时间起点）：

```python
reference_state = reference(t, v0=result['v0'], w0=result['w0'])
error_world = np.asarray(state, dtype=float) - reference_state
# 把实际航向与参考航向之差映射到 [-pi, pi)，避免跨越 2*pi 的跳变。
error_world[2] = (error_world[2] + np.pi) % (2 * np.pi) - np.pi

H_ref = error_transform(reference_state[2])
error_reference = H_ref @ error_world  # 即 E，前两项为前向、左向误差
K_e = result['K']
delta_torque = -K_e @ error_reference
reference_torque = np.zeros(2)  # 当前恒定速度、直立、无耗散参考的 tau_r=0
torque = reference_torque + delta_torque  # 顺序为 [tau_L, tau_R]
```

此处的 $\dot E=A_eE+B_e\delta\tau$ 描述 E 的变化率；反馈使用的是 **E 本身**。等价实现是先计算 $K_w(t)=K_eH(\psi_r)$，再用 $\tau=\tau_r-K_w(t)\delta Z$；不能省略 H，也不能把已经旋转过的 E 再旋转一次。

### 参考朝向示例与适用边界

下面列出参考朝向 0.7 rad 时的世界 A、B、K，验证矩阵不是只支持零偏航。
若 $v_0=\omega_0=0$，横向位置出现不可控零模态，不能用正定位置代价得到稳定全部七状态的 CARE 解；静止平衡请使用六状态分支。
一般加减速参考不满足常系数条件，需重新沿可行轨迹线性化并处理时变 Riccati 问题。
当前 ROS `balance_yaw` 仍使用已验收的六状态里程控制器，本次七状态理论并未自动替换它。'''),code("Aw,Bw,Ae,H=state_matrices(heading=.7)\nshow('A_w(0.7)',s.Matrix(Aw).evalf(8));show('B_w',s.Matrix(Bw).evalf(8))\nshow('K_w(0.7)',s.Matrix(result['K']@H).evalf(8))\nA0,B0,_,_=state_matrices(0,0)\nprint('静止七状态可控性秩：',np.linalg.matrix_rank(np.hstack([np.linalg.matrix_power(A0,i)@B0 for i in range(7)])))")]
    return cells

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--no-execute',action='store_true');args=parser.parse_args()
    jobs=[('base.ipynb',make_base()),('straight_balance/lagrange.ipynb',make_lagrange(False)),
          ('straight_balance/lqr.ipynb',make_lqr(False)),('turning_balance/lagrange.ipynb',make_lagrange(True)),('turning_balance/lqr.ipynb',make_lqr(True))]
    notebooks=[]
    # 临时 kernelspec 明确绑定执行本脚本的解释器，不依赖用户机器的默认 python kernel。
    import sys
    with tempfile.TemporaryDirectory(prefix='balance-notebook-') as temp:
        kernels=Path(temp)/'kernels'/'balance-build';kernels.mkdir(parents=True)
        (kernels/'kernel.json').write_text(json.dumps({'argv':[sys.executable,'-m','ipykernel_launcher','-f','{connection_file}'],'display_name':'Balance build','language':'python'}))
        old=os.environ.get('JUPYTER_PATH');os.environ['JUPYTER_PATH']=temp+(os.pathsep+old if old else '')
        try:
            for relative,cells in jobs:
                nb=nbf.v4.new_notebook(cells=cells,metadata={'kernelspec':{'name':'python3','display_name':'Python 3','language':'python'}})
                if not args.no_execute:
                    print('执行',relative,flush=True)
                    NotebookClient(nb,timeout=600,kernel_name='balance-build',resources={'metadata':{'path':str(ROOT)}}).execute()
                notebooks.append((ROOT/relative,nb))
        finally:
            if old is None:os.environ.pop('JUPYTER_PATH',None)
            else:os.environ['JUPYTER_PATH']=old
    for path,nb in notebooks:
        staging=path.with_suffix('.ipynb.tmp');nbf.write(nb,staging);staging.replace(path)
        print('已更新',path.relative_to(ROOT),flush=True)
    print('完成：5本笔记本。'+('尚未执行。' if args.no_execute else '全部执行无错误。'))
if __name__=='__main__':main()
