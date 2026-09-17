# 平衡车模型验证 · 阶段总结(v3 / 2026-09-05)

> 目的:把你手写的那份 ~1 万字拉格朗日推导,用 SymPy(符号)+ 数值对账的方式重新推一遍并核对。
> 本文件是**自包含**的恢复入口:哪天回来,读这一份就能接上,不必翻聊天记录。

---

## 0. 一句话状态

完整模型(六坐标 + 三个非完整约束)的**约化动力学已导出并用 KanesMethod 验证**:
约化质量阵对称、回归锚精确、独立数值对账到 ~1e-14。
**还差**:和你手稿的逐项人工比对 —— 这一步只能你来做,我没见过你的原稿。

---

## 1. 模型定义(与你的约定一致)

### 对象与刚体
左右轮 + 车体,三个刚体;**不计电机转子惯量**,轮子**纯滚动无滑**。

### 六个广义坐标
| 坐标 | 含义 |
|---|---|
| $x, y$ | 车轴中点 $C_o$ 在惯性系 $N$ 的水平位移(世界分量) |
| $\psi$ | 车轴系 $C$ 相对 $N$ 的偏航(绕 $N.z$) |
| $\theta$ | 车体 $B$ 相对 $C$ 的俯仰(绕车轴 $C.y$) |
| $\theta_L, \theta_R$ | 左右轮相对 $C$ 的滚动(绕 $C.y$) |

### 坐标系 / 原点
- $N$ 惯性系,$C$ 车轴系,$L/R$ 轮系,$B$ 车体系。
- $C_o$ = 轴心中点(离地 $r$);$L_o=C_o+\frac d2 C.y$;$R_o=C_o-\frac d2 C.y$;$B_o=C_o+l\,B.z$(质心)。
- 接地点 $C_L=C_L$轮心 $-r\,N.z$(接触点的速度按轮刚体算,不是固定点)。

### 参数
$r$ 轮半径,$d$ 轮距,$l=C_o\!\to\!B_o$,$g$;质量 $m_L,m_R,m_B$;
惯量 $I_{Lxx},I_{Lyy},I_{Lzz}$(L 系主轴,左轮)等;**
轴对称轮:$I_{Lxx}=I_{Lzz}$(直径惯量),$I_{Lyy}$(绕车轴自转)。

### 三个独立非完整约束($6\to 3$)
1. **无侧滑**:$v(C_o)\cdot C.y=0$ —— 注意是**车体侧向轴 $C.y$**,不是世界 $N.y$,所以允许转弯。
2. **左轮纯滚**:$v(C_L)\cdot C.x=0$
3. **右轮纯滚**:$v(C_R)\cdot C.x=0$

解出差速关系:
$$\dot y=\dot x\tan\psi,\qquad
\dot\theta_L=\frac{v_f-\frac d2\dot\psi}{r},\qquad
\dot\theta_R=\frac{v_f+\frac d2\dot\psi}{r},\qquad v_f=v(C_o)\cdot C.x.$$

独立广义坐标取 $(x,\psi,\theta)$;$y,\theta_L,\theta_R$ 由约束决定(速度层约束 → 非完整)。

> ⚠️ **$x$ 的语义(重要)**:当时你选了"选项 2",即 $x$ = **世界 $N.x$ 位移**(不是车头方向位移/弧长)。
> 因此 $\dot y=\dot x\tan\psi$、$\cos\psi$ 分母出现 —— 这是坐标选择的固有后果,不是建模错。
> 电机扭矩因此对 $x$ **没有显式广义力**($Q_x=0$),前进靠 θ 耦合间接产生(见 §3)。
> 若你的手稿里 $Q_x$ 带驱动项,多半你 $x$ 取的是弧长/车头前进位移 —— 这正是当初说的"分支 1(弧长 s)"的动机。

---

## 2. 最终结果:约化动力学

$M_r(\psi,\theta)\,(\ddot x,\ \ddot\psi,\ \ddot\theta)^\top = f_r(q,\dot q) + Q(\tau_L,\tau_R)$

### 约化质量矩阵(实对称;左右轮允许不对称)
$$\scriptsize
M_r=
\begin{bmatrix}
\dfrac{I_{Lyy}+I_{Ryy}+r^2(m_B+m_L+m_R)}{r^2\cos^2\psi}
& -\dfrac{d\,[I_{Lyy}-I_{Ryy}+r^2(m_L-m_R)]}{2r^2\cos\psi}
& \dfrac{l\,m_B\cos\theta}{\cos\psi}\\[2mm]
\ast
& I_{Bxx}\sin^2\theta+I_{Bzz}\cos^2\theta+I_{Lxx}+I_{Rxx}+\dfrac{d^2(I_{Lyy}+I_{Ryy})}{4r^2}+\dfrac{d^2(m_L+m_R)}4+l^2m_B\sin^2\theta
& 0\\[2mm]
\ast & 0 & I_{Byy}+l^2m_B
\end{bmatrix}
$$
($\ast$ = 对称)

### 右端 $f_r$ 的结构
- **重力只在 θ 方程**:$-m_B\,g\,l\sin\theta$ 那一族。
- $x,\psi$ 方程**无重力项**,只有速度耦合:陀螺项 $(\dots)\dot\psi\dot\theta\sin2\theta$、离心项等。ψ 是循环坐标。
- 静止时 $f_r=(0,\ 0,\ +m_B g l\sin\theta)$,精确等于 $-\partial V/\partial q$(其中 $V=m_B g(r+l\cos\theta)+\text{const}$)。

### 电机扭矩的广义力
$$Q_x=0,\qquad
Q_\psi=\frac{d}{2r}(\tau_R-\tau_L),\qquad
Q_\theta=-(\tau_L+\tau_R).$$

> 含义:电机是**内力**,作用在被约化掉的轮自转坐标上。以世界 $x$ 作广义坐标时前进不出显式 $Q_x$。
> 改用弧长 $s$ 为坐标时,$Q_s=(\tau_L+\tau_R)/r$ 会显式出现(更干净,对应你手稿里的直观直觉)。

---

## 3. 验证情况(都过了)

| 检查 | 方法 / 证据 | 结果 |
|---|---|---|
| 差速运动学 | 直行 / 原地转向自检 | $\dot\theta_L=\dot\theta_R=\frac1r$;$\ \pm\frac{d}{2r}$ ✓ |
| Mr 对称 | `sympy` | True |
| 回归锚 | 直立直线 $M_{00}=\dfrac{I_{Lyy}+I_{Ryy}}{r^2}+m_B+m_L+m_R$ | 差 = 0 |
| 独立数值对账 | `check_full_multi.py`:全六坐标拉格朗日 + d'Alembert 乘子 **vs** KanesMethod 约化,随机位形/速度 | 重力 ~1e-14,加电机 ~2.75e-14,含左右不对称轮 ✓ |
| 电机映射 | `test_motor.py`;载荷不改 Mr;Q 无 $\tau^2$ 残余 | $Q_x{=}0,\ Q_\psi{=}\tfrac d{2r}(\tau_R{-}\tau_L),\ Q_\theta{=}-(\tau_L{+}\tau_R)$ ✓ |
| 静止右端 | notebook 格 9 | $f_r=(0,0,+m_B g l\sin\theta)=-\partial V/\partial q$ ✓ |

### 三条约化显式方程(逐项对活用)
- **渲染版**:`sympy_scaffold.ipynb` 格 8–9(LaTeX)。
- **纯文本版**:`reduced_eom_v3.txt`(已代入 $\dot y=\dot x\tan\psi$,并把 $u_*$ 换成真导数;三条都写成 `表达式 = 0`)。

---

## 4. 文件清单

| 文件 | 角色 | 状态 |
|---|---|---|
| `sympy_scaffold.py` | 唯一"正本":§1–§7 完整可跑脚本 | ✅ v3,最新 |
| `sympy_scaffold.ipynb` | 同模型的**互动 notebook**(14 格,含 markdown 讲解、方程渲染、已执行输出) | ✅ v3,已执行 0 报错 |
| `build_notebook.py` | notebook 生成器;改模型后用它重建 | ✅ v3 |
| `reduced_eom_v3.txt` | 三条约化方程的纯文本全式 | ✅ 对活用 |
| `SUMMARY.md` | 本文件 | ✅ |
| `check_full_multi.py` | 独立数值对账(验证物证) | ✅ 保留 |
| `test_motor.py` | 电机扭矩→广义力映射(验证物证) | ✅ 保留 |
| `sketch_kane.py`, `sketch_out.txt` | §7 之前的原型与日志 | 🗑️ 已删(被 §7 取代) |
| `.venv/` | Python 环境(sympy/numpy/scipy/pin…) | ✅ 勿删 |

---

## 5. 怎么跑 / 怎么重建

```bash
# 环境
cd /home/mechcid/lagarain
.venv/bin/python -m jupyter lab        # 打开 notebook, kernel 选 python3(= .venv)或 lagarain

# 改了模型后重建 notebook
.venv/bin/python build_notebook.py
.venv/bin/python -m jupyter nbconvert --to notebook --execute --inplace sympy_scaffold.ipynb
```

环境备注:系统 `/usr/bin/python3` 是 apt 管的、没 pip,**别重指**;ROS2 Jazzy 依赖它。
`.venv` 是独立环境(勿删)。

---

## 6. 恢复入口:回来先做这几步

1. 打开 `sympy_scaffold.ipynb`(§3/§5 是核心),或读 `reduced_eom_v3.txt`。
2. 拿你的手稿逐项对 `reduced_eom_v3.txt` 三条方程。
3. 对不上的地方,先确认**符号约定**(见 §7),再让我查(可能是你那边坐标/角度定义,或我这边哪项)。
4. 对完确认无误 → 做 **弧长 s 分支**(当初的"选项 1",$Q_s=(τ_L+τ_R)/r$ 会显式出现)。
5. 之后 → **Pinocchio 数值基线**(Python 起步,再转 C++,为工作做准备)。

---

## 7. 对账时最容易踩的符号约定

- **θ 正向**:绕车轴 $C.y$ 右手转(由 $B.z$ 相对 $C$ 的取向 $\theta$ 定义)。
- **Kane 的 $f_r$ 含 $-\partial V/\partial q$**;重力项在 θ 方程里是 $-m_B g l\sin\theta$(写到 $=0$ 一侧)。
- **$x$ = 世界 $N.x$ 分量**,所以 $\dot y=\dot x\tan\psi$、质量阵里有 $\cos\psi$、$\cos^2\psi$ 分母;左右轮不对称时 $M_{01}\ne0$、$M_{00}$ 有 d 项耦合。
- **电机**:$Q_x=0$(世界 $x$ 下);若要"$Q_x$ 出现驱动项",那是弧长/车头坐标版本。
- 车轮惯量取 $I_{xx}=I_{zz}$(直径相等),自转轴惯量是 $I_{yy}$。
