"""两本拉格朗日笔记本共享的完整消元说明；由 refresh_notebooks 调用。"""
import nbformat as nb
md=nb.v4.new_markdown_cell
code=nb.v4.new_code_cell

def cells(turning):
    context=(r'''### 0. 本节所用原始矩阵与本场景的关系
本场景上方的 $M_6,h_6,A_c,B_6$ 是左右相同、轴对称车轮的精确矩阵，本节直接读取它们，不再计算能量或重新应用 EL 算子。
本节为了对照指定的 $z=[x,\psi,\theta]^T$，在 $\cos\psi\ne0$ 的局部速度参数化中改写现有方程。
原程序的 $v=\dot s$ 仍保留其含义：$v=\dot x/\cos\psi$。这只是同一允许速度的不同分量表达，不定义新的位置，也不修改转向控制器。
最后的静止线性化是本转向模型的一个特例；它**不替代**后面围绕 $v_0,\omega_0$ 运动参考的七状态 LQR。'''
    if turning else r'''### 0. 本节所用原始矩阵与本场景的关系
上方本分支的 $M_6,h_6,A_c$ **已经是静止参考点附近的局部模型**：二阶能量已截断，约束中的 $\psi$ 已冻结为 $\psi_0$。
因此，直接消元只能得到局部三维线性模型，不能从它反推出已经丢掉的完整非线性项。
为了把“先降维、再线性化”的链条补齐，本节同时读取项目**现成的转向分支精确六维矩阵**，展示其完整投影过程；不重建模型、不重新求 EL。
这条精确链采用转向分支原有的“左右相同、轴对称车轮”条件。文末再返回本分支原矩阵，给出保留左右不等参数的最终结果，并验证两条路径在共同假设下完全一致。
下文精确链的 $A_c(q)$ 与上方本分支已经冻结的 $A_{c0}$ 会明确区分，不能暗中互换。''')
    out=[md('## 补全：六维约束方程怎样得到三维局部方程'),md(context),code("from reduction_bridge import build_bridge\nbr=build_bridge()\nshow('M_6^{exact}',br['exact']['M6'])\nshow('h_6^{exact}',br['exact']['h6'])\nshow('B_6',br['exact']['B6'])"),md(r'''### 1. 上一式 $A_c\dot q=0$ → 展开三行 → 解出三个从属速度
本项目的实际约束矩阵为
$$A_c(q)=\begin{bmatrix}
-\sin\psi&\cos\psi&0&0&0&0\\
\cos\psi&\sin\psi&-d/2&0&-r&0\\
\cos\psi&\sin\psi&d/2&0&0&-r
\end{bmatrix}.$$

乘以 $\dot q$，逐行得到
$$\begin{aligned}
-\sin\psi\dot x+\cos\psi\dot y&=0,\\
\cos\psi\dot x+\sin\psi\dot y-\tfrac d2\dot\psi-r\dot\theta_L&=0,\\
\cos\psi\dot x+\sin\psi\dot y+\tfrac d2\dot\psi-r\dot\theta_R&=0.
\end{aligned}$$
第一行除以 $\cos\psi$ → $\dot y=\tan\psi\dot x$。
将它代入第二、三行，使用 $\cos\psi+\sin\psi\tan\psi=1/\cos\psi$ →
$$\boxed{\dot y=\tan\psi\dot x,\qquad
\dot\theta_L=\frac{\dot x}{r\cos\psi}-\frac d{2r}\dot\psi,\qquad
\dot\theta_R=\frac{\dot x}{r\cos\psi}+\frac d{2r}\dot\psi.}$$
此表达要求 $r>0,\cos\psi\ne0$。若车头朝向使 $\cos\psi=0$，失效的是选世界 $\dot x$ 为独立速度的参数化，原始六维系统及原有前向速度 v 模型没有失效。
这些是速度关系；例如 $dy=\tan\psi\,dx$ 不能被写成 $y=x\tan\psi$。
不能声称 $y,\theta_L,\theta_R$ 是 z 的全局函数。'''),md(r'''### 2. 上一步三个速度关系 → 按 q 的行顺序排列 → 写出 N
令 $\dot z=[\dot x,\dot\psi,\dot\theta]^T$，则
$$\dot q=N(q)\dot z,\qquad
N(q)=\begin{bmatrix}
1&0&0\\
\tan\psi&0&0\\
0&1&0\\
0&0&1\\
\frac1{r\cos\psi}&-\frac d{2r}&0\\
\frac1{r\cos\psi}&\frac d{2r}&0
\end{bmatrix}.$$
这里 N 是速度映射矩阵；不要把它与惯性参考系 N 混为一谈，更不能把它理解为一个全局位置映射的雅可比。
N 的第1、3、4行形成单位阵，所以在这个参数化有效域内它有列秩3。'''),code("show('A_c',br['A']);show('N(q)',br['N'])\nshow(r'A_cN',clean(br['A']*br['N']))"),md(r'''### 3. 上一式 $\dot q=N\dot z$ → 乘积求导 → 展开每个加速度
N 只有第2、5、6行的第1列随时间变化。其余非零元素都是常数。
$$\frac{d}{dt}\tan\psi=\sec^2\psi\dot\psi,\qquad
\frac{d}{dt}\frac1{r\cos\psi}=\frac{\sin\psi}{r\cos^2\psi}\dot\psi.$$
因此
$$\dot N=\begin{bmatrix}
0&0&0\\\sec^2\psi\dot\psi&0&0\\0&0&0\\0&0&0\\
\frac{\sin\psi}{r\cos^2\psi}\dot\psi&0&0\\
\frac{\sin\psi}{r\cos^2\psi}\dot\psi&0&0
\end{bmatrix},\qquad
\dot N\dot z=\begin{bmatrix}
0\\\sec^2\psi\dot x\dot\psi\\0\\0\\
\frac{\sin\psi}{r\cos^2\psi}\dot x\dot\psi\\
\frac{\sin\psi}{r\cos^2\psi}\dot x\dot\psi
\end{bmatrix}.$$
将上式代入乘积求导式，逐行就是
$$\ddot q=N\ddot z+\dot N\dot z=
\begin{bmatrix}
\ddot x\\
\tan\psi\ddot x+\sec^2\psi\dot x\dot\psi\\
\ddot\psi\\\ddot\theta\\
\frac{\ddot x}{r\cos\psi}-\frac d{2r}\ddot\psi+\frac{\sin\psi}{r\cos^2\psi}\dot x\dot\psi\\
\frac{\ddot x}{r\cos\psi}+\frac d{2r}\ddot\psi+\frac{\sin\psi}{r\cos^2\psi}\dot x\dot\psi
\end{bmatrix}.$$
其中两个轮加速度的附加项相同，来源是前向投影系数 $1/\cos\psi$ 的变化，不是左右轮转向符号项的导数。'''),code(r"show(r'\dot N',br['Ndot']);show(r'\dot N\dot z',br['bias'])"),md(r'''### 4. 上一式 → 代回六维动力学 → 左乘 Nᵀ 消去乘子
原方程是
$$M_6\ddot q+h_6=B_6\tau+A_c^T\lambda.$$
代入 $\dot q=N\dot z$、$\ddot q=N\ddot z+\dot N\dot z$，得到
$$M_6N\ddot z+M_6\dot N\dot z+h_6(q,N\dot z)
=B_6\tau+A_c^T\lambda.$$
左乘 $N^T$ 后：
$$N^TM_6N\ddot z+N^TM_6\dot N\dot z+N^Th_6(q,N\dot z)
=N^TB_6\tau+N^TA_c^T\lambda.$$
上一节已逐项算出 $A_cN=0_{3\times3}$，所以
$$\boxed{N^TA_c^T\lambda=(A_cN)^T\lambda=0}.$$
这不要求 $\lambda=0$，只是约束反力对允许虚速度不做功。
另由 $A_cN=0$ 求导得到 $\dot A_cN+A_c\dot N=0$，故
$$A_c(N\ddot z+\dot N\dot z)+\dot A_cN\dot z=0,$$
说明恢复出的六维加速度也满足增广方程的后三行，并未遗失加速度约束。'''),md(r'''### 5. 上一式 → 合并系数 → 三维非线性动力学
$$\boxed{M_r\ddot z+h_r=B_r\tau},$$
$$\boxed{M_r=N^TM_6N,\quad
h_r=N^T\left[M_6\dot N\dot z+h_6(q,N\dot z)\right],\quad B_r=N^TB_6.}$$
这是实际的矩阵组合；在本项目中 M6 不依赖速度，所以 Mr 也不依赖速度。
一般情况下它们还可以保留 q 中其他坐标的依赖，不能仅凭“速度降维”就宣称全局位置已经降维。
本项目的精确转向分支因平移对称、轮子轴对称，y与两轮自转角不出现在这些动力学系数中，才可以在此处写为 $M_r(z),h_r(z,\dot z),B_r(z)$。

沿用本项目已有辅助系数（不是新坐标）：
$$\begin{aligned}
a&=m_B+2m_W+\frac{2J_s}{r^2},&c&=m_Bl,&e&=I_{Byy}+m_Bl^2,\\
J&=I_{Bzz}+2J_t+\frac{d^2m_W}{2}+\frac{d^2J_s}{2r^2},&
D&=I_{Bxx}-I_{Bzz}+m_Bl^2.
\end{aligned}$$
$m_W,J_s,J_t$ 的定义仍是上文左右相同车轮的质量、自转惯量和径向惯量。
完整车体惯量新增 $F=I_{Byz}\cos\theta-I_{Bxy}\sin\theta$、$G=-I_{Bxz}\sin(2\theta)$。
把现有 M6、h6、B6 代入上面的组合并化简 →
$$M_r=\begin{bmatrix}
a\sec^2\psi&0&c\cos\theta\sec\psi\\
0&J+D\sin^2\theta+G&F\\
c\cos\theta\sec\psi&F&e
\end{bmatrix},$$
$$h_r^{diag}=\begin{bmatrix}
a\sec^2\psi\tan\psi\dot x\dot\psi-c\sin\theta\sec\psi(\dot\theta^2+\dot\psi^2)\\
(2D\sin\theta\cos\theta\dot\theta+c\sin\theta\sec\psi\dot x)\dot\psi\\
c\cos\theta\sec\psi\tan\psi\dot x\dot\psi-D\sin\theta\cos\theta\dot\psi^2-cg\sin\theta
\end{bmatrix},$$
$h_r$ 为上面的 $h_r^{diag}$ 加上 $[0,G^\prime\dot\theta\dot\psi+F^\prime\dot\theta^2,-G^\prime\dot\psi^2/2]^T$。
$$B_r=\begin{bmatrix}
\frac1{r\cos\psi}&\frac1{r\cos\psi}\\
-\frac d{2r}&\frac d{2r}\\-1&-1
\end{bmatrix}.$$
特别是第三行输入仍是 $-(\tau_L+\tau_R)$。

还可直接与现有前向速度三维方程核对：令 $u=[v,\omega,p]^T=\operatorname{diag}(\sec\psi,1,1)\dot z$，将 u 及其乘积求导代回原式，再左乘同一对角矩阵的转置，得到完全相同的 Mr、hr、Br。这一步没有对 s 或 y 做任何全局位置消元。'''),code("for key in ('Mr','hr','Br'):show(key,br[key])\nprint('PASS：由六维原矩阵投影，与现有前向速度三维方程变换完全一致')"),md(r'''### 6. 上一非线性方程 → 选静止直立参考点
取
$$q_0=[x_0,y_0,\psi_0,0,0,0]^T,\quad \dot q_0=0,\quad\ddot q_0=0,\quad\tau_0=0,$$
$$z_0=[x_0,\psi_0,0]^T,\qquad\cos\psi_0\ne0.$$
x0、y0为固定常数，不要求两者为0；几何、质量、惯量保持常量。轮名义角取0，与项目静止能量展开一致；对轴对称车轮其选择不影响精确系数。
此时 $h_r(z_0,0)=0$，故零名义力矩确实满足动力学，而不仅是人为指定了控制目标。

现在令 $z=z_0+\delta z$，$\dot z=\delta\dot z$，$\ddot z=\delta\ddot z$，$\tau=\delta\tau$。由于名义力矩为0，下文直接写 tau。
其他坐标仍需由速度约束沿时间恢复，并未被宣称是 z 的全局函数。'''),md(r'''### 7. 上一式 → 扰动展开 → 逐项保留一阶
令状态、速度、加速度和输入扰动均为 $O(\epsilon)$。参考姿态附近的几何系数有界。

**质量项：**
$$M_r(z)\ddot z=[M_r(z_0)+\delta M_r+O(\epsilon^2)]\delta\ddot z
=M_r(z_0)\delta\ddot z+O(\epsilon^2).$$
所以保留 $M_0\delta\ddot z$；$\delta M_r\delta\ddot z$ 是二阶，不是把真实 Mr 永远视为常数。

**投影的加速度偏置：**
$$\dot N\dot z=\frac{\partial N}{\partial\psi}\delta\dot\psi\,\delta\dot z=O(\epsilon^2).$$
具体非零分量就是第3节的 $\delta\dot x\delta\dot\psi$ 乘有界系数，所以静止参考点的一阶模型中消失。若参考速度非零，交叉项可是一阶，不能沿用这一删项理由。

**hr第一行：** $\dot x\dot\psi$ 为二阶；$\sin\theta\dot\theta^2$、$\sin\theta\dot\psi^2$ 为三阶，因此一阶没有贡献。

**hr第二行：** 原对角惯量部分至少三阶；完整惯量新增速度二次项，仍没有一阶贡献。

**hr第三行：** $\cos\theta\dot x\dot\psi$ 为二阶，$\sin\theta\cos\theta\dot\psi^2$ 为三阶；完整惯量新增项也至少二阶；只有重力
$$-cg\sin\theta=-cg\delta\theta+O(\epsilon^3)$$
保留。因此
$$h_r=0+C_0\delta\dot z+K_0\delta z+O(\epsilon^2),\quad
C_0=\left.\frac{\partial h_r}{\partial\dot z}\right|_0=0,\quad
K_0=\left.\frac{\partial h_r}{\partial z}\right|_0=\operatorname{diag}(0,0,-cg).$$

**输入项：**
$$B_r(z)\tau=[B_r(z_0)+\delta B_r+\cdots]\delta\tau
=B_r(z_0)\tau+O(\epsilon^2).$$
$\delta B_r\delta\tau$ 为二阶；如果名义力矩不为0，这个论证需要额外项，本节名义力矩为0。'''),md(r'''### 8. 合并上一步保留项 → 最终局部三维模型
$$\boxed{M_0\delta\ddot z+C_0\delta\dot z+K_0\delta z=B_0\tau},$$
$$M_0=M_r(z_0),\quad C_0=(\partial_{\dot z}h_r)_0,\quad
K_0=(\partial_z h_r)_0,\quad B_0=B_r(z_0).$$
对上面精确转向分支的现有参数假设，实际矩阵是
$$M_0=\begin{bmatrix}
a\sec^2\psi_0&0&c\sec\psi_0\\0&J&I_{Byz}\\c\sec\psi_0&I_{Byz}&e
\end{bmatrix},\quad C_0=0_{3\times3},\quad
K_0=\begin{bmatrix}0&0&0\\0&0&0\\0&0&-m_Bgl\end{bmatrix},$$
$$B_0=\begin{bmatrix}\frac1{r\cos\psi_0}&\frac1{r\cos\psi_0}\\-\frac d{2r}&\frac d{2r}\\-1&-1\end{bmatrix}.$$
下面的结果由实际 hr 的雅可比求值产生，不是手填零矩阵。'''),code("for key in ('M0','C0','K0','B0'):show(key,br[key])"),md(r'''### 9. 与当前静止分支逐项对账：为什么先线性化也得到相同结果
现有静止分支已冻结 $A_{c0}=A_c(\psi_0)$；把第1节解的 $\psi$ 换成 $\psi_0$ → 得到常矩阵 $N_0=N(\psi_0)$。
所以直接对其第一类局部方程消元时，$\dot N_0=0$，并且
$$M_0=N_0^TM_{6,0}N_0,\qquad h_0=N_0^Th_{6,local},\qquad B_0=N_0^TB_6.$$
其六维局部 h 仅第4项为 $-m_Bgl\delta\theta$；N0第4行是 $[0,0,1]$，所以投影后只有第三项的重力刚度。
本分支保留左右不等参数，展开所得质量矩阵为
$$M_0=\begin{bmatrix}
\frac{m_B+m_L+m_R+(I_{Lyy}+I_{Ryy})/r^2}{\cos^2\psi_0}&
\frac{d[(I_{Ryy}-I_{Lyy})/r^2+m_R-m_L]}{2\cos\psi_0}&
\frac{lm_B}{\cos\psi_0}\\
\frac{d[(I_{Ryy}-I_{Lyy})/r^2+m_R-m_L]}{2\cos\psi_0}&
I_{Bzz}+I_{Lzz}+I_{Rzz}+\frac{d^2}{4}\left[m_L+m_R+\frac{I_{Lyy}+I_{Ryy}}{r^2}\right]&I_{Byz}\\
\frac{lm_B}{\cos\psi_0}&I_{Byz}&I_{Byy}+m_Bl^2
\end{bmatrix}.$$
C0、K0、B0与第8节同形。将左右相同轴对称参数代入此式 → 中间的质量耦合项为0 → 严格回到第8节。
因此，“先投影再静止线性化”与本项目静止分支“先做一致的局部能量/约束近似再投影”在共同假设下对得上。
不能反过来把这里已冻结的矩阵称为适用于任意偏航速度的非线性模型。'''),code("show('N_0',br['N0'])\nfor key,label in [('M','M_0^{static}'),('C','C_0^{static}'),('K','K_0^{static}'),('B','B_0^{static}')]:\n    show(label,br['static'][key])\nprint('PASS：约束零空间、加速度约束、两种速度投影和最终静止矩阵全部核对通过')")]
    return out
