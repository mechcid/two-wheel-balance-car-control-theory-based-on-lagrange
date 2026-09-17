# 持续偏航运动：新动力学分支

本分支保留旧 `theory/sympy_scaffold.py` 和静止点近似，不用原点线性模型冒充转弯模型。推导仍采用六坐标拉格朗日方程及非完整约束投影，没有改用Kane方法。

## 1. 先把新旧符号对应写清楚

|量|旧分支|新分支|
|---|---|---|
|独立前向运动描述|世界x位移|累计前向行程 **l**，代码/CSV字段 `l`|
|车轴到车体质心距离|参数l，physical.yaml中的 `l`|**l_c**，数值代码用 `com_height`|
|偏航角|psi|仍用 **psi**；即本次需求称作的phi|
|车体俯仰角|theta|theta，未改|

新分支在参数适配处明确执行 `params['com_height'] = old_params['l']`。不能把物理参数文件中的0.30 m当成运动坐标l的初始值。l初值取0；物理参数l_c始终为0.30 m。

定义

\[
v=\dot l,\quad \omega=\dot\psi,\quad p=\dot\theta,
\qquad \dot x=v\cos\psi,\quad \dot y=v\sin\psi.
\]

**l是沿实际朝向累计的有符号行程，不是世界x，也不是世界平面上距起点的直线距离。** 它通过速度积分重建；我们没有把非完整约束误当成能消掉y的全局位置等式。x、y用于绘制轨迹，控制器不再除以cos(psi)，因此经过90°和270°也无坐标奇异性。

此处积分行程与偏航参考的跟踪不是全局XY轨迹反馈：初始误差、扰动、滑动可以留下世界轨迹偏移。第一版目标是边运动边平衡，非任意轨迹跟踪。

## 2. 建模假设和六坐标出发点

保留原三个刚体：左轮、右轮、车体；无电机转子惯量。地面平坦，两轮理想纯滚动，车体roll不作为独立自由度。本分支使用当前参数已有的左右对称轮，并显式检查两轮质量和惯量相等。

记轮质量m_w，自转惯量J_s=I_Lyy=I_Ryy，横向惯量J_t=I_Lxx=I_Lzz=I_Rxx=I_Rzz。车体惯量I_x,I_y,I_z为质心处主轴惯量。此轴对称假设使轮自转角本身不进入能量，但自转角速度仍产生动能。

原坐标

\[
q=(x,y,\psi,\theta,\alpha_L,\alpha_R)^T
\]

中alpha_L/R为轮相对C系的自转角，不是ROS中轮相对车体的joint角。定义速度投影

\[
u_f=\dot x\cos\psi+\dot y\sin\psi,\qquad
u_\perp=-\dot x\sin\psi+\dot y\cos\psi.
\]

原始精确能量可以紧凑写为：

\[
\begin{aligned}
T_W={}&\frac{m_w}{2}\big[(\dot x-\tfrac d2\omega\cos\psi)^2+(\dot y-\tfrac d2\omega\sin\psi)^2\big]\\
&+\frac{m_w}{2}\big[(\dot x+\tfrac d2\omega\cos\psi)^2+(\dot y+\tfrac d2\omega\sin\psi)^2\big]
+\frac{J_s}{2}(\dot\alpha_L^2+\dot\alpha_R^2)+J_t\omega^2,\\
T_B={}&\frac{m_B}{2}\big[\dot x^2+\dot y^2+l_c^2p^2+l_c^2\sin^2\theta\,\omega^2
+2l_c\cos\theta\,p\,u_f+2l_c\sin\theta\,\omega u_\perp\big]\\
&+\frac12\big[I_y p^2+(I_x\sin^2\theta+I_z\cos^2\theta)\omega^2\big],\\
V={}&m_Bgl_c\cos\theta+\text{constant}.
\end{aligned}
\]

这里的横向项在代入纯滚动后虽为零，**不能在求六坐标拉格朗日算子前删掉**；它会贡献约束投影后的耦合项。

三个约束写成允许速度映射：

\[
\dot q=S(q)u,\quad u=(v,\omega,p)^T,\quad
S=\begin{bmatrix}
\cos\psi&0&0\\ \sin\psi&0&0\\0&1&0\\0&0&1\\
1/r&-d/(2r)&0\\1/r&d/(2r)&0
\end{bmatrix}.
\]

先求六方向算子 `EL = d/dt(∂L/∂qdot) − ∂L/∂q`，再代入 `qdot=S u`、`qddot=S udot + Sdot u` 并左乘S转置。这样消掉理想约束反力，也保留了非完整修正。电机功率仍为

\[
P_m=\tau_L(\dot\alpha_L-p)+\tau_R(\dot\alpha_R-p).
\]

## 3. 包含转向耦合的紧凑精确方程

定义五个系数：

\[
\begin{aligned}
a&=m_B+2m_w+2J_s/r^2,&c&=m_Bl_c,\\
e&=I_y+m_Bl_c^2,&J&=I_z+2J_t+\frac{d^2m_w}{2}+\frac{d^2J_s}{2r^2},\\
D&=I_x-I_z+m_Bl_c^2.
\end{aligned}
\]

则

\[
M(\theta)\begin{bmatrix}\dot v\\\dot\omega\\\dot p\end{bmatrix}
+h(\theta,v,\omega,p)=B_\tau\begin{bmatrix}\tau_L\\\tau_R\end{bmatrix},
\]

\[
M=\begin{bmatrix}
a&0&c\cos\theta\\
0&J+D\sin^2\theta&0\\
c\cos\theta&0&e
\end{bmatrix},\qquad
B_\tau=\begin{bmatrix}1/r&1/r\\-d/(2r)&d/(2r)\\-1&-1\end{bmatrix},
\]

\[
h=\begin{bmatrix}
-c\sin\theta(p^2+\omega^2)\\
(2D\sin\theta\cos\theta\,p+c\sin\theta\,v)\omega\\
-D\sin\theta\cos\theta\,\omega^2-cg\sin\theta
\end{bmatrix}.
\]

其中 `−c sin(theta) omega²`、`c sin(theta) v omega`、`−D sin(theta)cos(theta) omega²`正是本次不能丢掉的转向耦合。绝对偏航角psi没有出现在质量矩阵里，是平坦均匀地面下转动对称性和前向速度参数化的结果，**不代表没有偏航运动**。psi仍决定世界位置重建。

约束后的能量供检查使用：

\[
T_r=\tfrac12av^2+c\cos\theta\,vp+\tfrac12ep^2+
\tfrac12(J+D\sin^2\theta)\omega^2,
\quad V=m_Bgl_c\cos\theta.
\]

实际推导不直接对这个T_r套三个普通位置坐标的拉格朗日方程。代码验证了 `d(T_r+V)/dt = u^T B_tau tau`，以及六坐标乘子法与投影方程的加速度一致。

## 4. 固定非零朝向与持续转向，不是同一种线性化

静止旧分支对应v0=0、omega0=0。固定非零psi0只涉及参考轴旋转，不能用它代替omega0不为零。

持续匀速转弯参考为

\[
l_r=l_0+v_0t,\qquad \psi_r=\psi_0+\omega_0t,\qquad
\theta_r=p_r=0,\qquad \tau_r=0.
\]

这是理想无损模型的匀速运动参考，不是空间固定点。在相对于该运动的误差

\[
\delta X=(l-l_r,\psi-\psi_r,\theta,v-v_0,\omega-\omega_0,p)^T
\]

附近，M0为M(0)，而偏置对theta的导数为

\[
h_\theta|_0=\begin{bmatrix}-c\omega_0^2\\cv_0\omega_0\\-(cg+D\omega_0^2)\end{bmatrix}.
\]

因此A的上右块仍为单位阵，下方仅theta列非零且等于 `−solve(M0,h_theta)`；输入矩阵下半块等于 `solve(M0,B_tau)`。这与先把所有名义速度置零再线性化不同。

代码使用原Q/R，以v0=0.10m/s、omega0=0.40rad/s的移动参考重新求K。这是有模型依据的新分支增益，不是通过乱调原K掩盖方向错误。v0=omega0=0时A、B、K逐项回归到旧导出结果。

起步和停车采用平滑速度参考，过程中不处于恒速trim；第一版没有专门的加速度前馈，允许暂态误差，由非线性数值积分和Gazebo分别验证。这不是任意高速转向的全局稳定性证明。

## 5. 程序与验证入口

- [derive.py](derive.py)：六坐标精确能量、拉格朗日投影、约束零空间、能量功率检查。
- [equations.json](equations.json)：可用于LaTeX输出的公式。
- [design_and_check.py](../../simulation/yaw_motion/design_and_check.py)：移动参考线性化、原模型回归、CARE和精确非线性数值验证。
- [check_projection.py](../../simulation/yaw_motion/check_projection.py)：30个随机状态，用独立9×9乘子系统验证数值动力学。
- [新分支使用与物理验收说明](../../ros2_ws/YAW_MOTION.md)。

旧物理参数l只在适配边界读取；新的运动CSV中l始终是行程。修改真实车参数时，要重新生成新K并检查对称轮假设，不应只改Gazebo质量而保留旧增益。
