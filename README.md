> 当前 `dissymmetry_motor` 分支使用完整车体惯量、CAD 实车参数和高度质心近似；原 `master` 分支保留无惯性积模型。新理论与数学仿真见 [theory/README.md](theory/README.md)。ROS/Gazebo 历史示例配置尚未切换到新参数或新增益。

# 平衡车：局部线性模型与 LQR 数值验证

首次接触数值仿真，请先读 [仿真代码与图表说明](simulation/GUIDE.md)。

## 当前数学仿真入口

阅读 [分场景仿真说明](simulation/README.md)，运行：

```bash
.venv/bin/python simulation/run_all.py
```

`simulation/straight_balance/` 验证局部平衡和前进/后退位置调节；`simulation/turning_balance/` 验证七状态转向跟踪并对比近似/精确模型。新输出位于各分支的 outputs 中。下文旧单文件程序和输出仍保留作回归基准。

## 文件结构

- `theory/README.md`：理论结构、坐标约定、两个场景及阅读顺序。
- `theory/base.py` / `base.ipynb`：共用位姿、速度和角速度。
- `theory/straight_balance/`：静止直立六状态拉格朗日和 LQR。
- `theory/turning_balance/`：偏航运动拉格朗日和七状态轨迹误差 LQR。
- `theory/update_notebooks.sh`：一键重建并执行五本笔记本。
- `theory/kane/`：独立存放的历史凯恩方法实验，不参与当前计算。
- `archive/theory_before_refactor_20260917/`：重构前完整备份。
- `simulation/lqr_simulation.py`：独立数值程序，顶部 Parameters/Settings 可修改。
- `simulation/check_matrix_source.py`：数值公式与最终符号矩阵的三个朝向交叉检查。
- `simulation/check_lagrange.py`：原拉格朗日分支对账，导入路径已更新。
- `simulation/outputs/`：运行报告、机器可读矩阵、CSV、三张图。
- `.venv/`：项目环境。

## 运行

从任意目录运行：

```bash
/home/mechcid/lagarain/.venv/bin/python /home/mechcid/lagarain/simulation/lqr_simulation.py
/home/mechcid/lagarain/.venv/bin/python /home/mechcid/lagarain/simulation/check_matrix_source.py
```

打开笔记本：

```bash
cd /home/mechcid/lagarain
.venv/bin/jupyter lab theory/base.ipynb
```

一键重建并执行全部五本笔记本：`./theory/update_notebooks.sh`。

## 矩阵与坐标核查

优先采用现有符号代码的最终 `M_linear,C_linear,K_linear,B_linear`。
代码的世界 x 约定为 M00 含 sec²(psi0)，M01、M02 含 sec(psi0)，输入首行是 sec(psi0)/r，**不是乘 cos(psi0)**。
本次 psi0=0，所以世界 x 就是名义前进方向坐标。修改 psi0 后 x 仍代表世界 x；不能把它直接重新解释为前向里程。
若确需前向位置坐标 s，在局部固定朝向下令 z_world=diag(cos(psi0),1,1)*z_forward，再对质量、刚度、输入矩阵作一致变换。

## 当前参数与结果

参数仅用于程序跑通，不是实车参数。SI单位：r=.1，d=.4，l=.3，mB=2，mL=mR=.5；ILyy=IRyy=.0025，ILzz=IRzz=.00125，IByy=.08，IBzz=.06，g=9.81，psi0=0。

状态序为 [x,psi,theta,xdot,psidot,thetadot]。
Q=diag(10,5,400,5,2,40)，R=diag(10,10)，u=-K_LQR*X。

- 质量矩阵正定、秩3，条件数约29.45（数值依赖坐标单位/缩放）。
- 可控性秩6。
- 开环极点含 +6.12016；无力矩时0.03 rad俯仰增长，在约0.4223 s达到0.2 rad停止显示。0.2是仿真展示截断阈值，不是经证明的模型有效域。
- 闭环六个极点均为负实部，15秒最大状态分量绝对值约1.38e-6（各分量单位不同）。
- 左右轮峰值力矩各约0.2497 N m；真实驱动可行性尚未验证。
- 额外测试 x/psi/theta 混合初始扰动，避免仅测对称俯仰未激励偏航。
- solve_ivp 与矩阵指数交叉核对；CARE残差检查；符号矩阵源验证。

完整 A/B/K、参数、极点、诊断及仿真配置见 `simulation/outputs/REPORT.md` 和 `results.json`；原始时序见CSV。

## 适用范围

仅验证静止直立点附近的理想连续时间局部线性模型。没有电机饱和、采样延迟、摩擦、传感器噪声或非线性车辆模型。因此闭环稳定结果不等于实车稳定保证。本节描述的数值仿真不包含 ROS2、Gazebo、Pinocchio。使用现有 SciPy 的 CARE 求解器；仅新增 Matplotlib 绘图依赖。

## ROS 2 / Gazebo 物理仿真

新工程见 [ros2_ws/README.md](ros2_ws/README.md)，包含力矩控制模型、控制节点和四项试验工具。系统依赖已安装，已运行真实Gazebo力矩闭环；最终接触假设、失败对照和验收数据见 [ros2_ws/RESULTS.md](ros2_ws/RESULTS.md)。物理验收使用独立CSV，不混用上述理想线性仿真结果。

系统学习入口：[ROS 2仿真项目剖析](ros2_ws/ROS2_PROJECT_ANALYSIS.md)，包括真实启动链路、运行取证、逐文件职责和学习优先级。

2026-09-13：[六问源码解答与交互面板](ros2_ws/ROS2_QUESTIONS_AND_INTERACTION.md)。运行 `ros2_ws/run_interactive.sh` 可打开支持扰动按钮的物理仿真。

## 持续偏航运动独立分支

新增 [两个场景的理论推导](theory/README.md) 和 [balance_yaw 启动/物理验收](ros2_ws/YAW_MOTION.md)。当前数学符号：s 为累计前向行程，l 为车轴到质心距离；历史ROS字段名保留兼容。

理论转向分支现新增七状态 `[x,y,psi,theta,v,omega,p]` 跟踪模型；现有 ROS `balance_yaw` 的六状态里程控制器保留，尚未替换为七状态版本。
