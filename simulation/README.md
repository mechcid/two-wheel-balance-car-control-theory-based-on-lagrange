# 数学仿真：固定朝向平衡与持续偏航跟踪

当前入口是 `run_all.py`。程序直接调用 `theory/straight_balance` 和 `theory/turning_balance` 的推导及 LQR，不从 ROS 节点复制一套动力学，不启动 ROS/Gazebo，也不写入 ROS 配置。

## 符号约定

- **l**：车轴到车体质心的距离，单位 m。理论符号、Python 变量和物理参数现在都叫 l，无下标。
- **s**：累计前向行程，`ds/dt=v`，单位 m；旧版将行程记为 l，现改为 s。
- x、y：世界 N 系的位置。转向时 `dx/dt=v*cos(psi)`、`dy/dt=v*sin(psi)`，因此 s 不是世界 x，也不是两位置间的直线距离。
- psi、theta：偏航、俯仰角；omega、p 是对应角速度。

静止分支保留 `[x,psi,theta,dx,dpsi,dtheta]` 六状态。转向分支使用 `[x,y,psi,theta,v,omega,p]` 七状态；另积分 s 用于记录和对照，**不把 s 加到七状态反馈里**，也不要求累计行程误差收敛到零。

## 目录和职责

```text
simulation/
  run_all.py                  统一入口，可选场景
  common.py                   solve_ivp、限幅常量、CSV/JSON和图表导出
  straight_balance/
    run.py                    开环、平衡、前进/后退位置调节
    outputs/<case>/           每个实验自己的结果
  turning_balance/
    run.py                    七状态运动跟踪，近似/精确模型对照
    outputs/<case>/<plant>/   每个实验、每种被控模型自己的结果
  results.json / results.md   最近一次全场景数据与可读结果表
  README.md                   本说明
  GUIDE.md                    初学者阅读入口
  lqr_simulation.py           保留的旧六状态回归程序和原始参数定义
  check_matrix_source.py      旧数值矩阵与当前符号矩阵一致性校验
  check_lagrange.py           转到 theory/validate.py 的兼容校验入口
  yaw_motion/                旧入口兼容及六状态ROS动力学对账
  outputs/                   原六状态回归结果，不是新实验输出目录
```

旧仿真脚本及说明备份位于 `../archive/simulation_before_scenarios_20260917/`。新控制场景不覆盖这些历史结果。旧 `yaw_motion/design_and_check.py` 现转到新七状态仿真，已取消其写 ROS 配置的行为；旧 `check_projection.py` 仍可核对现有 ROS 六状态动力学。

## 运行

在项目根目录使用已有 .venv：

```bash
.venv/bin/python simulation/run_all.py
.venv/bin/python simulation/run_all.py --scenario straight
.venv/bin/python simulation/run_all.py --scenario turning
```

两个场景也可分别直接运行：

```bash
.venv/bin/python simulation/straight_balance/run.py
.venv/bin/python simulation/turning_balance/run.py
```

全部运行写 `simulation/results.json` 和 `results.md`；选单个分支写 `straight_results.json` 或 `turning_results.json`，不冒充全套结果。

## 从代码到一次实验

1. `design()` 读取理论导出的矩阵和当前参数，求 CARE 得到 K。每个转向参考速度 v0、omega0 都重新求相应 K，不拿一组与参考运动不匹配的增益遍历全部场景。
2. `reference()` 给出理想参考状态。静止分支是固定位置目标；转向是给定速度和偏航速度的可行匀速轨迹。
3. `control()` 计算误差和反馈力矩。静止直接用状态差；转向先将世界位置误差旋转到参考朝向，再乘 2×7 的 K。
4. `np.clip` 把每轮电机输出限制在 ±1 N·m；同时记录未限幅和实际电机输出。
5. `rhs()` 把当前状态和力矩送入数学模型，求状态导数。
6. `solve_ivp` 自适应积分。输出间隔和最大积分步长均为0.02 s，但反馈在求导时连续计算，**这不是50 Hz数字控制器仿真**，不包含采样保持或延迟。
7. 保存 CSV、JSON、状态和力矩曲线；自动检查积分成功、有限数值及闭环误差。

## 固定朝向实验

| 名称 | 设置 | 预期 |
|---|---|---|
| open_loop | theta(0)=0.03 rad，两轮输入为0 | 俯仰增长；到0.2 rad附近停止，避免把远离局部范围的线性输出误读为真实运动 |
| balance | 同样俯仰扰动，位置参考0 | 六状态回到参考 |
| forward | 位置目标 x=+0.1 m | 先产生短暂俯仰再停在前方 |
| backward | 位置目标 x=-0.1 m | 回到后方目标并恢复直立 |

该分支的被控对象是局部线性模型；前进/后退是固定位置调节，没有强加不连续速度轨迹。默认 psi0=0，因此此分支 x 与初始前向位移一致。

## 偏航运动实验

| 名称 | v0 (m/s) | omega0 (rad/s) | 初始参考偏航 (rad) |
|---|---:|---:|---:|
| left_turn | 0.1 | 0.4 | 0.7 |
| right_turn | 0.1 | -0.4 | -0.6 |
| reverse_turn | -0.1 | 0.4 | 1.0 |
| fixed_heading | 0.1 | 0 | 1.2 |
| turn_disturbance | 0.1 | 0.4 | 0.7 |

每次从参考轨迹附近起步：世界位置误差 (0.01,-0.01) m，偏航误差0.01 rad，俯仰误差0.03 rad，速度为参考速度。它验证运动轨迹附近的小误差恢复，不是在静止时突然要求达到整段匀速轨迹。

每种场景跑两次：

- `small_pitch`：由理论的小俯仰 M_small、h_small 构造被控对象，保留任意偏航和相关速度耦合。
- `exact`：同一个 LQR 控制器作用于理论的精确 M、h。它是近似模型的数值对照，不是另一个重新调优的控制器。

`turn_disturbance` 在8到8.2秒给左右轮各加0.025 N·m的**关节附加力矩**，使用与电机相同的输入映射。它不是车体外力、也不是无量纲的瞬时冲量。积分在这两个时刻分段，避免跳过扰动。

## 图表和数据怎么看

每个结果文件夹中：

- `states.png`：各状态实际曲线与参考虚线。转向中的 x/y/psi 在持续运动，不应趋于0；应观察是否贴近参考。theta应趋于0。
- `torques.png`：请求力矩、限幅后电机力矩，以及附加扰动力矩。灰线表示电机限幅。扰动力矩是额外输入，不包含在电机限幅内。
- `path.png`（转向）：世界 x-y 轨迹，坐标轴等比例。圆弧不代表失稳，是指定的偏航运动。
- `comparison.png`（每个转向场景的父目录）：精确与近似模型的俯仰，以及两模型位置之间的欧氏距离；这是模型间的差异，不是相对参考的跟踪误差。
- `trajectory.csv`：时间、状态、参考、世界状态误差、原始/实际电机力矩、扰动力矩。转向反馈使用的前两项旋转位置误差与此处的世界误差有区别。
- `result.json`：当前参数对应的 A/B/Q/R/P/K、可控性秩、闭环极点、末端误差、最后2秒误差峰值、力矩峰值和限幅采样比例。转向 A/B/K 属于**参考旋转误差坐标**，不能直接当作世界坐标常矩阵使用。

CSV中 x/y/s 单位 m，角度 rad，v和dx单位 m/s，omega/p/dpsi/dtheta单位 rad/s，力矩 N·m。不同单位的误差应逐项读；程序的最大绝对分量只用作宽松的自动检查，不是统一物理范数。

## 如何修改

- 车辆参数沿用 `lqr_simulation.py` 的 `Parameters`，由 `theory/parameters.py` 读取；I_Bxx读取现有 `physical.yaml`。这些仍是示例值，不是实车测量值。
- 静止权重沿用原 Settings；转向权重在 `theory/turning_balance/lqr.py` 的 Q_WEIGHTS/R_WEIGHTS，分别也可通过 design 参数传入。
- 修改 `straight_balance/run.py` 的 target/初值，设置位置调节实验。
- 修改 `turning_balance/run.py` 的 scenarios 设置恒定参考速度、偏航速度、初始朝向。
- 修改 `common.py` 的 TORQUE_LIMIT、DT设置电机限幅和积分/输出间隔。
- 改动力学应修改 theory，对应场景始终读取该来源。更新理论笔记本使用 `./theory/update_notebooks.sh`。

## 本次结果与边界

四个静止实验和五个转向场景×两种模型均完成。闭环实验收敛检查通过；所有闭环电机峰值均低于1 N·m，未触发限幅。fixed_heading 的横向恢复比转弯更慢，35秒末仍有约1.35e-4的最大状态误差；报告中保留实际结果，不写成精确等于0。

开环不稳定是预期结果，不应套闭环收敛标准。数据不包含 ROS、Gazebo、轮地接触求解、传感器、采样延迟；因此不能据此声称实车或新的七状态Gazebo闭环已通过。
