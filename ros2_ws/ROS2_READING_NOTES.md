# 读这套工程：三条数据链、一个增益、一个被控对象

这个工程里有三条互不相交的数据链：**力矩下行**、**状态下行**、**扰动注入**。控制节点只持有一个 2×6 的增益矩阵 `K`，它不认识 Gazebo，也不认识接触。而被控对象不是 `theory/` 里那套带约束的模型，是一个用摩擦接触**近似**滚动约束的多体仿真。

把这三件事分开看，代码里就没有含糊的地方了。三条链之间唯一的共享量是 `/clock` 和 `base_link` 这个实体名——排查问题时第一步永远是先确认自己在哪条链上。

---

## 0. 全局图

```text
力矩链（不经过 parameter_bridge）
  X, K  →  u = clip(-K(X - ref))  →  /wheel_effort/commands  (Float64MultiArray)
        →  JointGroupEffortController  →  命令接口 left/right_wheel_joint/effort
        →  GazeboSimSystem::write()  →  Gazebo 关节力矩  →  物理引擎

状态链（经过 parameter_bridge）
  Gazebo IMU 传感器  ─┐
  OdometryPublisher  ─┴→ GZ 消息 → bridge → ROS 消息 → state.extract → X
  joint_state_broadcaster → /joint_states →（只进 CSV，不进 X）

扰动链（经过 parameter_bridge，方向相反）
  面板按钮 → Interaction.pulse → /world/balance_world/wrench/persistent
          → ApplyLinkWrench → base_link 上的 world 外力 / 外力矩
```

轮驱动力矩走的是 **ros2_control 进程内接口**，不是 ROS 话题到 GZ 话题的翻译；传感器和扰动才走 `parameter_bridge`。三条链在节点列表里长得像，在机制上完全是三回事。

---

## 1. 力矩链：从 K 到关节力矩的五跳

每一步的负责人不同，这是读代码时最容易糊掉的地方。

| 跳 | 载体 | 谁在做 |
|---|---|---|
| 1. 算 u | Python 变量 | `controller.py:174` |
| 2. 发布 | ROS 话题 `/wheel_effort/commands` | `controller.py:83` 建发布器，`:133` 发布 |
| 3. 收命令 | 控制器插件内部 | `JointGroupEffortController`，由 spawner 激活 |
| 4. 写接口 | 内存中的命令槽 | `left/right_wheel_joint/effort`，见 `balance.urdf.xacro:259` |
| 5. 变关节力 | Gazebo 组件 | `GazeboSimSystem` 的 write 路径 → `JointForceCmd` |

第 4 跳到第 5 跳的边界在上游 C++ 库里（`libgz_ros2_control-system.so`），不在这个仓库里。仓库里能看到的是接口声明和插件配置：

```xml
<!-- balance.urdf.xacro:259 -->
<command_interface name="effort">
    <param name="min">-5</param>
    <param name="max">5</param>
</command_interface>
```

```xml
<!-- balance.urdf.xacro:245 -->
<hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</hardware>
```

### 1.1 一个必须记住的顺序契约

`/wheel_effort/commands` 的 `data` 数组长度是 2，第 0 项给谁、第 1 项给谁，**由 `controllers.yaml:10` 的 joints 列表决定**：

```yaml
wheel_effort:
  ros__parameters:
    joints: [left_wheel_joint, right_wheel_joint]
```

控制器发的是 `[τ_L, τ_R]`——K 的第 0 行给左轮、第 1 行给右轮。这个对应关系在话题消息里看不出来，它是两处配置之间的隐式约定。改 `joints` 的顺序而不改 K 的行序，系统不会报错，会继续用左右颠倒的增益"工作"下去。

### 1.2 三层限幅，但只有一层会触发

| 层 | 位置 | 值 |
|---|---|---|
| 软件限幅 | `controller.py:175` 的 `np.clip` | ±1 N·m（`lqr.yaml:28`） |
| 命令接口界限 | `balance.urdf.xacro:260` | ±5 N·m |
| URDF 关节 limit | `balance.urdf.xacro:205` | effort=5 |

力矩在**参数层**就被截到 ±1，永远到不了接口的 ±5。所以看 CSV 里的 `saturated` 列时，它反映的是"K 算出来的原始值有没有超过 1 N·m"，而不是接口限制。理想线性仿真的峰值力矩约 0.25 N·m（`simulation/outputs/REPORT.md`），1 N·m 是给非线性接触留的余量——余量够不够，只能从实测 CSV 的饱和比例看。

CSV 的 `tau_L/tau_R` 列记的是**发出的命令**，不是实测作用力矩。命令到物理效果的验证要靠关节速度、姿态响应和 Gazebo 日志，不能只看话题有没有发出去。

---

## 2. 状态链：X 的六个分量各来自哪里

X 是控制侧的契约，不是 Gazebo 的某个变量：

```
X = [x, ψ, θ, ẋ, ψ̇, θ̇]
```

| 分量 | 数据源 | 加工者 |
|---|---|---|
| x | `Odometry` 的 `pose.position` | `state.py` 投影到固定名义前向 |
| ψ | `IMU` 的 `orientation` | `state.py` 取相对名义朝向的偏航**误差** |
| θ | `IMU` 的 `orientation` | `state.py` 按 ZYX 提取 |
| ẋ | `Odometry` 的 `twist.linear` | `state.py` 转回 world 再投影 |
| ψ̇、θ̇ | `IMU` 的 `angular_velocity` | `state.py` 体轴角速度 → 欧拉角速率 |

两个来源是**分开的两条路**，不存在"把 Gazebo 的广义坐标数组改名成 IMU"这回事：

- **姿态**来自真正的仿真传感器。`balance.urdf.xacro:87` 在 `base_link` 上挂了一个 IMU，姿态参考系按 `custom_rpy parent_frame="world"` 配置，所以 `orientation` 是相对 world 的绝对姿态。传感器逻辑由 world 里的 `gz-sim-imu-system` 插件推进（`balance.sdf:46`）。
- **位置和速度**来自 `OdometryPublisher`（`balance.urdf.xacro:300`），它输出模型的原地位姿——是理想里程计，不是轮编码器估计。README 里注明其速度由位姿差分并平滑，**不能当无延迟的速度传感器用**。

`joint_state_broadcaster` 发布的轮角/轮速目前只进 CSV，不参与 X 的构造。

### 2.1 姿态角提取

`state.py:20` 按 ZYX 约定从旋转矩阵取角：

```python
pitch = math.asin(clip(-R[2, 0]))      # θ
roll  = math.atan2(R[2, 1], R[2, 2])   # φ
yaw   = math.atan2(R[1, 0], R[0, 0])   # ψ
```

### 2.2 体轴角速度 → 欧拉角速率

这一步经常被跳过，但三自由度模型里 θ̇ 和 ψ̇ 是欧拉角速率，不是 IMU 的 `ωy`、`ωz`。完整映射是

```
[φ̇]   [1  sinφ·tanθ  cosφ·tanθ] [ωx]
[θ̇] = [0     cosφ       −sinφ  ] [ωy]
[ψ̇]   [0  sinφ/cosθ   cosφ/cosθ] [ωz]
```

`state.py:26` 只保留了第二、三行：

```python
pitch_rate = cos(roll)*omega.y - sin(roll)*omega.z
yaw_rate   = (sin(roll)*omega.y + cos(roll)*omega.z) / cos(pitch)
```

两处值得注意：

- **第一行被丢掉了**——三自由度模型里没有滚转。但 `roll` 本身仍然要算，因为 θ̇、ψ̇ 的系数里有它。
- 分母有 `cos(pitch)`。θ 接近 ±π/2 时 ψ̇ 会爆，这是 ZYX 参数化本身的问题，不是实现问题。三自由度模型在直立点附近工作，所以不构成实际风险。

### 2.3 两个方向约定

- **x 是固定名义前向的投影**，不是机体系前向：`forward = [cos(heading), sin(heading), 0]`（`state.py:32`），`heading` 是启动参数 `nominal_heading`（默认 0），不随车体转动。
- **ψ 是误差**：`yaw_error = atan2(sin(yaw-heading), cos(yaw-heading))`，用 `atan2` 形式是为了在 ±π 处不跳变。

速度那边多做了一步：`OdometryPublisher` 的 `twist` 按子坐标系给分量，所以先 `R_odom @ v_local` 转回 world，再投影（`state.py:39`）。

---

## 3. K：它从哪来，长什么样

控制节点里唯一的"模型知识"是 `lqr.yaml:6` 那 12 个数。它来自 `simulation/outputs/results.json` 的 `matrices.K_LQR`，是 `theory/` 里手推模型在直立静止点线性化后的 LQR 解——**在仿真里没有重算过，也没有重调过**。

### 3.1 K 可以从纸面矩阵精确复算

`simulation/outputs/REPORT.md` 给出了 M0、K0、B0 和 Q、R。用

```
A = [[0, I], [−M0⁻¹K0, 0]],  B = [0; M0⁻¹B0],  K = R⁻¹BᵀP
```

复算出来的 K 与 `lqr.yaml` 里的逐位一致，开环极点 `±6.12016` 和闭环极点也与 REPORT 一致。也就是说这条链是完整闭合的：**改 `physical.yaml` 里的参数不会自动更新 K**，K 是上一次离线求解的结果快照。

M0 的每个元素都能对上手推量，这本身是对模型的交叉检查：

| 元素 | 值 | 闭式 | 代入 |
|---|---|---|---|
| M0[0,0] | 3.5 | m_B+m_L+m_R + 2I_Lyy/r² | 3.0 + 0.5 |
| M0[0,2] | 0.6 | m_B·l | 2.0 × 0.3 |
| M0[1,1] | 0.1225 | I_Bzz + (m_L+m_R)(d/2)² + I_Lzz+I_Rzz + 2I_Lyy(d/2)²/r² | 0.06+0.04+0.0025+0.02 |
| M0[2,2] | 0.26 | I_Byy + m_B·l² | 0.08+0.18 |

M0[1,1] 里那个 `2I_Lyy(d/2)²/r²` 项是轮子转动惯量经差动约束折算过来的，它证明约化用的是

```
ω_L = (ẋ − ψ̇·d/2)/r,   ω_R = (ẋ + ψ̇·d/2)/r
```

即**理想纯滚动**（轮速被约束完全决定，不是独立广义坐标）。

B0 的三行同样有直接物理含义：

| B0 行 | 数值 | 含义 |
|---|---|---|
| x | [10, 10] | 1/r，力矩化成轮缘切向力 |
| ψ | [−2, 2] | ±d/(2r)，差动驱动产生偏航 |
| θ | [−1, −1] | 电机对车体的反作用力矩 |

注意 `I_Bxx = 0.1` 在这个模型里**没有被用到**（M0 里不出现），README 也已经注明它是为了 URDF 完整性补的示例值。

### 3.2 K 的两行只差两个元素

```
row L: [-0.7071068, -0.5,       -8.3245030, -1.3793526, -0.3614208, -2.0446623]
row R: [-0.7071068, +0.5,       -8.3245030, -1.3793526, +0.3614208, -2.0446623]
                             ↑ ψ 列                        ↑ ψ̇ 列
```

只有 ψ 和 ψ̇ 两列变号。所以：

- **共模** `(τ_L+τ_R)/2` 只由 x、θ、ẋ、θ̇ 决定 —— 前后平衡通道
- **差模** `(τ_L−τ_R)/2` 只由 ψ、ψ̇ 决定 —— 偏航通道

差模那一支可以单独算清楚。由 K 得 `τ_L − τ_R = ψ + 0.72284·ψ̇`；B0 的 yaw 行给出广义力 `−2(τ_L−τ_R)`，再经 `M0⁻¹` 的 yaw 分量 `1/0.1225 = 8.1633` 放大，得到

```
ψ̈ = −16.3265·(τ_L − τ_R) = −16.3265·ψ − 11.8015·ψ̇
```

这个二阶子系统的极点是 `−10.2010` 和 `−1.6005`，与 REPORT 里闭环极点中的两个**逐位吻合**。吻合不是巧合：M0 在 ψ 行列上块对角，前后通道和偏航通道在这个模型里完全解耦，所以偏航那对极点就是上面这个二阶系统的根。

这也是对称车辆应有的结构——如果 K 的两行在 ψ 列上符号相同，差模就退化成没有偏航反馈，偏航通道只剩开环。

### 3.3 三个失效闩锁

这三条都在 `controller.py` 里，而且在文档里很容易被漏掉：

| 条件 | 位置 | 行为 |
|---|---|---|
| 三个话题未齐，或任一消息超过 **0.05 仿真秒**未更新 | `controller.py:141` | 发 `[0,0]` 并直接返回 |
| `\|θ\| > 0.6` rad（约 34°） | `controller.py:158` | 发零力矩、flush、报错、置 `finished`，**此后永久零力矩直到重启** |
| 参数非法（K 非有限、limit ≤ 0 等） | `controller.py:54` | 启动即抛异常 |

第一条是"失去观测就放弃控制"，不是保持最后指令。第二条是一旦跌倒就锁死，不会自己恢复——交互面板还有一道更早的软门槛（`interaction.py:55`，`|θ| > 0.3` rad 时拒绝新的扰动），所以正常操作中先撞到的是 0.3 那道。

### 3.4 `probe_torque` 是开环开关，不是扰动

```python
# controller.py:174
raw = self.probe if np.any(self.probe) else -self.K @ (X - self.ref)
```

只要 `probe_torque` 有一个非零分量，**整个反馈项被替换掉**，LQR 完全不参与。这是给 `run_case.py left_probe/right_probe` 做输入方向标定用的（给单轮恒力矩、看偏航加速度符号），不是"加一个扰动"。正常试验里必须全零——README 也专门强调了这一点。

`disturbance:=true` 是另一回事：走的是第 7 节的外力链，不碰力矩接口。

---

## 4. 被控对象：仿真里的车和纸面模型差在哪

这是整套系统里最需要意识到的一处落差。

纸面模型在三条纯滚动约束下把 6 个广义坐标约化成 3 个自由度，再用 `M0`、`K0`、`B0` 线性化得到 K。**Gazebo 里没有这三条约束方程。** 它有的是：

- 轮子用**球形碰撞体**（`balance.urdf.xacro:161`），半径等于轮半径，近似理想窄轮的点接触；可视和惯量仍是圆柱轮。
- 接触面摩擦系数 1.0（`balance.urdf.xacro:216`，地面同值 `balance.sdf:112`）。
- 关节阻尼和摩擦都是 0（`balance.urdf.xacro:208`），所以轮上唯一的力矩就是命令力矩。

滚动不是被"约束"出来的，是**摩擦接触碰巧维持**的。这意味着：

1. 仿真允许打滑和侧向擦动，只是摩擦系数足够大时看不出来。K 是在"纯滚动恒成立"的前提下算的，物理引擎不保证这个前提。
2. `cylinder_contact` 的对照试验（`results/cylinder_contact`）证明了这一点：换成有限宽圆柱碰撞后偏航受阻，失败数据保留着。球形碰撞是为了让点接触成立而做的**局部理想化**，代价是失去了轮宽方向的真实行为——它不是"更准确的模型"，是"和纸面假设更一致的模型"。
3. 大范围运动、真实宽轮轮胎不在这套验证的覆盖范围内。

换句话说：K 是一个**局部线性模型上的增益，用来控制一个非线性接触系统**。两者在直立点附近一致，偏离越远越不可信。这也解释了为什么会有 0.6 rad 的停机闩锁。

另一个方向的落差在测量侧：IMU 和 Odometry 都是仿真理想传感器。位置和速度来自 `OdometryPublisher` 的同一次位姿输出，不是独立的轮编码器估计，也不含噪声、延迟和标定误差。**换到实车时，`state.py` 是替换边界**——但仅仅保持消息类型相同不够，frame、单位、时间戳和参考点都必须一致。

---

## 5. 坐标系与 TF 的现状

TF 没有消失，只是当前闭环不消费它。

```
Gazebo 关节状态 → GazeboSimSystem 状态接口
                → joint_state_broadcaster → /joint_states
                → robot_state_publisher → /tf
                       ├─ base_link → left_wheel
                       └─ base_link → right_wheel
```

实际存在的只有这两条边（`learning_evidence/tf.json`）。**没有 `world → base_link` 的 ROS TF**，也没有 C 系、质心系或 IMU 外参的 TF frame。Odometry 消息里带 `world`/`base_link` 标签，不等于它广播 TF。

三点容易混的地方：

- **`robot_state_publisher` 不是模型导入器。** 它接收 `robot_description` 并做正运动学，把 `origin` / `axis` 加实时关节角变成 TF。真正把模型送进 Gazebo 的是 `create` 节点。
- **`create` 同时承担实体生成和初始姿态**（`balance.launch.py:42`）。`-P`、`-Y` 设置的是**整个模型的初始位姿**，不是给某个 joint 赋初值。
- **`base_link` 原点在车轴中心，但它的姿态随车体俯仰**（`balance.urdf.xacro:34` 的 link 没有姿态约束，质心在 inertial origin `(0,0,l)`）。它不是纸面上那个只随偏航转动的 C 系。轮关节角是相对车体的，在共轴约定下 `q_L = θ_L − θ`。质心不自动生成 TF frame。

当前控制节点直接在 `state.py` 里做旋转，不订阅 `tf2`。将来接入多个有相对安装角的传感器、或用 RViz 以 world 为参考显示时，TF 链才会进入状态处理流程。

---

## 6. 启动时序：为什么需要 `startup` 节点

世界是**暂停**启动的。`gz sim` 的 `-r/--run` 才是"启动后立即运行"，而 `balance.launch.py:27` 给的是 `['gz','sim','-s','-v','3', world]`——**没有 `-r`**，所以 `/clock` 停在 0，物理不推进。

这不是省电那么简单：`spawner` 请求管理器加载并切换控制器时，管理器可能等待一次物理更新，而世界一直暂停就形成死锁。因果链是：

```text
展开 Xacro → robot_description 可读 → create 生成实体
→ Gazebo 加载控制插件 → controller_manager 服务出现
→ spawner 请求加载并激活 → 确认两轮 effort 被认领
→ 解除暂停 → /clock 与传感器开始推进 → LQR 持续反馈
```

`startup.py` 就是在补最后两步。两个设计点值得记：

- **它用墙钟定时器**（`startup.py:38`），因为暂停世界里基于仿真时间的定时器根本不会触发。
- **推进被限量并且可以不等服务返回**：`startup.py:76` 在等待 `ListControllers` 响应的同时就提交单步（在子进程里调 `gz service`，避免阻塞 ROS 执行器），启动阶段总共只允许推进 0.15 仿真秒（`startup.py:88`），超预算就直接报错。整个流程有 120 秒墙钟上限。

`auto_start` 和 `panel` 都可以关掉。批量测试（`run_case.py`）明确传 `gui=false panel=false auto_start=false`，由自己管理单步和等待——**不能让两个启动监督者同时控制 world**。

`run_interactive.sh` 是交互路径的入口：设置环境、把日志写进带时间戳的目录，再 `exec ros2 launch`。

---

## 7. 扰动链：面板按钮不是电机力矩

面板按钮不碰两个轮子的力矩接口，也不改 K。它给车体加一个**世界系的外力 / 外力矩**：

```python
# interaction.py:61
msg = EntityWrench()
msg.entity.name = 'balance::base_link'
msg.entity.type = Entity.LINK
msg.wrench.force.x = magnitude   # 或 msg.wrench.torque.z
```

话题 `/world/balance_world/wrench/persistent` 经 bridge 进 GZ，由 world 里的 `ApplyLinkWrench` 插件（`balance.sdf:50`）作用到 `base_link`。清除走的是另一个话题 `/wrench/clear`，用 `Entity` 消息。

| 按钮 | 实际作用 |
|---|---|
| 向前 / 向后推 1 N | world ±x 方向力，1 N |
| 正 / 负偏航扰动 | world ±z 方向力矩，0.05 N·m |
| 清除外力 | 撤销该实体的持续外力请求 |
| 暂停 / 继续 | 调 Gazebo world control；暂停前先清外力 |

几个细节：

- **脉冲时长按仿真时间计**（`interaction.py:69` 记录 `self.sim_time + duration`），检查由墙钟定时器做（`interaction.py:89`）。这样仿真变慢不会改变物理作用时长。
- `pulse` 有一组前置检查（`interaction.py:53`）：仿真在跑、状态已到、`|θ| ≤ 0.3`、bridge 订阅端已就绪，并且**只接受四组预置量**（±1 N / ±0.05 N·m）。这是一组人工小扰动，不是通用外力接口。
- 面板本身只做显示和转发，`LQR` 在另一个进程里照常跑。窗口大小按桌面 DPI 缩放（`panel.py:17`），曲线画最近 10 仿真秒的 θ 和 ψ。

`launch` 的 `disturbance:=true` 走的是**同一个** persistent wrench 话题，在 `t = 3 s` 施加 +x 1 N、`t = 3.1 s` 清除（`controller.py:197`）。它和面板是同一机制的两个入口——**同时开会让两个发布者互相覆盖**，日常交互建议只用面板。

---

## 8. 运行

```bash
# 首次或改代码后
source /opt/ros/jazzy/setup.bash
cd /home/mechcid/lagarain/ros2_ws
colcon build --symlink-install --packages-select balance_sim

# 图形桌面：Gazebo + 扰动面板 + 自动就绪
/home/mechcid/lagarain/ros2_ws/run_interactive.sh

# 无桌面（后台验证）
/home/mechcid/lagarain/ros2_ws/run_interactive.sh gui:=false panel:=false
```

默认初始俯仰 0.03 rad。先看它站稳，再点扰动。同一个脉冲未结束时需要等完成或先清除。不要同时开多个同名 world——日常交互前先关掉上一次启动。

---

## 9. 当前边界

明确没做到的，和做到的同样重要：

- 参数全部是**程序示例值**（`physical.yaml` 顶部注明），不是实车参数；`I_Bxx` 和 `wheel_width` 是为 URDF 完整性补的。
- K 只在 `ψ0 = 0` 的直立点线性化。改 `physical.yaml` 里的 `psi0` 而不重算矩阵，所有旧解释失效。
- 接触是理想窄轮点接触（球碰撞），不对应真实宽轮轮胎的转向行为；宽圆柱对照试验是失败的。
- 传感器是理想仿真传感器，无噪声、无延迟、无标定。
- 纯 Python 异步控制，不是硬实时；200 Hz 是设置值，实际采样间隔要看 CSV。
- 未解决：`world → base_link` TF 缺失、状态时间同步、参数热更新。这些与交互入口是不同任务，不应该被"面板能用了"掩盖。
- 前进/后退需要连续参考轨迹和前馈；大角度偏航需要更新局部参考朝向和处理角度展开——都不能直接把大角度塞进静止点的 K。

---

## 附：文件职责

| 文件 | 职责 |
|---|---|
| `balance_sim/controller.py` | 200 Hz 反馈、限幅、CSV、`disturbance` 测试路径 |
| `balance_sim/state.py` | 传感器 → 六状态 |
| `balance_sim/startup.py` | 暂停世界的就绪处理与限量单步 |
| `balance_sim/interaction.py` | 面板的 ROS 端，外扰期限与暂停控制 |
| `balance_sim/panel.py` | Tk 界面、按钮、实时曲线 |
| `launch/balance.launch.py` | 全部节点与世界 |
| `description/balance.urdf.xacro` | 刚体、碰撞、IMU、力矩接口、Gazebo 插件 |
| `config/*.yaml` | 物理参数、K 与限幅、控制器、桥接 |
| `worlds/balance.sdf` | 重力、地面、接触、传感器与外力系统插件 |
| `tools/run_case.py` | 批量物理试验，自行管理启动 |
