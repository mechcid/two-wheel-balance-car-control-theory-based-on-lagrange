# LQR 快捷计算使用说明

写于：2026年10月20日

场景名称统一为 `origin_static_balance`：目录、配置节、JSON 控制器键和仿真命令行均使用此名称。旧配置中的 `[balance]` 需改名；新导出 schema_version 为 2。此场景是静止直立平衡，恒速直行案例留待后续独立实现。

## 1. 日常只需要修改一个文件、运行一个脚本

在项目根目录打开 `config/lqr.toml`，修改物理参数、Q/R 和参考速度，然后执行：

```bash
cd /home/mechcid/lagarain
.venv/bin/python control/generate_lqr.py
```

配置文件支持 `#` 注释，每个物理参数、权重顺序和设计选项都有中文说明。首次运行包含符号模型构建，可能需要等待；不需要打开 Notebook、运行全部仿真或启动 ROS。

成功后得到：

- `control/generated/controllers.json`：完整精度的控制器参数，供控制模块读取。
- `control/generated/summary.md`：便于直接查看两种 K 的摘要。

同一输出目录再次生成会覆盖上次结果。只有全部启用场景求解成功后才更新 JSON；如果命令失败，旧结果仍然是旧结果，不能当成新配置的输出。核对 JSON 中 `generated_at`、`config_sha256` 和 `config_snapshot`。

保留不同方案可以复制配置，并指定独立输出目录：

```bash
.venv/bin/python control/generate_lqr.py \
  --config config/my_robot.toml --output control/generated/my_robot
```

这里 `my_robot.toml` 需要先由你复制默认配置并修改。相对路径相对于运行命令时的工作目录。

## 2. 修改哪些参数

### 车辆物理参数 `[physical]`

统一使用 SI 单位：质量 kg、长度 m、惯量 kg·m²、角度 rad。

- `r` 是车轮半径，`d` 是两轮中心平面间距，`l` 是车轴中点到车体质心的高度。
- `m_B` 不包含车轮，包含电机、车架、电池；`m_L/m_R` 是各车轮质量。
- `I_Bxx/I_Byy/I_Bzz/I_Bxy/I_Bxz/I_Byz` 构成对称的车体质心惯量矩阵，坐标方向必须与项目刚体系一致。
- 惯量必须在质心处定义；不能把车轴中点处的惯量直接填入。当前 CAD 导出采用正张量记数法，惯性积取反后成为矩阵非对角项；kg·mm² 乘 `1e-6`。
- 当前仍忽略车体质心前后、横向偏移，只保留高度；车轮仍采用左右相同、绕 y 轴轴对称的模型。轮轴惯量是 `I_Lyy/I_Ryy`，径向 `xx=zz`。结构假设改变时需要修改动力学。
- 快捷入口的直立场景要求 `psi0=0`，使 x 表示名义前进方向的位置。

`theory/data/physical.json` 现在是旧 CAD 数值快照，不再是运行配置。修改该 JSON 不会改变新计算。

### 设计方式 `[design]`

```toml
domain = "discrete"
sample_time = 0.005
```

默认按 0.005 s（200 Hz）设计离散控制器。这只是可修改的设计周期，必须与实际控制周期一致，并不表示已完成硬件验证。

- `discrete`：连续误差模型通过精确零阶保持得到 Ad/Bd，使用 DARE 求 K。输出 `A/B` 就是 Ad/Bd，闭环极点必须位于单位圆内。
- `continuous`：使用连续 A/B 和 CARE 求 K。输出 `sample_time=null`，闭环极点实部必须小于零。配置中的 `sample_time` 不参与连续求解。

两种模式都额外保存 `A_continuous/B_continuous`。不能把连续 K 直接当作任意周期的离散 K。

**Q/R 的含义明确区分：**连续模式最小化时间积分代价；离散模式最小化每拍代价之和。本入口不做连续代价的精确积分离散化，也不生成交叉代价项 N。离散模式下直接用填写的 Q/R 作为 Qd/Rd，因此切换模式或周期后应重新生成、检查表现。

### 两个场景的 Q/R

一维列表是对角矩阵的对角线；也支持完整二维矩阵。Q 必须对称半正定，R 必须对称正定。例如输入耦合权重可以写：

```toml
R = [[10.0, 1.0], [1.0, 10.0]]
```

`origin_static_balance.Q` 是 6 项列表或 6×6 矩阵，`turning.Q` 是 7 项列表或 7×7 矩阵，两个 R 都是 2 项列表或 2×2 矩阵。不要在同一配置节重复写同名键。

提高某状态的相对 Q 权重，通常会更重视压低该误差；提高 R 权重，通常更抑制控制力矩。不同状态量单位不同，不能单纯比较权重数字大小判断实际控制强弱。Q/R 并不设置硬力矩上限。

使用 `enabled=false` 可以跳过对应场景；至少启用一个。

## 3. 原点静止直立平衡控制器

输出键：`controllers.origin_static_balance`，K 为 2×6。

| 序号（从 0 起） | 状态 | 意义 | 单位 |
|---|---|---|---|
| 0 | x | 车轴中点沿名义前向的位置 | m |
| 1 | psi | 偏航角 | rad |
| 2 | theta | 俯仰角，直立为 0 | rad |
| 3 | x_dot | 名义前向位置速度 | m/s |
| 4 | psi_dot | 偏航角速度 | rad/s |
| 5 | theta_dot | 俯仰角速度 | rad/s |

以固定位置 x_ref 平衡：

```python
error = state - [x_ref, 0, 0, 0, 0, 0]
tau = -K @ error
# tau[0] 左轮力矩，tau[1] 右轮力矩，单位 N·m
```

偏航角误差应归一到 [-pi, pi)。这是名义朝向 0 附近的局部控制器，不应通过随意更换 psi_ref 来覆盖任意朝向；实际应用可先统一导航坐标的名义朝向。前后位置调节仍然使用固定目标附近的控制。

## 4. 恒速运动／圆周跟踪控制器

输出键：`controllers.turning`，K 为 2×7。

```toml
v0 = 0.1       # m/s
omega0 = 0.4   # rad/s
```

默认对应半径 0.25 m 的左转。omega0 为负表示反向偏航；v0 可为负。omega0=0、v0 非零对应恒速直行。两者同时为零时拒绝七状态设计，请使用 origin_static_balance。

物理状态为 `[x,y,psi,theta,v,omega,p]`，其中 x/y 是世界位置，v 是车体前向速度，omega 是偏航角速度，p 是俯仰角速度。K 作用的是下列**参考朝向坐标中的误差**：

| 序号 | 误差 | 单位 |
|---|---|---|
| 0 | e_parallel，沿参考朝向的位置误差 | m |
| 1 | e_left，参考朝向左侧的位置误差 | m |
| 2 | e_psi，偏航角误差 | rad |
| 3 | e_theta，俯仰角相对 theta_ref 的误差 | rad |
| 4 | e_v，相对 v0 的前向速度误差 | m/s |
| 5 | e_omega，相对 omega0 的偏航角速度误差 | rad/s |
| 6 | e_p，俯仰角速度误差，参考为 0 | rad/s |

参考俯仰角 `reference.theta` 和参考力矩 `reference.tau` 由精确动力学求解，不应强制设为 0。给定轨迹起点 x0/y0 和初始朝向 psi0：

```python
psi_ref = psi0 + omega0 * t
if abs(omega0) > 1e-12:
    x_ref = x0 + v0 / omega0 * (sin(psi_ref) - sin(psi0))
    y_ref = y0 + v0 / omega0 * (cos(psi0) - cos(psi_ref))
else:
    x_ref = x0 + v0 * t * cos(psi0)
    y_ref = y0 + v0 * t * sin(psi0)

dx, dy = x - x_ref, y - y_ref
error = np.array([
    cos(psi_ref) * dx + sin(psi_ref) * dy,
    -sin(psi_ref) * dx + cos(psi_ref) * dy,
    (psi - psi_ref + pi) % (2*pi) - pi,
    theta - theta_ref,
    v - v0,
    omega - omega0,
    p,
])
tau = tau_ref - K @ error
```

这里使用**参考偏航角**旋转位置误差，不能改成当前实测偏航角。修改 v0/omega0 后必须重新生成；不在控制周期中进行符号推导或求 Riccati 方程。恒速参考设计验证的是参考附近的误差恢复，从静止进入运动需要单独规划过渡过程。

## 5. 读取结果与接入边界

```python
import json
import numpy as np

with open('control/generated/controllers.json') as f:
    bundle = json.load(f)
controller = bundle['controllers']['origin_static_balance']
K = np.asarray(controller['K'])
assert controller['domain'] == 'discrete'
Ts = controller['sample_time']
```

JSON 包含 A/B/Q/R/P/K、闭环极点、Riccati 残差、连续模型可控性秩、状态顺序、单位、参考值及原配置快照。`A/B` 总是对应所选设计模式；七状态矩阵属于旋转误差坐标。

力矩正方向采用项目虚功模型的广义关节输入约定，按 `[左轮,右轮]` 排列。控制模块应校准电机正反、编码器和 IMU 坐标方向；K 的输出是 N·m，不能直接当作 PWM、转速或电流。需要执行器的力矩转换和限幅。

生成脚本检查配置、惯量、参考可行性、Riccati 残差和局部闭环稳定性。它不包含硬件延迟、饱和、传感器噪声或轮地接触验证。离散求解假定每个周期内保持输入增量不变。

## 6. 理论展示和仿真仍是独立流程

理论 `.py` 是公共动力学和线性化计算源。快捷入口调用它们；Notebook 展示这些公式和连续 LQR 推导。理论的 `design()` 仍做连续 CARE，读取相同物理参数和 Q/R，不受快捷入口 `domain` 切换影响。

```bash
# 阅读／重建理论展示：会覆盖生成 Notebook 的手工修改
./theory/update_notebooks.sh

# 数学仿真：仍为连续反馈，不能当作新离散 K 的采样闭环验证
.venv/bin/python simulation/run_all.py
```

原有仿真重新求连续 K，未改为消费快捷入口的离散结果。转向仿真的多个参考速度仍定义在 `simulation/turning_balance/run.py`，并不跟随配置的单一 v0/omega0；Q/R 和物理参数来自统一配置。ROS/Gazebo 不自动写入新 K。

改变配置后重新启动脚本。已有 Notebook 会话可能缓存数值动力学，重启内核后再运行。`theory/data/physical.json` 与旧 `simulation/lqr_simulation.py` 的参数不再控制当前理论默认值。

## 7. 本次验证与回归命令

```bash
.venv/bin/python -m unittest discover -s tests -p test_lqr_entry.py -v
```

五项测试覆盖：连续增益与理论接口一致、离散 ZOH 与独立积分一致及采样闭环收敛、完整非对角 Q/R、非法权重及静止七状态拒绝、切换物理参数和参考速度时缓存刷新。默认配置的两套离散结果已实际生成。测试使用理想局部模型，不构成实车验收。
