# 从理论推导到仿真：我的 LQR 两轮平衡车

这个项目起于一个很具体的想法：做出一辆属于自己的、用 LQR 控制的两轮平衡车。为了把它做出来，我一边实践，一边补上需要的理论，也把学习过程中的推导、代码和实验结果逐步整理到了这里。

项目涉及分析力学、拉格朗日方程、控制理论、数学仿真，以及 ROS 2 / Gazebo 仿真。如果你也对这些内容感兴趣，希望这份记录能为你提供一条学习路径：从“怎样描述小车的运动”开始，逐步走到“怎样计算控制器，再观察它能否让小车保持平衡”。

我希望这个项目同时做好两件事：**把 LQR 增益矩阵 K 的求解过程一步步展示出来**，以及**提供一个修改配置、运行脚本就能得到 K 的实用入口**。这两部分各有入口，共用底层动力学模型。

## 关于这份学习记录

也想坦诚说明：项目中有不少内容是在 AI 的辅助下完成的，其中有些部分我还没有完全理解，尤其是 `simulation` 中的数学验证，目前尚未经过我本人的完整审查。程序中的自动检查和已有实验结果，不能代替这部分理解与核对。

如果你在阅读时发现推导、代码或说明有问题，欢迎指出；如果有些地方写得不够清楚，也请多包涵。我会随着学习和实践继续修正、补充。

后续我计划上传学习过程中的手写笔记。字可能不太好看，但我会尽量用容易理解的话，从实际需求出发，记录自己遇到了什么问题、为什么需要这些知识，以及如何一步步解决。希望结合这些笔记，刚接触相关知识的读者也能更容易地跟上。

## 项目由哪些部分组成

| 目录 | 主要内容 | 适合什么时候看 |
|---|---|---|
| [`theory/`](theory/README.md) | 运动学、能量、拉格朗日方程、状态空间与 LQR 推导 | 想理解模型和 K 是怎样得到的 |
| [`config/`](config/lqr.toml) | 带中文注释的物理参数、Q/R、参考速度和控制周期 | 想修改车辆参数或控制器设置 |
| [`control/`](control/generate_lqr.py) | 读取配置、计算并导出控制器参数 | 想直接生成当前参数对应的 K |
| [`simulation/`](simulation/README.md) | 数学模型上的闭环仿真、状态曲线和力矩记录 | 想观察给定控制器下系统的响应 |
| [`ros2_ws/`](ros2_ws/README.md) | ROS 2 / Gazebo 模型、控制节点与物理仿真实验 | 想进一步了解仿真环境中的控制流程 |
| [`doc/`](doc/) | 理论理解、使用说明和阶段性记录 | 想查阅补充解释和操作方法 |
| [`archive/`](archive/) | 历史代码与重构前的备份 | 想回看早期实现 |

当前理论包含两个场景：

- **`origin_static_balance`**：原点静止直立附近的六状态模型与 LQR。已有前后位置阶跃测试用于静止目标调节，不代表已经完成恒速直行平衡案例。
- **`turning_balance`**：恒定参考速度下的七状态轨迹误差模型与 LQR，主要用于圆周运动跟踪。

## 当前分支与实现进度

当前 `dissymmetry_motor` 分支使用 CAD 导出的车体质量和完整惯量矩阵，保留惯性积；车体质心采用只保留高度的近似。原 `master` 分支保留无惯性积模型。

物理参数和控制设置统一读取 [`config/lqr.toml`](config/lqr.toml)。`theory/data/physical.json` 保留为历史 CAD 参数快照，不再是当前计算入口。

**理论计算、数学仿真和 ROS/Gazebo 工程的进度并不完全一致：**

- 快捷计算支持连续与离散 LQR，并导出矩阵、状态顺序、单位和参考值。
- 当前数学仿真仍采用连续反馈，不直接验证快捷入口导出的离散控制器。
- ROS/Gazebo 保留历史示例参数和控制器，尚未切换到本分支的 CAD 参数与新七状态控制器。

已有仿真记录描述的是各自参数和模型下的结果，不能直接视为当前实车控制效果。

## 快捷计算 K

以下命令均在项目根目录执行，使用项目本地的 `.venv` 环境。首次使用请准备 Python 3.12 或更高版本，再创建环境并安装依赖：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirement.txt
```

[`requirement.txt`](requirement.txt) 记录了当前项目使用的 Python 直接依赖及版本，覆盖理论计算、数学仿真和 Notebook。`.venv` 是本地环境，不随仓库分发；`simulation/requirements.txt` 转向同一份清单。ROS 2 / Gazebo 及相关系统依赖需另按 [ROS 工程说明](ros2_ws/README.md) 配置。

先修改带中文注释的 [`config/lqr.toml`](config/lqr.toml)，再运行：

```bash
.venv/bin/python control/generate_lqr.py
```

结果写入：

- `control/generated/controllers.json`：完整精度的 A/B/Q/R/P/K、状态顺序、单位、参考值、检查结果和配置快照。
- `control/generated/summary.md`：便于阅读的 K 矩阵摘要。

支持对角权重或完整 Q/R 矩阵。当前配置选择离散设计，控制周期为 `0.005 s`（200 Hz）；应根据实际控制周期调整，并重新生成。

具体参数含义、控制律和接入方式见 [LQR 快捷计算使用说明](doc/LQR快捷计算使用说明.md)。

## 一步步阅读理论推导

建议从 [理论目录说明](theory/README.md) 开始，按下面的顺序阅读：

```text
位姿与坐标变换 → 质心速度和角速度 → 动能与势能
→ 拉格朗日方程与约束 → 工作点或参考运动
→ 状态空间与线性化 → 给定 Q/R，求解 P 和 K
```

对应文件：

1. [`theory/base.ipynb`](theory/base.ipynb)：共用的坐标、运动学与能量表达。
2. [`theory/origin_static_balance/`](theory/origin_static_balance/)：静止直立场景的 `lagrange.ipynb` 和 `lqr.ipynb`。
3. [`theory/turning_balance/`](theory/turning_balance/)：运动跟踪场景的 `lagrange.ipynb` 和 `lqr.ipynb`。

使用已安装 JupyterLab 的项目环境打开笔记本：

```bash
.venv/bin/jupyter lab theory/base.ipynb
```

需要根据当前代码和配置重新生成、执行五本笔记本时，运行：

```bash
./theory/update_notebooks.sh
```

`.py` 文件是计算源，Notebook 用于展示推导和结果。**重建会覆盖生成笔记本中的手工修改**，请先保存需要保留的内容。理论 Notebook 继续展示连续时间 LQR；快捷入口的连续／离散选择不会改变这一点。

## 运行数学仿真

首次阅读建议先看 [仿真代码与图表说明](simulation/GUIDE.md)，再看 [分场景运行说明](simulation/README.md)。

```bash
# 运行全部数学仿真场景
.venv/bin/python simulation/run_all.py

# 只运行原点静止直立模型相关实验
.venv/bin/python simulation/run_all.py --scenario origin_static_balance

# 只运行转向跟踪实验
.venv/bin/python simulation/run_all.py --scenario turning
```

结果保存在各场景的 `outputs/` 目录，包括状态曲线、力矩曲线、CSV 和 `result.json`。运行全部场景时，还会更新 `simulation/results.json` 和 `simulation/results.md`。

原点静止场景使用局部线性模型；转向场景对比精确非线性模型与小俯仰近似模型。具体参考速度、初始扰动、限幅和检查条件以 [仿真说明](simulation/README.md) 为准。

## ROS 2 / Gazebo 仿真

ROS/Gazebo 工程保留了力矩控制模型、控制节点、扰动工具和历史实验记录，可以从以下文档进入：

- [工程说明与运行方式](ros2_ws/README.md)
- [ROS 2 仿真项目剖析](ros2_ws/ROS2_PROJECT_ANALYSIS.md)
- [实验结果与接触模型说明](ros2_ws/RESULTS.md)
- [六问源码解答与交互面板](ros2_ws/ROS2_QUESTIONS_AND_INTERACTION.md)
- [历史偏航运动控制器说明](ros2_ws/YAW_MOTION.md)

其中 `balance_yaw` 仍是历史六状态里程控制器，尚未替换为理论目录中的七状态轨迹误差控制器。理解或复现实验时，请同时核对对应的模型参数、控制器版本和接触假设。

## 历史实现与结果

`simulation/lqr_simulation.py` 和 `simulation/outputs/` 保留了早期六状态示例，使用的示例参数不同于当前 CAD 参数。它们用于回看和回归对照，日常计算请使用上面的统一入口。

早期数值结果见 [历史仿真报告](simulation/outputs/REPORT.md)；凯恩方法实验保存在 `theory/kane/`，重构前文件保存在 `archive/`。历史文档中的路径和参数应按当时版本理解。

后续会继续补充手写笔记，并在实车实践中逐步核对模型、控制器与仿真结果。
