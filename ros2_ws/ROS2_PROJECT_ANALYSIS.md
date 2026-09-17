# 两轮平衡车 ROS 2 仿真项目剖析

> 2026-09-13新增了自动启动就绪节点和扰动面板。本文主体保留此前学习取证；新入口、六问解答及修复见 [ROS2_QUESTIONS_AND_INTERACTION.md](ROS2_QUESTIONS_AND_INTERACTION.md)。默认launch现在自动开始，旧手动播放步骤仅适用于auto_start=false。

这份文档面向已经理解运动学、动力学和反馈控制，具备Linux及C/C++、Python基础，但尚未建立ROS 2工程整体认识的读者。目标是让你能沿真实文件追踪：**谁启动谁，谁读取谁，力矩怎样作用于刚体，状态怎样回来。**

本文对照当前源码和本次运行取证编写。物理验收结果见 [RESULTS.md](RESULTS.md)；运行取证原文在 [learning_evidence](learning_evidence/)。后文标明哪些是源码结论、哪些是实际消息。本文不会重新推导LQR，也不会把当前没有使用的ROS子系统塞进学习范围。

建议先读第1～6节建立整体图，再按第7节文件卡片查代码，最后读运行证据、实验和已知问题。不必背包模板。

### 阅读导航

- 先建立系统认识：[总图](#1-先看整个项目三套程序各有边界) → [启动时间线](#2-启动时到底发生了什么) → [机械与TF](#3-机械描述运动学和物理仿真如何对应) → [力矩链路](#4-控制量究竟怎样作用到关节)。
- 查具体文件：[文件卡片](#7-重要文件卡片谁读何时读删掉会怎样)。
- 查事实依据：[运行取证](#8-实际运行验证源码推断和现场结果对上了吗) → [小实验](#9-两个已经做过的小实验以及你可以继续做的实验)。
- 准备自己改工程：[问题清单](#10-当前实现中应明确知道的问题) → [人工设计与模板](#11-你应该自己设计什么哪些让工具生成即可) → [学习优先级](#12-学习优先级给ros知识划边界)。

## 1. 先看整个项目：三套程序，各有边界

工作区根目录是 `/home/mechcid/lagarain`。

|区域|工作内容|产物如何进入下一阶段|
|---|---|---|
|`theory/`|SymPy推导、笔记本、运动学及动力学公式|给出局部矩阵；不是Gazebo运行时调用的模块|
|`simulation/`|NumPy/SciPy局部线性模型与LQR数值验证|`outputs/results.json`保存参数与K|
|`ros2_ws/`|Gazebo刚体接触仿真及ROS闭环|用导出的K控制真实仿真关节，产生新的CSV|

**Gazebo没有每一步调用你的SymPy拉格朗日方程，也没有运行 `simulation/lqr_simulation.py`。** 它读取刚体、惯量、关节和接触配置，由自己的物理引擎积分。用户节点只拿状态算 `-KX`。因此Gazebo验证有意义：被控对象的计算途径变了，控制器仍是同一个K。

当前K已复制到 `config/lqr.yaml`，物理参数已复制到 `config/physical.yaml`。它们是导出快照，**不是自动双向同步**。改原JSON不会使运行中的Gazebo参数自动改变；`check_model.py`负责检查这些快照是否一致。

### 1.1 按功能层理解，而不是按文件后缀理解

|功能层|当前实现|在你已有知识中的位置|
|---|---|---|
|构建与安装|package.xml、setup.py、setup.cfg、resource/|软件包清单、安装规则、可执行入口注册|
|机械描述|description/balance.urdf.xacro、physical.yaml|刚体、关节、惯量与几何的声明式模型|
|物理仿真|worlds/balance.sdf、Gazebo内置插件|带关节与接触约束的动力学数值推进|
|运动学/TF|robot_state_publisher|读取关节角做连杆树正运动学|
|执行器适配|gz_ros2_control、ros2_control硬件接口|类似嵌入式HAL，提供读状态/写力矩的统一入口|
|命令转发|wheel_effort控制器|将ROS消息中的两路数字写入effort接口|
|用户算法|balance_sim/controller.py、state.py|状态组装、LQR、限幅；对应你真正设计的控制软件|
|通信|bridge.yaml、ROS topic/service|进程间消息通道和管理请求|
|启动编排|launch/balance.launch.py、tools/run_case.py|程序拉起、传参、等待就绪、解除暂停、退出|
|记录与显示|CSV、plot_results.py、summarize_tests.py、可选Gazebo GUI|实验记录、绘图和场景显示|

这里有两个容易混淆的“controller”：`balance_lqr`实现反馈算法；`wheel_effort`是ros2_control插件，当前仅转发力矩。**没有在它里面再做一层轮速PID。**

### 1.2 当前工程目录

```text
lagarain/
├── README.md
├── theory/
│   ├── sympy_scaffold.py / sympy_scaffold.ipynb
│   ├── build_notebook.py / SUMMARY.md
│   └── archive_kane/                 历史方案，不是当前仿真依赖
├── simulation/
│   ├── lqr_simulation.py             原线性模型和K的来源
│   ├── check_matrix_source.py / check_lagrange.py
│   ├── GUIDE.md / requirements.txt
│   └── outputs/                     原数值仿真的JSON、CSV、图
└── ros2_ws/
    ├── README.md / RESULTS.md / ROS2_PROJECT_ANALYSIS.md
    ├── src/balance_sim/              一个ROS包，而非整个ROS安装
    │   ├── package.xml
    │   ├── setup.py / setup.cfg
    │   ├── resource/balance_sim      空标记文件，供包索引发现
    │   ├── balance_sim/
    │   │   ├── __init__.py
    │   │   ├── controller.py         用户节点，入口名lqr_node
    │   │   └── state.py              状态转换纯函数
    │   ├── description/balance.urdf.xacro
    │   ├── config/
    │   │   ├── physical.yaml
    │   │   ├── lqr.yaml
    │   │   ├── controllers.yaml
    │   │   └── bridge.yaml
    │   ├── worlds/balance.sdf
    │   ├── launch/balance.launch.py
    │   └── tools/
    │       ├── check_model.py
    │       ├── run_case.py
    │       ├── plot_results.py
    │       └── summarize_tests.py
    ├── results/                     物理验收CSV、图、日志、模型快照
    │   └── cylinder_contact/        旧圆柱接触的对照结果
    ├── learning_evidence/           本文新增的只读取证及小实验
    └── build/ install/ log/          colcon生成物
```

`__pycache__/`只是Python缓存，不是额外模块。`build/`及`install/`中看到的同名文件可能是链接或安装产物，不应当作三份手工维护的源码。

本包**没有**CMakeLists.txt、include/、自定义C++源码、meshes/、rviz/、自定义msg/srv/action。也没有自行编写的Gazebo插件源代码：使用的是 `/opt/ros/jazzy` 中现成的C++共享库。没有这些目录是因为用不到，而不是工程不完整。

### 1.3 一张总图

```mermaid
flowchart TD
    P[physical.yaml] --> U[Xacro展开为URDF]
    U --> R[robot_state_publisher]
    U --> S[ros_gz_sim create]
    W[worlds/balance.sdf] --> G[Gazebo物理世界]
    S --> G
    G --> H[gz_ros2_control 仿真硬件]
    H --> J[joint_state_broadcaster]
    J --> JS[/joint_states]
    JS --> R
    R --> TF[/tf 运动学支路]
    G --> IM[IMU 与 OdometryPublisher]
    IM --> BR[ros_gz_bridge]
    BR --> ST[state.py 组装X]
    JS --> AL[balance_lqr]
    ST --> AL
    K[lqr.yaml 原始K] --> AL
    AL --> CMD[/wheel_effort/commands]
    CMD --> E[wheel_effort]
    E --> H
    H --> G
    AL --> CSV[CSV与调试状态]
```

注意：`/joint_states`被算法节点订阅，但当前只用来检查数据到齐、记录编码器；x与速度来自ground truth。移除该订阅前必须同步改就绪逻辑，否则节点会一直等待。

## 2. 启动时到底发生了什么

### 2.1 构建、source、launch分别做什么

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mechcid/lagarain/ros2_ws
colcon build --symlink-install --packages-select balance_sim
source install/setup.bash
export ROS_LOG_DIR=/home/mechcid/lagarain/ros2_ws/results/ros_logs
ros2 launch balance_sim balance.launch.py gui:=true pitch:=0.04
```

- 第一条让当前shell找到系统ROS程序和Python模块。
- colcon组织包构建；本包是ament_python，主要做Python包安装、资源索引和脚本入口生成，不会编译出我们自己的C++控制器。
- 第二次source把这个工作区叠加到包搜索路径。它不启动任何节点，也不安装系统组件。
- launch像一份Linux进程编排脚本：找到已安装的包资源，展开模型，拉起多个程序，传入参数。
- 上述手动launch让世界**暂停启动**，需要控制器激活后播放；自动测试入口还会处理等待和单步推进。

自动入口：

```bash
/usr/bin/python3 src/balance_sim/tools/run_case.py pitch --output results
```

它最终调用同一个launch，附加初始化角度、日志路径等参数。自动测试使用ROS_DOMAIN_ID=73和独立GZ_PARTITION。另开终端观察自动测试时，必须source同一环境并 `export ROS_DOMAIN_ID=73`；否则看到空ROS图不等于节点没启动。

### 2.2 launch动作不是严格串行就绪保证

`balance.launch.py`中的Python列表描述要启动的动作。把spawner写在create后面，**不表示机器人已生成完毕才执行spawner**。真正的依赖通过等待话题、Gazebo服务、controller_manager服务和控制器状态解决。

|启动对象|由哪个package提供|读什么|为什么存在|
|---|---|---|---|
|`gz sim -s`|系统Gazebo程序|worlds/balance.sdf|创建物理世界，初始暂停|
|可选 `gz sim -g`|Gazebo|服务器发布的场景|独立GUI；关闭显示不改变控制律|
|robot_state_publisher|robot_state_publisher|robot_description参数、use_sim_time|发布模型描述，后续做关节树TF|
|parameter_bridge|ros_gz_bridge|bridge.yaml|转换ROS/GZ不同消息类型和传输通道|
|create|ros_gz_sim|robot_description话题、姿态参数|向Gazebo请求生成balance实体，成功后退出|
|spawner|controller_manager|两个控制器名、等待超时|请求加载、配置、激活插件，完成后退出|
|lqr_node|balance_sim|lqr.yaml及launch覆盖参数|创建名为balance_lqr的ROS节点|

**controller_manager没有被launch作为单独程序显式启动。** 机器人被生成后，Gazebo加载URDF中的 `GazeboSimROS2ControlPlugin`，由该集成创建管理器及仿真硬件。`spawner`只发管理请求，不承担持续控制循环。

### 2.3 清楚的启动时间线

1. ROS包索引找到 `share/balance_sim/launch/balance.launch.py`。
2. `OpaqueFunction(setup)`在launch上下文里读取参数；Xacro读取physical.yaml，展开两轮宏和ros2_control声明，生成URDF字符串。
3. Gazebo读取world，加载Physics、UserCommands、Imu等系统插件。此时世界中只有地面等world实体。
4. robot_state_publisher获得URDF参数并发布 `/robot_description`；create等待它，向 `/world/balance_world/create` 这类Gazebo创建服务提交机器人。该步骤通过Gazebo传输完成，不要期待一定在 `ros2 service list` 中出现。
5. Gazebo将URDF转换为其内部SDF/实体表示，创建三刚体与两关节；同时加载模型上的控制集成插件和OdometryPublisher。
6. 控制集成读取ros2_control硬件声明和controllers.yaml，建立两个轮joint的state/command接口，提供controller_manager服务。
7. spawner加载并激活joint_state_broadcaster、wheel_effort。暂停状态下切换可能需要物理单步，run_case.py对此做每次1 ms推进。
8. 用户节点和桥接已在并行等待。run_case.py确认两个控制器active、effort available/claimed后，调用Gazebo world control解除暂停。
9. `/clock`开始推进；IMU、odometry、joint_states到齐且够新，balance_lqr开始200 Hz定时反馈。
10. 16仿真秒左右，测试监督程序请求launch关闭。节点清理CSV和力矩输出，Gazebo退出。图由后处理工具生成，不在动力学循环里绘制。

## 3. 机械描述、运动学和物理仿真如何对应

### 3.1 URDF声明的是机械装配

`link`声明刚体；`joint`声明父子连接及允许的相对运动。质量和惯量决定动力学，collision决定接触，visual决定显示。你可以把Xacro看成机械模型的预处理器：宏和变量展开后才得到URDF，它本身不做动力学积分。

本模型三个link：`base_link`、`left_wheel`、`right_wheel`；两个continuous joint：`left_wheel_joint`、`right_wheel_joint`。轴均为+ y，左轮中心偏移+ d/2，右轮为- d/2。continuous表示没有位置上下限，不表示没有转轴约束或没有力矩边界。

**base_link的位置在车轴中点，姿态随车体转动。它不是之前纸面上只随偏航转动的C系。** 它与B系姿态对应，但原点不在车体质心。`inertial/origin=(0,0,l)`才把质心放到正确位置。惯量在该惯性坐标系表达，本项目该坐标系与link轴平行。

因此ROS wheel joint角是“轮相对车体”的角；纸面上的theta_L、theta_R是轮相对C的角。在本模型同轴旋转约定下：

\[
q_{L,\mathrm{joint}}=\theta_L-\theta,\qquad
q_{R,\mathrm{joint}}=\theta_R-\theta.
\]

这解释了为什么不能把joint_states里的position直接抄成原推导中的绝对轮角。轴心平移不受绕y俯仰影响，所以两个模型的装配位置可以一致。

`inertial/origin`、`visual/origin`和`collision/origin`是link内不同用途的偏移，并不会自动各生成一个TF frame。这里没有单独的Bo、Co、imu_link；IMU直接作为base_link上的传感器配置存在，不额外增加刚体质量。

### 3.2 Gazebo怎样接管动力学

URDF进入Gazebo后，物理引擎使用质量、惯量、关节约束、重力与接触摩擦推进状态。ros2_control给joint施加的是相对转轴的力矩；父、子刚体上的反作用由关节动力学处理，用户节点无需再手动给车体补一个 `-(tau_L+tau_R)`。

两轮纯滚动不是在Gazebo里再输入你纸面上的三条非完整方程。这里依靠轮地接触和摩擦近似实现；存在滑动、接触瞬态及数值误差的可能。因此URDF与符号模型参数一致，并不保证所有工况的约束行为完全一样。

world的 `max_step_size=0.001` 是物理步长；`real_time_factor=0.5`是目标仿真时间/墙钟时间比例，不是控制增益。`physics type="ignored"`不是关闭物理，而是当前Gazebo Sim以Physics系统和物理后端处理该部分。没有显式指定某个后端名称时，不应仅凭此字段宣称使用ODE。

当前最终轮collision为同半径sphere，用于理想窄轮点接触近似；visual仍是圆柱，惯量仍是原薄盘惯量。圆柱宽轮对照中偏航受阻的实测结果说明，collision是动力学模型的一部分，不是画得好不好看的问题。详见RESULTS.md。

### 3.3 TF是正运动学结果，不是动力学执行通道

robot_state_publisher读取URDF关节树，收到关节角后计算父子刚体变换。例如：

\[
{}^{base}T_{left}(q_L)=\operatorname{Trans}(0,d/2,0)\operatorname{Rot}_y(q_L).
\]

当前应有的TF边：

```text
base_link
├── left_wheel     随 left_wheel_joint 的相对角变化
└── right_wheel    随 right_wheel_joint 的相对角变化
```

固定joint通常发布到 `/tf_static`，运动joint发布到 `/tf`。本模型没有固定joint，所以 `/tf_static`即使被advertise，也不意味着有静态边。

**当前没有ROS TF的 `world → base_link` 边。** ground truth odometry消息中有world/base_link标签，但Odometry消息不会自动变成TF。Gazebo的OdometryPublisher另有GZ侧变换话题，这个工程没有桥接它。用户算法直接读odometry而不查TF，所以闭环可以正常工作；以后要在RViz用world作Fixed Frame，需要补该变换发布，不能伪造一个固定变换代替运动姿态。

这也回答“是否必须先学完整TF系统才能控制”：本项目必须理解坐标含义与TF的职责，但反馈计算当前不依赖TF查询服务。

## 4. 控制量究竟怎样作用到关节

一次用户节点回调的顺序是：检查三种状态消息 → state.extract组装X → `raw=-K@(X-ref)` → clip → publish。

```text
balance_lqr 的Python代码
  ↓ Float64MultiArray.data=[tau_L,tau_R]，单位N·m
/wheel_effort/commands
  ↓ ROS订阅回调
wheel_effort（JointGroupEffortController，位于Gazebo内的管理器中）
  ↓ 按controllers.yaml的joints顺序写入
left_wheel_joint/effort、right_wheel_joint/effort
  ↓ gz_ros2_control/GazeboSimSystem 的write路径
Gazebo关节力矩 → Physics推进三刚体 → 新姿态、速度、关节状态
```

“interface”不是一个topic。可以类比HAL导出的两个可写寄存器/数据槽；controller_manager负责分配访问权、调度插件。`claimed=true`表示该接口被控制器占用，避免不同活动控制器同时写同一资源。话题是进程间传输，接口是控制框架内部的资源抽象。

state interface反向提供position、velocity、effort；joint_state_broadcaster读取它们，发布JointState。IMU在本工程**绕过ros2_control sensor接口**，通过Gazebo传感器系统和桥接发布，所以日志里“IMU未列入hardware_info”不代表反馈缺失。

若以后换实车，可以保留effort接口及用户节点的基本边界，将GazeboSimSystem替换为编码器读取/电机电流或力矩命令的硬件适配。不过真实电机的电流环、延迟、标定、安全停机仍需实际实现，不能把topic中的数字当作已经实现的电机力矩。

## 5. 状态从哪里回来

|状态/数据|原始来源|ROS通道|消费者和用途|
|---|---|---|---|
|IMU姿态、体轴角速度|Gazebo Imu系统，base_link传感器|`/balance/imu`|state.py计算theta、psi及欧拉角速率|
|车体模型位姿、局部速度|Gazebo OdometryPublisher|`/balance/odometry`|state.py投影到固定名义前向，求x、xdot|
|轮相对角/角速度|Gazebo关节→硬件state接口|`/joint_states`|robot_state_publisher做TF；用户节点检查和记录|
|仿真时间|Gazebo world时钟|`/clock`|use_sim_time节点的ROS clock及定时器|
|TF|robot_state_publisher|`/tf`|可视化/其他运动学消费者；当前算法不订阅|
|8元素调试数组|balance_lqr|`/balance/state_and_torque`|六状态后接左右力矩；可供监视|
|CSV|balance_lqr|文件，不是ROS话题|后处理绘图和指标验收|

`state.py`中IMU角速度先从体轴分量转换成ZYX欧拉角导数；odom twist从子系旋回world，再投影到固定heading。它是状态来源适配层，不是EKF。每次使用最新缓存的三条消息，检查年龄但没有严格时间同步。

当前位置参考是生成位置x=y=0，名义heading=0；初始yaw扰动不会自动被消掉。`reference_state`只在反馈误差中相减。默认没有积分器和跟踪前馈。

## 6. ROS通信的必要性：用本项目说明

### 6.1 node与process不一一对应

Python进程 `lqr_node`中创建的ROS节点名是 `/balance_lqr`。Gazebo进程内部则可以同时有controller_manager、wheel_effort、joint_state_broadcaster及控制集成节点。运行时节点数量不等于launch的Node动作数量。

### 6.2 topic：持续数据流

|话题|发布端|订阅端|类型/约定|
|---|---|---|---|
|`/robot_description`|robot_state_publisher|create及需要描述的组件|std_msgs/String，展开后的URDF|
|`/wheel_effort/commands`|balance_lqr|wheel_effort|Float64MultiArray，严格左、右|
|`/joint_states`|joint_state_broadcaster|balance_lqr、robot_state_publisher|JointState，以name映射数组索引|
|`/balance/imu`|parameter_bridge的ROS发布端|balance_lqr|sensor_msgs/Imu|
|`/balance/odometry`|parameter_bridge的ROS发布端|balance_lqr|nav_msgs/Odometry|
|`/clock`|parameter_bridge|使用仿真时钟的节点|rosgraph_msgs/Clock|
|`/world/balance_world/wrench/persistent`|balance_lqr测试代码|parameter_bridge|EntityWrench，创建持续外力|
|`/world/balance_world/wrench/clear`|balance_lqr测试代码|parameter_bridge|Entity，移除该实体持续外力|

为什么不是普通函数调用：算法节点与Gazebo分属不同进程、不同语言和不同调度循环。消息将二者解耦，还允许监视、记录及替换状态来源。到了controller_manager内部，插件与硬件适配之间确实主要是普通接口调用，不再每层套一个ROS topic。

ROS和GZ即使话题字符串相同，也不是同一条通信总线。bridge.yaml指定方向、两端类型，parameter_bridge才做转换。**effort命令不经过这座桥**，它经gz_ros2_control直接进Gazebo。

QoS也属于数据契约：本节点状态订阅用sensor_data的best-effort方式，允许丢旧采样；命令发布默认reliable、深度10。不要因此推断反馈是硬实时或无延迟。运行时具体端点QoS见取证command.json。

### 6.3 service：管理请求

spawner请求加载、配置、激活控制器；run_case查询 `/controller_manager/list_controllers` 和 `/controller_manager/list_hardware_interfaces`。这些是离散的“请求→返回”，适合service，不是每5 ms发送的控制数据。

ROS参数通常也通过get/set/list等服务管理。Gazebo world control/create属于另一套GZ服务；目前没有为它们都建立ROS服务桥。

### 6.4 action与参数

本项目没有自定义ROS action，也没有用导航或轨迹跟踪action。Python launch中的“Action”是启动框架的动作对象，与ROS action服务不是同一概念。

参数解决同一程序不同配置的问题。但当前controller.py在构造时把参数拷贝进self.K等成员，**运行中ros2 param set即使返回成功，也不会自动改变实际K/定时器/限幅**。修改配置后重启才是当前有效用法，后文单列此设计问题。

### 6.5 三个时间尺度

物理步长1 ms；controller_manager目标更新1000 Hz；用户反馈200 Hz，即每5 ms更新一次命令，中间保持最新值。use_sim_time让用户定时器跟随仿真时间，暂停世界也暂停该定时器。CSV的t从第一次有效状态开始计，和world从零开始的绝对clock有小的启动偏移。节点日志时间不能直接替代CSV横轴。

## 7. 重要文件卡片：谁读、何时读、删掉会怎样

下列源码路径均相对于 `ros2_ws/src/balance_sim/`。各卡片保留同样的八个问题；“删掉”指干净安装后缺失，旧install中残留副本可能暂时掩盖问题。

### 7.1 package.xml

源码：[package.xml](src/balance_sim/package.xml)

- **【文件是什么】** ROS包身份与依赖清单，属于构建安装层。
- **【谁会读取它】** colcon的ROS包识别、ament工具、rosdep等。
- **【什么时候读】** 发现包、构建和依赖检查时。
- **【关键内容】** name=balance_sim；build_type=ament_python；rclpy、ros_gz、gz_ros2_control等运行依赖。依赖声明不会自行执行apt。
- **【输入是什么】** 工程师选定的包名、构建类型与依赖关系。
- **【输出是什么】** 工具可识别的包元数据；不产生topic。
- **【删掉会怎样】** 失去规范的ROS包身份；有时colcon仍把setup.py识别为普通Python项目，但不会因此得到完整ROS包行为。
- **【掌握程度】** 必须理解用途和依赖；模板标签会修改即可。maintainer邮箱目前为example占位，license为生成时填写，正式发布前需确认。

### 7.2 setup.py

源码：[setup.py](src/balance_sim/setup.py)

- **【文件是什么】** setuptools安装规则。
- **【谁会读取它】** ament_python/colcon构建过程中调用的Python打包工具。
- **【什么时候读】** 构建和安装。
- **【关键内容】** packages安装Python模块；data_files安装config/description等资源；console_scripts把 `lqr_node` 指向 `balance_sim.controller:main`。
- **【输入是什么】** Python源文件、资源文件与安装目标。
- **【输出是什么】** 可导入模块、可执行入口、share下资源。
- **【删掉会怎样】** 当前包构建安装失败；系统不会知道怎样安装lqr_node。
- **【掌握程度】** 会修改即可；入口映射必须懂，模板不用背。

### 7.3 setup.cfg

源码：[setup.cfg](src/balance_sim/setup.cfg)

- **【文件是什么】** setuptools的脚本安装位置配置。
- **【谁会读取它】** setuptools。
- **【什么时候读】** develop/install阶段。
- **【关键内容】** 两段都把脚本放到 `$base/lib/balance_sim`。
- **【输入是什么】** 包安装前缀。
- **【输出是什么】** ROS约定位置的可执行脚本。
- **【删掉会怎样】** 可能仍能安装Python模块，但脚本落到普通bin位置，ros2 run/launch可能找不到。
- **【掌握程度】** 知道存在，换包名时会同步修改即可。

### 7.4 resource/balance_sim 与 balance_sim/__init__.py

- **【文件是什么】** 前者为ament资源索引的空标记；后者标识当前Python包。
- **【谁会读取它】** setup.py安装索引、ament_index查询包位置；Python导入机制读取包结构。
- **【什么时候读】** 安装及运行时查找/import。
- **【关键内容】** 前者的文件名必须与ROS包名一致；后者当前无业务代码。
- **【输入是什么】** 包名称和安装位置。
- **【输出是什么】** 让 `get_package_share_directory('balance_sim')` 和Python模块导入有入口。
- **【删掉会怎样】** 标记文件缺失使安装规则失败或包无法被ament发现；__init__.py缺失可能改变Python包发现/导入行为，不应指望命名空间包偶然可用。
- **【掌握程度】** 知道存在即可，不把空文件误判为可删垃圾。

### 7.5 description/balance.urdf.xacro

源码：[description/balance.urdf.xacro](src/balance_sim/description/balance.urdf.xacro)

- **【文件是什么】** 机械描述，同时携带Gazebo扩展及ros2_control接口声明。
- **【谁会读取它】** launch调用xacro预处理；展开的URDF再由robot_state_publisher和Gazebo读取。
- **【什么时候读】** 每次启动时展开；不是每个物理步重新解析。
- **【关键内容】** 三link、两continuous joint、+y转轴、质心偏移、惯量、轮接触分支、IMU、模型插件、effort/state接口。
- **【输入是什么】** physical.yaml、wheel_contact参数、controllers.yaml路径。
- **【输出是什么】** 完整URDF字符串，再转成仿真实体/运动学树。
- **【删掉会怎样】** launch展开模型失败；整个机器人无法生成。
- **【掌握程度】** 必须深入理解与力学模型有关的内容；宏语法会用即可。

### 7.6 config/physical.yaml

源码：[config/physical.yaml](src/balance_sim/config/physical.yaml)

- **【文件是什么】** 示例质量、惯量、r/d/l等参数快照。
- **【谁会读取它】** Xacro，以及launch读取r决定生成高度；check_model检查来源一致性。
- **【什么时候读】** 模型展开/启动和离线检查。
- **【关键内容】** r=.1、d=.4、l=.3、mB=2、轮质量=.5；车体惯量xx=.1/yy=.08/zz=.06，轮yy=.0025、xx=zz=.00125，均SI单位。
- **【输入是什么】** 来自旧results.json的模型参数和新增I_Bxx/轮宽示例值。
- **【输出是什么】** 机械描述中的数值。
- **【删掉会怎样】** Xacro读取失败。注意其中g、psi0当前不是world重力/节点heading的自动配置源。
- **【掌握程度】** 必须掌握参数物理含义和同步关系；无需学习YAML复杂特性。

### 7.7 worlds/balance.sdf

源码：[worlds/balance.sdf](src/balance_sim/worlds/balance.sdf)

- **【文件是什么】** 物理场景及world级插件声明。
- **【谁会读取它】** gz sim服务器。
- **【什么时候读】** 服务器启动。
- **【关键内容】** world名balance_world；重力；1 ms步长；地面collision/visual；Physics、UserCommands、SceneBroadcaster、Imu、ApplyLinkWrench插件。
- **【输入是什么】** XML配置与已安装的插件共享库。
- **【输出是什么】** 世界、地面、生成/控制服务和传感器/外力处理能力。
- **【删掉会怎样】** 当前launch打不开场景；可以换另一个world，但名称和所需插件必须同步。
- **【掌握程度】** 重力、接触和步长必须理解；灯光/场景显示参数会改即可。

### 7.8 config/controllers.yaml

源码：[config/controllers.yaml](src/balance_sim/config/controllers.yaml)

- **【文件是什么】** controller_manager管理的插件类型和关节顺序配置。
- **【谁会读取它】** Gazebo中的控制集成/manager，在加载controller时传给各控制器。
- **【什么时候读】** 管理器启动、controller加载配置。
- **【关键内容】** update_rate=1000；wheel_effort类型为JointGroupEffortController；joints固定左、右；joint_state_broadcaster输出全局joint_states。
- **【输入是什么】** 已声明的joint/interface名称、插件类型名。
- **【输出是什么】** 两个可加载controller及其运行参数。
- **【删掉会怎样】** Gazebo可能仍有刚体，但spawner无法按当前名称配置控制器，闭环不成立。
- **【掌握程度】** 必须理解类型、资源和顺序；其余字段会查即可。这里没有LQR增益。

### 7.9 config/lqr.yaml

源码：[config/lqr.yaml](src/balance_sim/config/lqr.yaml)

- **【文件是什么】** 用户反馈算法参数，不是controller_manager配置。
- **【谁会读取它】** launch将其作为ROS参数文件交给balance_lqr。
- **【什么时候读】** 节点启动，成员变量在构造时初始化。
- **【关键内容】** 12个gain按行reshape(2,6)；200Hz；1 N·m；ref六零；heading=0；use_sim_time；日志路径。launch对log_path等进行覆盖。
- **【输入是什么】** 原仿真K及实验设置。
- **【输出是什么】** 用户节点参数和实际反馈计算成员值。
- **【删掉会怎样】** 当前launch指向的参数文件缺失会报错；若人为删去该参数来源只用节点默认值，gain默认全零，会静默变成无反馈，这是风险。
- **【掌握程度】** 必须理解顺序、符号、量纲和限幅；不要背数字。

### 7.10 config/bridge.yaml

源码：[config/bridge.yaml](src/balance_sim/config/bridge.yaml)

- **【文件是什么】** 两套通信系统之间的消息桥接表。
- **【谁会读取它】** ros_gz_bridge/parameter_bridge。
- **【什么时候读】** 桥接节点启动。
- **【关键内容】** clock/imu/odom为GZ_TO_ROS；persistent wrench/clear为ROS_TO_GZ；两端消息类型必须匹配。
- **【输入是什么】** 一侧消息流和桥接配置。
- **【输出是什么】** 另一侧消息流。
- **【删掉会怎样】** 当前桥接启动配置失败或没有所需桥；缺IMU/odom/clock使用户控制不正常运行，缺wrench桥只使外力测试消失。
- **【掌握程度】** 必须理解桥接边界；具体类型映射查文档即可。

### 7.11 launch/balance.launch.py

源码：[launch/balance.launch.py](src/balance_sim/launch/balance.launch.py)

- **【文件是什么】** 本系统的主进程编排入口。
- **【谁会读取它】** ros2 launch。
- **【什么时候读】** 每次启动。
- **【关键内容】** OpaqueFunction读取launch参数；展开Xacro；ExecuteProcess启动gz；Node启动ROS程序；robot_description以字符串参数传入；初始世界不加-r所以暂停。
- **【输入是什么】** pitch/yaw/gui/wheel_contact/log_path/probe等启动参数及已安装包。
- **【输出是什么】** 多进程和节点组成的运行系统。
- **【删掉会怎样】** 无法使用当前一条命令启动；可以手动启动所有程序替代，但必须自己处理相同依赖。
- **【掌握程度】** 必须掌握谁启动谁和就绪条件；Python模板写法会修改即可。

### 7.12 balance_sim/state.py

源码：[balance_sim/state.py](src/balance_sim/balance_sim/state.py)

- **【文件是什么】** 状态来源适配层，主要是rotation和extract纯函数。
- **【谁会读取它】** controller.py导入；离线状态转换检查也调用它。
- **【什么时候读】** 模块加载后，每次控制回调执行extract。
- **【关键内容】** 四元数转矩阵；姿态到pitch/yaw；体轴角速度到欧拉角速率；odom子系速度旋回world；固定heading投影。
- **【输入是什么】** IMU消息、Odometry消息、名义heading。
- **【输出是什么】** `[x,psi,theta,xdot,psidot,thetadot]` NumPy数组。
- **【删掉会怎样】** controller导入失败；也可由其他传感器适配层替代，但必须保证状态契约一致。
- **【掌握程度】** 必须深入理解。这是你已有运动学知识最直接对应的代码。

### 7.13 balance_sim/controller.py

源码：[balance_sim/controller.py](src/balance_sim/balance_sim/controller.py)

- **【文件是什么】** 用户算法节点，夹带初版实验记录和施力逻辑。
- **【谁会读取它】** lqr_node入口调用main，rclpy执行消息回调和timer。
- **【什么时候读】** 启动时构造Controller，运行时receive缓存、step反馈。
- **【关键内容】** 参数复制；三路消息年龄检查；u=-K(X-ref)；限幅；theta超过0.6停止输出；debug和CSV；probe旁路；3秒外力测试；退出清理。
- **【输入是什么】** 三种消息、参数和ROS仿真时钟。
- **【输出是什么】** 力矩topic、调试topic、CSV、可选外力请求。
- **【删掉会怎样】** lqr_node入口无法导入；机械仿真仍可能运行但没有这套反馈。
- **【掌握程度】** 必须深入理解控制和状态检查；CSV与信号处理会用即可。duration只触发提示/flush，不负责停止仿真。

### 7.14 tools/run_case.py

源码：[tools/run_case.py](src/balance_sim/tools/run_case.py)

- **【文件是什么】** 自动实验监督程序，不是机器人控制算法。
- **【谁会读取它】** 你从终端用系统Python执行。
- **【什么时候读】** 每个试验开始，持续监视到结束。
- **【关键内容】** case映射初始条件；ROS_DOMAIN/GZ_PARTITION隔离；启动launch；查询active和claimed；逐毫秒启动；解除暂停；记录接口；等待clock；关闭自己创建的进程。
- **【输入是什么】** case、output、contact参数及已安装环境。
- **【输出是什么】** 完整物理试验、launch日志、接口JSON；控制CSV由controller写。
- **【删掉会怎样】** 自动验收入口消失，但手动launch仍可用。其打印completed不等于收敛，需再检查数据。
- **【掌握程度】** 会修改即可；服务就绪和退出时序必须理解。

### 7.15 tools/check_model.py

源码：[tools/check_model.py](src/balance_sim/tools/check_model.py)

- **【文件是什么】** 离线一致性检查。
- **【谁会读取它】** 工程师手动运行。
- **【什么时候读】** 开始试验前或改模型后。
- **【关键内容】** 惯量正值/三角不等式；两轮轴向；状态转换示例；K与参数是否等于旧JSON；psi0=0假设。
- **【输入是什么】** 展开URDF、config、项目外层simulation/outputs/results.json。
- **【输出是什么】** PASS或断言失败。
- **【删掉会怎样】** 不影响仿真运行，但少一道防止参数漂移的检查。
- **【掌握程度】** 会用即可。通过它不证明接触行为或闭环稳定。

### 7.16 tools/plot_results.py

源码：[tools/plot_results.py](src/balance_sim/tools/plot_results.py)

- **【文件是什么】** 单次物理CSV后处理。
- **【谁会读取它】** 使用已有.venv的Python执行，因其中有matplotlib。
- **【什么时候读】** CSV生成后。
- **【关键内容】** 六状态图、力矩图、末2秒阈值、峰值力矩、饱和比例、采样间隔。
- **【输入是什么】** 实际记录CSV。
- **【输出是什么】** states.png、torques.png、metrics.json。
- **【删掉会怎样】** 只少图和判据汇总，不改变Gazebo控制。
- **【掌握程度】** 会修改指标即可；应理解轴单位、窗口和阈值，不必掌握绘图库底层。

### 7.17 tools/summarize_tests.py

源码：[tools/summarize_tests.py](src/balance_sim/tools/summarize_tests.py)

- **【文件是什么】** 多工况验收汇总。
- **【谁会读取它】** 工程师在各CSV和metrics生成后运行。
- **【什么时候读】** 测试集完成后。
- **【关键内容】** 汇总四项工况；独立轮力矩短测拟合加速度并检验符号；生成总图和JSON。
- **【输入是什么】** 约定目录内四组结果和两组probe CSV。
- **【输出是什么】** acceptance.png、acceptance.json及断言结果。
- **【删掉会怎样】** 不影响闭环，仅失去统一验收入口。
- **【掌握程度】** 会使用和修改阈值/目录即可。

### 7.18 README、RESULTS、results与learning_evidence

- **【文件是什么】** 使用说明、验收报告、实验数据及本次学习取证。
- **【谁会读取它】** 人、绘图/汇总工具；Gazebo不靠Markdown工作。
- **【什么时候读】** 学习、复核、故障排查。
- **【关键内容】** 模型接触假设、命令、原始CSV、失败对照、CLI原文；results/balance.urdf/sdf是导出快照，不是launch模型源。
- **【输入是什么】** 源码分析和实际运行记录。
- **【输出是什么】** 可复习的证据和解释。
- **【删掉会怎样】** 源程序仍可构建运行，但历史证据不可恢复，必须重新做试验。learning_evidence不会被setup.py当成机器人运行依赖安装。
- **【掌握程度】** 核心结论必须读，原始日志按需查。

## 8. 实际运行验证：源码推断和现场结果对上了吗

本次在独立输出目录重新构建并运行pitch工况，未覆盖原results验收记录。构建日志见 [build.log](learning_evidence/build.log)，运行日志和CSV在 [physical_run](learning_evidence/physical_run/)。采集脚本 [capture_runtime.py](learning_evidence/capture_runtime.py) 为每条命令保存了目的、命令、返回码和原始输出。

### 8.1 本机版本和命令工具的区别

本项目实际组件为Jazzy、Gazebo Harmonic 8.11.0、gz_ros2_control 1.2.20、controller_manager 4.48.0，两个控制插件4.42.1。当前**没有ros2 control命令行扩展**：运行它返回invalid choice，但控制管理器及其服务正常存在。因此本次用以下实际接口，不额外安装CLI包：

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
ros2 service call /controller_manager/list_hardware_interfaces \
  controller_manager_msgs/srv/ListHardwareInterfaces '{}'
```

这两条的目的分别是确认控制器状态和硬件资源，不是为了展示命令长度。早期README中列出的ros2 control快捷命令需要相应CLI组件；上述服务形式是本机当前可用的替代。

### 8.2 实际节点与职责

[节点快照](learning_evidence/nodes.json)记录到：

```text
/balance_lqr
/controller_manager
/gz_ros_control
/joint_state_broadcaster
/robot_state_publisher
/ros_gz_bridge
/spawner_joint_state_broadcaster
/test_supervisor
/wheel_effort
```

其中test_supervisor来自run_case；spawner是取证时尚未退出的短生命周期节点，后续快照可能没有。create在模型生成成功后退出，所以不应要求稳态节点列表仍出现它。这是一次时间截面，不是“必须永远恰好九个节点”的测试。

### 8.3 力矩链路的现场证据

[command.json](learning_evidence/command.json)显示：`/wheel_effort/commands`类型Float64MultiArray，发布者唯一为balance_lqr，订阅者唯一为wheel_effort。发布端RELIABLE，订阅端BEST_EFFORT；不要把二者都写成reliable，现场输出比习惯猜测可靠。

[controllers.json](learning_evidence/controllers.json)显示：

- wheel_effort：active、JointGroupEffortController、update_rate=1000、占用左右轮effort；无需state接口，因为该插件只是转发，不计算LQR。
- joint_state_broadcaster：active、update_rate=1000；读取每轮position、velocity、effort。

[hardware.json](learning_evidence/hardware.json)显示两轮effort command接口均available=true、claimed=true；六项state接口available=true。state的claimed=false不表示不可读取，也不应套用command独占写入的解释。

### 8.4 状态与TF的现场证据

- [joint_states](learning_evidence/joints.json)：名字确为left_wheel_joint/right_wheel_joint，三个数组position/velocity/effort都存在。该快照在开始阶段，数值为零。
- [TF](learning_evidence/tf.json)：父均为base_link，子为left_wheel/right_wheel，平移分别 `(0,+0.2,0)` 和 `(0,-0.2,0)`，旋转四元数只有y轴旋转分量，与URDF一致。
- [IMU](learning_evidence/imu.json)：frame_id为 `balance/base_link/imu`，姿态q_y≈0.0199987、q_w≈0.9998000，对应0.04rad俯仰。传感器标签不等于TF中已存在该frame；本模型传感器轴和base_link对齐，state.py利用这一约定，未做TF查询。
- [Odometry](learning_evidence/odom.json)：header.frame_id=world，child_frame_id=base_link；pose高度约0.10m，与车轴位置一致。

**值得留意的启动样本：**取到的odom在0.005s附近有很大的原始twist（局部z约24.97m/s、角速度y约10rad/s），同时IMU角速度为零。它不能被解释为小车真的产生了该刚体运动；这是启动阶段ground-truth位姿差分输出的异常/瞬态证据。具体初始化原因没有在本次深入追踪插件源代码。当前state.py不使用odom角速度；该样本旋回world后前向分量接近零，闭环没有因此失败。但以后复用三维twist或改姿态时，应增加启动有效性检查，不要把ground truth等同于所有字段从第一帧起都理想。

各echo独立取样，时间戳不完全相同；不能用某次joint角和另一次TF旋转逐位比较。要严格数值对账应使用同一时刻的记录或时间同步，而不是混合截图。

### 8.5 其余取证命令为什么运行

|命令/取证文件|验证目的|得到的事实|
|---|---|---|
|`ros2 topic list -t --no-daemon`，[topics.json](learning_evidence/topics.json)|确认真实消息通道|存在命令、状态、描述、clock、TF等话题|
|`ros2 service list --no-daemon`，[services.json](learning_evidence/services.json)|确认管理接口|controller_manager和各节点参数服务实际可查询|
|`ros2 action list`，[actions.json](learning_evidence/actions.json)|核实是否需要学本项目action协议|返回空；当前没有该类业务接口|
|`ros2 param list /balance_lqr`，[parameters.json](learning_evidence/parameters.json)|确认节点声明了哪些参数|包括gain、reference_state、control_frequency等|
|`ros2 param get /balance_lqr control_frequency`，[frequency.json](learning_evidence/frequency.json)|核实配置传递结果|实际值200.0|
|`ros2 interface show std_msgs/msg/Float64MultiArray`，[message_type.json](learning_evidence/message_type.json)|确认命令数据契约|它是layout加float64数组，不自带左右轮名和单位|

所有这14项取证命令本次返回码为0；完整执行情况见 [capture.log](learning_evidence/capture.log)。其中接口原文也提示MultiArray是泛用数组类型；当前现成effort controller使用它，工程师仍需通过配置保证顺序，不能靠消息类型检查出左右交换。

## 9. 两个已经做过的小实验，以及你可以继续做的实验

### 9.1 只改变collision，机械接口会改变吗？——已执行

运行 [small_experiments.py](learning_evidence/small_experiments.py)，分别展开sphere/cylinder分支，输出见 [contact_expansion.json](learning_evidence/contact_expansion.json)：两种模型都为3 link、2 joint，左轮惯量完全相同，仅碰撞几何不同。这是静态模型对照；先前物理验收又表明偏航响应不同。二者合起来说明：

> visual、collision、inertial是三个独立模型描述维度。相同关节树和K并不足以规定相同的接触动力学。

未修改源配置，临时分支通过Xacro参数展开。可结合results/cylinder_contact中的实际失败曲线理解，没必要再次跑一遍才能接受这个结论。

### 9.2 参数服务器更新，计算变量会自动更新吗？——已执行

在隔离ROS_DOMAIN_ID=77的临时节点上执行 [parameter_experiment.py](learning_evidence/parameter_experiment.py)，没有Gazebo、没有轮执行器，源YAML不变。结果 [parameter_update.json](learning_evidence/parameter_update.json)：

```text
修改前：参数 torque_limit = 1.0，节点成员 self.limit = 1.0
set_parameters返回 successful=True
修改后：参数 torque_limit = 0.2，节点成员 self.limit = 1.0
```

ROS参数服务存储与应用算法状态不是同一件事。当前实现只在__init__读取参数，必须重启才生效。以后要热更新，需加参数校验和更新回调；control_frequency还需要重建或调整timer，不能只改一个float。

### 9.3 建议按这个顺序自行实验（以下未在本次逐项运行）

|实验|只改哪里|观察什么|学到的边界|
|---|---|---|---|
|切换gui:=false/true|launch参数|有无窗口，CSV闭环仍工作|可视化进程不是动力学控制器|
|改wheel visual颜色|Xacro visual|显示改变，质量/碰撞描述不变|显示几何与物理配置独立|
|切换wheel_contact|launch或run_case --contact|偏航回正与受阻对照|接触是被控对象的一部分|
|把轮距d临时改小并展开Xacro|副本physical.yaml|两轮joint origin变小|装配几何来自参数而非TF手写常量|
|调低torque_limit后重启|副本lqr.yaml|命令截顶、饱和比例、恢复能力|执行器约束影响闭环，不只是图形幅值|
|暂停Gazebo|GUI或world control|clock、timer、CSV停止推进|仿真时间与墙钟不同|

涉及改配置建议用副本，并记下恢复位置；不要同时改K、惯量、摩擦，再试图从一条曲线归因。副本参数改变后check_model可能故意报来源不一致，这不是Xacro坏了，而是保护条件提醒你已离开原验收模型。

## 10. 当前实现中应明确知道的问题

这一节是代码审查记录，**本次以解释为主，没有把这些问题借机重构掉**。

### 10.1 会影响后续使用的设计缺口

1. **热更新参数假象（已实测）**：ROS服务接受更新，实际成员不变。现阶段重启节点；将来实现参数回调或把参数声明为只读。
2. **配置不完全单源**：physical.yaml的g未驱动world重力，psi0未驱动nominal_heading；摩擦、body盒尺寸、接口限值也有硬编码。改g/朝向时必须沿消费链检查。当前数值一致不等于已经自动联动。
3. **K默认全零且参数校验不完整**：gain形状错误reshape会报错，但全零、reference非有限值、probe长度不对等未完整防护。不要绕过lqr.yaml后以为仍运行原控制器。
4. **消息校验只检查是否到齐/年龄**：未检查frame_id、时间同步、四元数归一化和关节数组完整性；关节名缺失会使index查找报错。首帧odom异常也说明需要明确初始化有效性。
5. **IMU标签无对应TF边**：当前依赖人为保证与base_link对齐。以后安装有转角的真实IMU，需要外参及相应转换，而非仅改frame_id字符串。
6. **缺执行器侧命令超时保护**：用户节点发现旧传感器消息时会发零，但节点若崩溃就不能再发零；转发控制器可能保持最后命令。真实硬件应有独立watchdog。正常退出发送一次零命令也不等于对端已确认执行。
7. **持续外力清理依赖控制step**：如果施力后状态失效导致提前return，3.1s的clear可能不执行。测试正常退出会销毁world，但手动长时间运行应把外力发生器和清理做成独立机制。
8. **duration名称容易误读**：当前只是超过该时长提示/flush，仍继续控制和记录；真正结束由run_case以world时间判定。不是节点自动运行15秒后退出。
9. **probe模式隐藏在非零数组判断中**：任一probe非零就绕过LQR，未用明确mode参数表达。初版能工作，维护时应清楚区分测试激励与正常控制。
10. **手动launch缺接触类型校验**：run_case只允许sphere/cylinder，但手动launch可传其他字符串，导致两个Xacro分支都不生成轮collision。应校验枚举，不把拼写错误变成物理模型改变。
11. **缺world→base_link TF**：不影响当前直接读odom的LQR，但会影响以world为参考的RViz/其他TF消费者。不能用不随运动变化的静态边修补。

### 10.2 工程整理与可复用性问题

- package.xml依赖主要够当前环境运行，但对直接导入的launch、launch_ros、ament_index_python等没有逐项显式声明；新环境应补全，而不是依赖传递安装碰巧可用。
- 没有自定义CMake文件是正确的；当前Python包不需要为了“像ROS工程”硬加CMakeLists.txt。
- controllers.yaml、physical.yaml与lqr.yaml分开有职责依据；没必要为缩短目录把它们合成一份无边界的大配置。
- `Controller`把反馈、CSV、probe、外力测试都写在一起，是初版为了跑通的折中；生产代码宜把实验驱动/记录与控制职责分离。
- 代码里较多一行多语句、短变量名和lambda，是生成代码的可读性债务，不是ROS要求；理解时按职责块读，不需要模仿压缩写法。
- check_model.py和summarize_tests.py通过父目录定位外层文件，对这个仓库有效，但不是搬到任意已安装位置都能运行的独立工具。后者输出目录也没有可配置参数。
- run_case使用固定ROS_DOMAIN_ID=73；GZ_PARTITION虽独立，若同时运行两次，ROS节点和topic仍可能冲突。所以当前按顺序运行，不并行多车测试。
- summarize_tests先写JSON/图再断言；run_case打印completed也只表示跑完。自动CI应读取判据/退出码，不能只查文件存在。
- model_provenance.json是部分模型文件哈希快照，不覆盖所有算法和运行环境，不应称为完整可复现实验锁文件。
- maintainers/license等元数据是生成模板的占位，正式开源前需人工确认。

### 10.3 哪些现象不是当前bug

根link惯量的KDL警告影响robot_state_publisher的KDL表示，不代表Gazebo忽略车体惯量；IMU未进入ros2_control hardware_info也不代表没有ROS IMU数据，因为本工程走bridge。对日志要沿实际数据链判断，不能见WARN就随意加link或额外传感器接口。

## 11. 你应该自己设计什么，哪些让工具生成即可

这套新增ROS包、配置和工具是在本次项目协作中由agent生成并调试的；它们不是ROS官方提供的某个完整示例原封不动复制。系统插件和标准ROS库由上游提供。K来自此前已有的数值模型，不是在Gazebo调试中另造的控制器。

|部分|生成器/模板能做什么|工程师必须决定什么|
|---|---|---|
|package.xml、setup.cfg、resource、__init__|生成骨架和一致的名字|实际依赖、包边界、发布元数据|
|setup.py|生成安装和入口规则|哪些文件安装、入口对应哪个main|
|launch基本语法|生成Node/ExecuteProcess写法|启动对象、参数覆盖、暂停/就绪/退出时序|
|Xacro左右轮宏|消除重复XML|轴、左右符号、质心、惯量、接触假设|
|bridge.yaml格式|填标准消息映射模板|哪些信号跨桥、方向、frame/时间语义|
|controller配置|填插件类型和接口字段|effort而非velocity、joint顺序、更新周期|
|state.py|辅助写矩阵代码|各量表达在哪个系、参考点、角速率含义|
|controller.py|生成订阅、timer、publish样板|状态顺序、K来源、限幅、异常处理和安全策略|
|绘图/CSV|生成常见I/O和图形|验收阈值、实验设计、失败归因|

最不能外包理解的是：**状态契约、执行器契约、机械/接触假设和时序。** 一份能编译的模板无法替你选择正确的坐标系和输入方向。

## 12. 学习优先级：给ROS知识划边界

### A. 必须深入掌握

- link/joint/inertial/collision与原机械模型的对应；base_link与C/B的区别。
- node、topic、消息数据契约、参数，尤其数组顺序、单位、坐标和时间戳。
- launch启动依赖及就绪条件；程序开始运行不等于能接收有效状态。
- ros2_control的controller/command interface/state interface/hardware边界；effort如何进入joint。
- TF树与robot_state_publisher做什么、不做什么；Odometry与TF的区别。
- ROS/GZ桥接边界、use_sim_time、实际采样周期。
- 状态提取与反馈主循环。这部分直接决定物理符号和闭环是否正确。

### B. 应熟练使用，不必深入内部实现

colcon构建、工作区overlay/source、ament包发现；查topic/service/parameter；Xacro宏/YAML；plugin加载配置；CSV与绘图；controller加载/激活服务；启动日志与资源接口排查。DDS只需先掌握影响连接的域、QoS和发现，不必立即研究协议实现。

### C. 当前知道存在即可

Gazebo物理后端内部求解算法；pluginlib细节；rclcpp/rclpy executor内部调度；ROS 2通用生命周期节点全套机制；复杂TF缓存插值；消息IDL生成器。controller的active状态需要懂操作含义，不要求先读完整生命周期框架源码。

### D. 本项目没涉及，先不学

自定义msg/srv/action构建、C++ ament_cmake工程、MoveIt、Nav2、SLAM、EKF、相机/激光、MPC、全局轨迹跟踪、多机器人命名空间、真实硬件驱动实现。rosbag2和RViz是以后可加入的工具，当前记录用CSV、显示用Gazebo/Matplotlib；不必为了完成本项目把它们都装起来。

## 13. 如果自己从零搭建，建议怎样分模块

1. 先写机械参数和URDF，明确基座frame与关节角的含义，检查惯量和轴向。
2. 在最简单world中生成机器人，确认自由下落、关节和接触确实工作。
3. 配好hardware接口与最简单effort转发，先验证单轮正力矩的实际运动方向。
4. 独立验证IMU/odom/state.py，输出六状态，和几何直觉核对。
5. 接原K与限幅，处理启动时序和仿真时钟。
6. 加可复现实验和CSV验收，再按需要加GUI/TF消费者。

这一顺序按契约逐层验收，出问题容易定位；不是先把所有模板一次生成完，再只看“车能不能站住”。

## 14. 完整运行链路：最后用一张图串起来

```text
构建阶段：package.xml + setup.py/setup.cfg + resource
                   ↓ colcon安装，source建立搜索路径
启动阶段：balance.launch.py
    ├─ physical.yaml → Xacro → robot_description
    │                           ├─ robot_state_publisher
    │                           └─ create → Gazebo里的balance模型
    ├─ balance.sdf → Gazebo world/Physics/Imu/外力插件
    ├─ bridge.yaml → ros_gz_bridge
    ├─ lqr.yaml → lqr_node进程 → /balance_lqr节点
    └─ spawner → Gazebo插件创建的 /controller_manager
                       ├─ /joint_state_broadcaster
                       └─ /wheel_effort

运行阶段：
Gazebo Imu/OdometryPublisher
    ↓ GZ话题 → bridge → /balance/imu、/balance/odometry
state.py（参考系/角速率/固定前向投影）
    ↓ X=[x,psi,theta,xdot,psidot,thetadot]
/balance_lqr → -K(X-ref) → ±1 N·m限幅
    ↓ /wheel_effort/commands，data=[tau_L,tau_R]
/wheel_effort → 左右joint/effort command interfaces
    ↓ gz_ros2_control写入
Gazebo Physics → 关节力矩/接触/重力 → 下一时刻刚体运动
    ├─ IMU/odometry → 回到反馈输入
    └─ joint states → 硬件state interfaces → joint_state_broadcaster
                          ↓ /joint_states
                          ├─ balance_lqr检查/记录
                          └─ robot_state_publisher正运动学 → /tf

旁路：/clock决定仿真时间；CSV/调试topic记录输出；GUI和绘图负责显示。
```

读完后，你应该能分别指出机械模型、数值推进器、算法、执行器适配和通信所在的位置。文件存在的理由就不再是“教程里都有”，而是它承担了这条链上的一个明确职责。
