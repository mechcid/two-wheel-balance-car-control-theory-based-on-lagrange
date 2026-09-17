# Gazebo 两轮车力矩闭环工程

首次系统学习请读 [ROS2_PROJECT_ANALYSIS.md](ROS2_PROJECT_ANALYSIS.md)：架构、启动、数据流、文件卡片、运行取证和实验。


## 2026-09-13 交互入口更新

六个具体问题及对应源码见 [ROS2_QUESTIONS_AND_INTERACTION.md](ROS2_QUESTIONS_AND_INTERACTION.md)。

```bash
/home/mechcid/lagarain/ros2_ws/run_interactive.sh
```

默认打开Gazebo和独立扰动面板，自动等待effort接口就绪后开始。面板支持暂停/继续、±1N前后推、±0.05N·m偏航脉冲和清除外力，每次0.1仿真秒。日志在results/interactive_时间戳；不修改原K。

后台运行可传 `gui:=false panel:=false`。批量run_case仍自行管理启动，已明确关闭GUI、面板和新的自动启动节点。以下较早文档中的“手动单步/播放”仅适用于显式设置 `auto_start:=false` 的调试模式。

## 当前验收状态（2026-09-11）

系统依赖已安装并确认，Gazebo已实际运行。最终验收结果见 [RESULTS.md](RESULTS.md)，原始CSV和曲线在results。

采用ROS 2 Jazzy、Gazebo Sim Harmonic 8.11.0、gz_ros2_control 1.2.20、controller_manager 4.48.0、effort_controllers/joint_state_broadcaster 4.42.1。控制器使用原数值仿真的K，未重新调参。

**最终模型使用球形轮碰撞作为理想窄轮点接触近似**，可视化仍为圆柱轮，轮的转轴、质量和惯量不变。有限宽圆柱碰撞的对照试验出现偏航受阻；失败数据保留在results/cylinder_contact。该局部理想接触仿真结果不能等同于真实宽轮轮胎的转向能力。

## 系统依赖（本机已安装）

新系统缺少这些包时，由你在自己的终端执行；本机不用重复安装：

```bash
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-effort-controllers ros-jazzy-joint-state-broadcaster
```

apt 会安装 ROS 系统包和依赖，不修改本项目虚拟环境。移除这三个包可用 `sudo apt remove`，不要盲目自动移除其他 ROS 依赖。版本方案对应 [Jazzy gz_ros2_control 官方文档](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)，未使用 Classic 插件。

## 文件职责

```text
ros2_ws/
  src/balance_sim/
    package.xml / setup.py / setup.cfg  ROS包和安装入口
    description/balance.urdf.xacro     刚体、碰撞、IMU、力矩接口
    config/physical.yaml              示例物理参数
    config/lqr.yaml                   原始K、频率、限幅、参考值
    config/controllers.yaml           effort controller和关节状态广播
    config/bridge.yaml                GZ与ROS消息桥接
    worlds/balance.sdf                地面、重力、接触动力学、外力插件
    launch/balance.launch.py          启动全部节点，物理世界初始暂停
    balance_sim/state.py              传感器到六状态的转换
    balance_sim/controller.py         200 Hz反馈、限幅、CSV记录、外力测试
    tools/check_model.py              不运行Gazebo的模型和状态转换检查
    tools/run_case.py                 四种物理试验的启动/等待/结束
    tools/plot_results.py             读取实测CSV画图并评估收敛
  results/                            物理试验日志、CSV、图和指标
  build/ install/ log/                colcon生成，可重新构建
```

## 构建与检查

使用系统 ROS Python，不激活项目 `.venv` 来构建 ROS 包。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mechcid/lagarain/ros2_ws
colcon build --symlink-install --packages-select balance_sim
source install/setup.bash
/usr/bin/python3 src/balance_sim/tools/check_model.py
```

`check_model.py` 检查惯量正值及三角不等式、轮轴、状态转换、K和参数与已有导出结果完全一致。它不证明闭环物理稳定。

## 运行物理试验

每项试验均重新创建世界，自动等待effort控制器激活并开始仿真；以CSV和曲线为收敛依据。

```bash
cd /home/mechcid/lagarain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 src/balance_sim/tools/run_case.py upright --output results
/usr/bin/python3 src/balance_sim/tools/run_case.py pitch --output results
/usr/bin/python3 src/balance_sim/tools/run_case.py push --output results
/usr/bin/python3 src/balance_sim/tools/run_case.py yaw --output results
```

自动试验隔离 ROS_DOMAIN_ID=73 和 GZ_PARTITION，仅关闭自己启动的进程。先等待 effort 和 joint-state 两个控制器激活，再解除暂停；必要时单步推进处理控制器切换，限制启动阶段无控制推进量。日志写入 results，不必使用 ~/.ros/log。

|试验|初始/施加条件|需要检查|
|---|---|---|
|upright|零角度|是否自发发散、接触抖动|
|pitch|初始俯仰 +0.04 rad|是否回正，轮力矩同向是否符合符号|
|push|3秒时请求车体 world +x 方向1 N，持续0.1秒|外力是否实际施加并消除、恢复过程|
|yaw|初始偏航 +0.04 rad，参考仍为0|左右差动力矩是否使偏航回零|

外力请求经 persistent wrench 桥接，再 clear 移除；不能仅凭“已发布请求”证明力已作用，需结合姿态/速度变化及Gazebo日志核实。

手动看GUI：

```bash
export ROS_LOG_DIR=/home/mechcid/lagarain/ros2_ws/results/ros_logs
ros2 launch balance_sim balance.launch.py gui:=true pitch:=0.04 log_path:=/home/mechcid/lagarain/ros2_ws/results/manual.csv
```

另一终端 source ROS 和工作区后调用 `ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'`（本机未安装ros2 control CLI扩展），确认 `wheel_effort` 和 `joint_state_broadcaster` 都为 active，再点击播放。若暂停导致控制器切换等待，可在GUI先单步。自动脚本中已有对应单步机制。

查看接口：`ros2 service call /controller_manager/list_hardware_interfaces controller_manager_msgs/srv/ListHardwareInterfaces '{}'`，应看到两个 wheel joint 的 effort 为 claimed，position/velocity 为 state。指令话题 `/wheel_effort/commands` 为 Float64MultiArray，顺序左、右，单位 N·m。不要同时运行另一个力矩发布器干扰闭环。

## 参数和模型对应

当前全部为原数值仿真的**程序示例值**，不是实车参数：

|参数|值|
|---|---|
|轮半径 r|0.10 m|
|轮距 d|0.40 m|
|质心相对车轴高度 l|0.30 m|
|车体/左轮/右轮质量|2.0 / 0.5 / 0.5 kg|
|左右轮 Iyy|各0.0025 kg·m²|
|左右轮 Ixx=Izz|各0.00125 kg·m²|
|车体 Iyy / Izz|0.08 / 0.06 kg·m²|
|新增车体 Ixx|0.10 kg·m²（原局部矩阵未用到，示例补充）|
|轮可视宽度/圆柱对照碰撞宽度|0.025 m|
|重力|9.81 m/s²|
|轮地摩擦系数|1.0|

惯量以原模型给定值为准，visual/collision几何不用于重新估算惯量。轮惯量是薄盘近似；最终球形碰撞半径等于轮半径，属于窄轮点接触近似，不按实心球重新计算惯量。车体盒子是简化碰撞外形，并非要求均匀实心盒。base_link在车轴中点，inertial origin在(0,0,l)。IMU直接配置在base_link，不添加虚假刚体质量。

轮轴均 +y，左轮在 +d/2·y，右轮在 -d/2·y。俯仰正方向为绕 +y，偏航绕 +z。当前导出K仅对应 psi0=0；不要单独更改 physical.yaml 里的 psi0 而沿用所有旧矩阵解释。

## 控制与状态来源

K直接取自 `../simulation/outputs/results.json` 的 `matrices.K_LQR`，完整精度写入config/lqr.yaml，未重新调整：

```text
[[-0.70710678, -0.5, -8.32450297, -1.37935257, -0.36142081, -2.04466231],
 [-0.70710678,  0.5, -8.32450297, -1.37935257,  0.36142081, -2.04466231]]
```

控制频率200 Hz，物理步长1 ms，ros2_control更新频率1000 Hz。力矩软件限幅±1 N·m，接口界限±5 N·m。`gain`按行展平为12个数；`reference_state`长度6，当前全零。参数在节点启动时读入，更改后重启。旧理想仿真峰值约0.25 N·m，故1 N·m限幅为初版留出余量；实际饱和比例需通过CSV检查。

`state.py` 使用IMU姿态得到俯仰/偏航；体轴角速度通过ZYX关系得到对应角速率。x为world位置在固定名义前向轴投影，速度取OdometryPublisher的子系速度转回world后投影。该插件提供真实模型位姿，但速度由位姿差分并平滑，不能当作无延迟的速度传感器。[对应Gazebo 8源码](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc)。轮关节状态单独记录，暂不参与位姿估计。

参考原点固定为生成位置x=y=0；名义朝向固定为0。不会把初始俯仰或偏航扰动重新定义为零。控制器等待三种消息齐全才输出，消息超过0.05仿真秒未更新则置零。俯仰超过0.6 rad则停止控制并报错。当前纯Python/ROS异步控制非硬实时，需检查采样间隔，不能只看设置频率。

## 如何读图和指标

有实际CSV后再运行（用已有venv的matplotlib，不安装新库）：

```bash
cd /home/mechcid/lagarain/ros2_ws
../.venv/bin/python src/balance_sim/tools/plot_results.py results/pitch.csv
```

生成 `pitch.states.png`、`pitch.torques.png`、`pitch.metrics.json`。六状态图横轴是仿真时间，左列位移/角度，右列对应速度；俯仰先回正、车体位置短暂偏移后回零通常符合捕获重心的过程。两轮同向力矩主要纠正前后俯仰，反向分量主要纠正偏航。扭矩图是**发送的命令**，不是独立测得的实际作用力矩；必须结合关节运动和物理响应验证effort链路。

CSV还记录轮相对车体角度/速度、是否饱和、传感器数据年龄。持续锯齿、高频大幅反向、长时间卡在±1 N·m都需要排查；最大力矩步变只是提示，不能单凭该值判断接触抖动。

初版收敛判据：运行至少14.9秒，末2秒各状态绝对值峰值小于 x=0.01m、psi=0.01rad、theta=0.005rad，三个速度均小于0.02对应单位。指标阈值写在绘图脚本中，可修改。小扰动成功也只覆盖这组参数和工况。

## 后续扩展

已有局部验收范围见RESULTS.md。前进/后退需要引入连续位置/速度参考和一致的前馈；偏航运动需要更新局部参考朝向、处理角度展开，不能仅把大角度yaw塞进静止点K。真实传感器替换放在state.py边界。当前阶段不添加这些功能。

## 复现接触对照和独立力矩检查

```bash
/usr/bin/python3 src/balance_sim/tools/run_case.py yaw --contact cylinder --output results/cylinder_repeat
/usr/bin/python3 src/balance_sim/tools/run_case.py left_probe --output results
/usr/bin/python3 src/balance_sim/tools/run_case.py right_probe --output results
```

left_probe/right_probe在零初始姿态下仅给相应轮+0.1 N·m，另轮为零，短暂关闭LQR以独立检查输入方向，总仿真约0.12秒；它们不是收敛测试。正常四项试验probe_torque全零，使用原LQR。不要在正常平衡试验中混用probe参数。

`*.interfaces.json` 保存实际服务返回的effort接口 available/claimed状态。正左轮力矩应造成负偏航加速度，正右轮力矩应造成正偏航加速度；两者均应使前进加速度为正、俯仰加速度为负。结合短测曲线判断物理作用，不能仅凭命令话题存在就认定成功。

Gazebo启动日志中KDL根link惯量警告只涉及robot_state_publisher的KDL表示；Gazebo仍读取base_link惯量。`IMU not found in hardware_info`表示IMU没有通过ros2_control的sensor接口输出，本工程改由Gazebo IMU桥接，实际消息已用于闭环。无需为消除这两条提示而改动质量或重复接入IMU。

## 新增：持续偏航运动

[独立 balance_yaw 分支](YAW_MOTION.md)支持累计前向行程l与连续偏航参考，原balance_sim静止点模型和入口保留。查看新运动分支可执行 `./run_yaw.sh`。
