# 数学仿真运行结果

参数为临时示例；累计前向行程 s，质心距离 l。下表是实际运行值，不是 Gazebo/实车结果。

| 分支/场景 | 末端俯仰误差 rad | 末端位置误差 m | 电机峰值 N m | 限幅采样比例 |
|---|---:|---:|---:|---:|
| straight/open_loop | 0.1972 | 0.02867 | 0 | 0 |
| straight/balance | -4.445e-09 | 2.048e-08 | 0.2497 | 0 |
| straight/forward | 2.457e-09 | 1.537e-08 | 0.179 | 0 |
| straight/backward | -1.135e-08 | 2.559e-08 | 0.3204 | 0 |
| turning/left_turn/small_pitch | -4.663e-10 | 9.969e-09 | 0.266 | 0 |
| turning/left_turn/exact | -4.664e-10 | 9.97e-09 | 0.266 | 0 |
| turning/right_turn/small_pitch | -4.53e-10 | 9.681e-09 | 0.2857 | 0 |
| turning/right_turn/exact | -4.531e-10 | 9.683e-09 | 0.2857 | 0 |
| turning/reverse_turn/small_pitch | -4.683e-10 | 1.001e-08 | 0.2721 | 0 |
| turning/reverse_turn/exact | -4.685e-10 | 1.001e-08 | 0.2721 | 0 |
| turning/fixed_heading/small_pitch | -7.381e-11 | 9.516e-05 | 0.2494 | 0 |
| turning/fixed_heading/exact | -7.381e-11 | 9.516e-05 | 0.2494 | 0 |
| turning/turn_disturbance/small_pitch | 9.759e-10 | 2.007e-08 | 0.266 | 0 |
| turning/turn_disturbance/exact | 9.758e-10 | 2.007e-08 | 0.266 | 0 |

open_loop 的发散是预期现象，达到俯仰展示阈值后提前停止；其余为闭环。
静止分支为六状态局部线性模型；转向分支每种场景分别运行小俯仰模型和精确模型。
转向时评价的是相对运动参考的误差，世界位置和偏航角本身不趋于0；额外记录的累计行程 s 不参与控制。

每个案例的 outputs 子目录包含 trajectory.csv、states.png、torques.png 和 result.json；转向还有 path.png，父目录有 comparison.png 和 comparison.json。
A/B/Q/R/P/K 与当前物理参数保存在案例 result.json；七状态 K 作用于参考朝向下的旋转误差。
