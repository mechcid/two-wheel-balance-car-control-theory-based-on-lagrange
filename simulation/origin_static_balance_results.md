# 数学仿真运行结果

参数来自 CAD 与用户尺寸；累计前向行程 s，质心距离 l。下表是实际运行值，不是 Gazebo/实车结果。

| 分支/场景 | 末端俯仰误差 rad | 末端位置误差 m | 电机峰值 N m | 限幅采样比例 |
|---|---:|---:|---:|---:|
| origin_static_balance/open_loop | 0.1965 | 0.005846 | 0 | 0 |
| origin_static_balance/balance | 6.553e-12 | 3.814e-12 | 0.2379 | 0 |
| origin_static_balance/forward | -5.431e-12 | 3.266e-11 | 0.01433 | 0 |
| origin_static_balance/backward | 1.854e-11 | 4.029e-11 | 0.4616 | 0 |

open_loop 的发散是预期现象，达到俯仰展示阈值后提前停止；其余为闭环。
静止分支为六状态局部线性模型；转向分支每种场景分别运行小俯仰模型和精确模型。
转向时评价的是相对运动参考的误差，世界位置和偏航角本身不趋于0；额外记录的累计行程 s 不参与控制。

每个案例的 outputs 子目录包含 trajectory.csv、states.png、torques.png 和 result.json；转向还有 path.png，父目录有 comparison.png 和 comparison.json。
A/B/Q/R/P/K 与当前物理参数保存在案例 result.json；七状态 K 作用于参考朝向下的旋转误差。
