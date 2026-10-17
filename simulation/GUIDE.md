# 数值仿真阅读入口

当前说明已按两个理论场景重写，请阅读 [simulation/README.md](README.md)。它包含：文件职责、运行方式、反馈和积分过程、每个实验的设置、图表/CSV的含义，以及修改参数的方法。

新的统一入口是 `run_all.py`。旧版详细说明完整保存在 `../archive/simulation_before_scenarios_20260917/GUIDE.md`，主要对应原来的六状态单文件仿真。

符号已统一：累计前向行程 **s**；车轴到质心距离 **l**。世界位置 x、y 保持原义。
