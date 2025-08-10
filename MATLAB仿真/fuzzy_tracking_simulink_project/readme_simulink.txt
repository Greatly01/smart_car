
【Simulink 工程说明 - 基于模糊控制的移动机器人路径跟踪】

此项目包含：
1. robot_fuzzy_sim.slx  - Simulink 模型文件
2. fuzzy_controller.fis - 模糊控制器规则文件
3. main.m               - MATLAB 脚本（用于仿真或对比）

使用说明：
1. 打开 fuzzy_controller.fis 文件，在 MATLAB 的 fuzzyLogicDesigner 中完善规则与隶属函数；
2. 打开 robot_fuzzy_sim.slx，检查 FIS 模块是否正确加载 fuzzy_controller.fis；
3. 运行仿真，观察路径跟踪效果（Scope 中显示实际轨迹与目标轨迹）；
4. 若需修改路径规划，可在 Simulink 中 Target Generator 模块处更改；
