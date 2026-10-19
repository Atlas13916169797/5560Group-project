# 仿真文件检查与选用

来源：工作区 `flight simulation student version`。以下说明区分本次运行依赖和后续参考。表中 sim 与 sim/help.txt 是初次整理时的归档位置；现已统一移动到本 trim 文件夹，核心函数与 help.txt 均直接位于当前目录。reference 位于项目根目录，示例不参与配平检查运行。

| 原文件 | 选用情况 | 原因 |
|---|---|---|
| aero5560_aero.p | 已复制到 sim，实际调用 | 非线性气动与推进力/力矩模型；包含状态导数依赖 |
| aero5560_motion.p | 已复制到 sim，实际调用 | 根据力/力矩求 12 状态导数 |
| aero5560_atmos.p | 已复制到 sim | 非线性框架的大气模型依赖 |
| aero5560_euler.p | 已复制到 sim，本次不调用 | 课程 Euler 积分函数，后续可用；本次用 ode45 并作更严格精度复核 |
| Simulation_Function _Help.txt | 已复制为 sim/help.txt | 状态、输入顺序和单位的依据 |
| elevator_step_example.m | reference/elevator_example.m | 只参考调用流程。原文件加载默认飞机，并且已被改为短时修改 w，不是干净的升降舵阶跃实验 |
| rudder_impulse_example.m | reference/rudder_example.m | 只参考调用/动画流程；原文件同样加载默认飞机，不直接用于本组配平检查 |
| aero5560_LoadFlightData.p | 不纳入本组运行依赖 | 返回默认示例飞机，本组必须调用 FlightDataAircraft8.p |
| ICs_FC1.mat | 不纳入本组运行依赖 | 默认示例初始条件，不能代替本组 200 knots / 300 ft 数据 |
| week8_aircraft_response.m | 已检查、复用状态导数迭代思路 | 主脚本含其他作业与默认模型设置，不整段直接运行；独立实现为 rates.m |
| week7task*.m、week8task*.m、Week8_template.m | 后续阵风/建模参考，本次未复制 | 有默认飞机、特定输出和作业格式，需针对 Group 8 改写 |
| aero5560_gustTransfer.m | 后续传递函数参考，本次未复制 | 当前不进行阵风或传递函数分析；使用前仍需核对消去容差与输出维度 |
| week9_root_locus.m、root_locus_y1.m | 后续控制设计参考，本次未复制 | 配平检查不需要根轨迹和 Control System Toolbox |
| uav.m、visualisation.avi | 本次未复制 | 动画不是配平检查依赖，也不应复制生成的视频作为代码依赖 |
| AERO5560_check_data.mat、AERO5560_Task2a_coefficients.mat | 本次未复制 | 旧作业数据，不作为 Group 8 的模型依据 |

所有采用的课程 P-code 均保留原函数名，避免破坏 MATLAB 调用关系。原文件未修改。reference 内示例不作为运行入口，也不要求把默认飞机文件补回项目路径。
