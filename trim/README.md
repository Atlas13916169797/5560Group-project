# 配平检查

所有代码、依赖、过程和结果直接放在当前 trim 文件夹内，无子文件夹。再次运行也不会创建 results 等子目录。

在 MATLAB 中打开并运行 run_trim.m，只需要基础 MATLAB，可从任意工作目录启动。

代码：run_trim.m（入口）、init.m（初始化）、rates.m（状态导数）、check_trim.m（残差与60秒自由运行）、plot_trim.m（读取已保存数据重新绘图）。

依赖：FlightDataAircraft8.p、ICsAircraft8_200knts_300ft.mat、aero5560_*.p，全部位于当前目录。

过程说明：trim_report.md（方法与结论）、files.md（原仿真文件选用）、help.txt（接口）、Note.txt（兼容说明）。aircraft.md/csv/json 是读取的飞机参数。

结果：trim_residuals.csv（残差）、trim_summary.csv（摘要）、trim_history.csv（601个采样点）、trim.mat（完整数据）、trim.json（可读摘要）、trim.png/pdf（图表）。

日志：trim_log.txt、matlab.log、plot.log。它们保留实际运行时的原始路径，整理目录后未改写历史记录。

结论：提供的配平点满足数值平衡，60秒无扰动自由运行无漂移。该检查不替代扰动稳定性分析；诊断容差是本次工程选择，并非课程评分指标。
