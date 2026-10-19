# 俯仰角速度开环对象 Gqde

运行 `run_Gqde.m`，读取同项目 `A_B_Matrix/linear.mat` 中的本组四状态纵向模型。需 MATLAB 与 Control System Toolbox。当前目录内部无子文件夹。

状态 `[u,w,q,theta]`，升降舵输入列取 Blon(:,2)，输出矩阵 Cq=[0 0 1 0]，Dq=0。

`Gqde(s) = delta_q(s)/delta_e(s) = Cq*(sI-Alon)^(-1)*Blon(:,2)`。

输入是相对配平的升降舵增量 rad；输出是俯仰角速度增量 rad/s；Laplace 变量 s 的单位为 1/s。此处没有执行器模型、控制器或闭环。

文件：

- `Gqde.mat`：包含可直接用于 sisotool 的 Gqde（zpk 对象）、原始 Gqde_raw、状态空间 sys_q、系数和验证结果。
- `Gqde.txt`：ZPK 显示及多项式系数。
- `Gqde.json`：完整精度的零极点、系数与数值摘要。
- `Gqde_roots.csv`：零极点表。
- `Gqde_check.csv`：501 个频率点的传递函数/状态空间比较。
- `report.md`：中文计算结论。
- `Gqde_log.txt / matlab.log`：运行记录。

因为 q=d(theta)/dt 且当前四状态 A 可逆，该 plant 应有原点零点。程序独立检查状态方程和 DC 增益后，仅把浮点计算产生的近原点零点归零，保留原始版本。不做一般零极点消去，也没有为实现指标而修改飞机 plant。

提取后可在 MATLAB 中使用：

```matlab
load('Gqde.mat','Gqde');
Gqde
sisotool(Gqde)
```

打开 sisotool 是后续内环设计操作；提取脚本不会自动打开交互窗口，也不代表已完成三种补偿器设计。
