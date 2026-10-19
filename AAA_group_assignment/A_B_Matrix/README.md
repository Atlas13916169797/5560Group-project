# Group 8 线性化

运行 `run_linear.m`，在已验证的 200 knots、300 ft 配平点计算 A/B。依赖同项目 `trim` 文件夹内的本组数据、init.m、rates.m 及课程非线性模型，所有本阶段文件直接保存在当前目录。只需基础 MATLAB。

模型为 `deltaXdot = A*deltaX + B*deltaU`。完整状态顺序：

`[u, v, w, p, q, r, phi, theta, psi, Xe, Ye, Ze]`

完整输入顺序：

`[throttle, elevator, aileron, rudder, flap]`

全部采用 SI 单位，角度和舵角用 rad，角速度用 rad/s，油门为 0–1 的无量纲比例。B 对应操纵量增量，不是把配平操纵量设为零。

水平位置的标称值随时间前进，deltaX 是相对于标称飞行轨迹的偏差，不能把标称 Xe 变化率当作残差。当前模型对水平位置平移不敏感，因此可在初始点计算恒定雅可比矩阵。

## 输出文件

- `A.csv`：完整 12×12 矩阵。
- `B.csv`：完整 12×5 矩阵，包含襟翼列。
- `A_lon.csv / B_lon.csv`：纵向 4 状态 `[u,w,q,theta]`、2 输入 `[throttle,elevator]`，固定高度近似。
- `A_lon5.csv / B_lon5.csv`：含 Ze 的纵向 5 状态模型，保留高度与大气密度变化的影响。
- `A_lat.csv / B_lat.csv`：横航向 5 状态 `[v,p,r,phi,psi]`、2 输入 `[aileron,rudder]`，保留航向积分状态。
- `matrices.txt`：便于阅读和复制的数值矩阵。
- `linear.mat`：全部矩阵、配平数据、差分结果、特征值、验证数据；另含 Bcontrols（去掉襟翼的 12×4 矩阵）。
- `linear.json`：可读数值摘要。
- `convergence.csv`：五组差分步长的矩阵变化。
- `taylor.csv`：非线性状态导数与线性近似的局部误差及缩小扰动后的误差比。
- `validation.csv`：小阶跃线性/非线性对比误差与响应峰值。
- `elevator_check.csv / throttle_check.csv`：10 s 小阶跃输出历程，速度 m/s、q 为 deg/s、垂直速度 m/s（向上为正）。
- `poles.csv`：纵向、横航向和完整模型特征值。
- `linear_log.txt / matlab.log`：运行记录。
- `report.md`：计算完成后的中文结论。

CSV 第一列为状态导数行名，例如 d_q；后续列是对应状态或输入。A/B 元素的单位由行输出单位除以列输入单位决定，不是所有元素都无量纲。

## 方法

使用中心差分，并在每个扰动点重新迭代求解隐式气动状态导数。状态变量按配平速度、单位角度/角速度和位置量级设置尺度，输入尺度为 1。

相对步长依次为 1e-4、3e-5、1e-5、3e-6、1e-6，选取中间的 1e-5 结果。检查所有结果相对所选矩阵的 Frobenius 范数差，并记录最大元素差。该全矩阵指标不代表每个接近零的元素都具有相同相对精度。

另用独立混合扰动检查 Taylor 余项随扰动减半约缩小四倍。最后用 0.05° 升降舵、0.001 油门增量分别验证 10 s 输出响应；线性解由矩阵指数计算，非线性由 ode45 求解，无需 Control System Toolbox。

小阶跃验证使用完整 12 状态 A/B。截取的纵向 4 状态模型忽略高度反馈，横航向模型基于对称配平的子系统分离，后续设计应明确使用的模型版本。
