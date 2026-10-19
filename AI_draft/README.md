# 当前阶段 LaTeX 报告

打开 report.pdf 查看已编译的 17 页中文报告。完整正文和框图均在单一源文件 report.tex 中；配平图 trim.png 需与它放在同一目录。

英文备份：report_en.tex 与 report_en.pdf（16 页）。英文反馈框图在第 9 页；数值、编号公式和附录矩阵与中文版一致。两种语言共用 trim.png，编译方法相同，将命令中的 report.tex 替换为 report_en.tex 即可。

报告包含：Group 8 飞机数据与气动参数、配平方法和实际结果、完整 A/B 与各子矩阵、数值验证、Gqde 提取和零极点、俯仰角速度反馈框图、信号定义、闭环推导和 SCK 关系。

本次新增的反馈回路在第 6 节，完整框图在 PDF 第 11 页。假设理想测量和理想执行器，补偿器 Kq 的参数仍待后续设计，没有给出未经验证的闭环达标结论。

使用 XeLaTeX 编译两次以更新目录和引用：

```text
xelatex -interaction=nonstopmode -halt-on-error report.tex
xelatex -interaction=nonstopmode -halt-on-error report.tex
```

本机使用 Windows 中文字体。迁移到其他平台时，可将文档类选项 fontset=windows 改为可用的 ctex 字体方案。

loop.m 是与框图对应的 MATLAB 函数，需要传入已选择、带正确符号的补偿器 Kq；它返回 Lq、Sq、Cq 和 Uq，不自动选择补偿器。该函数依赖相邻 Gqde 目录的 Gqde.mat 及 Control System Toolbox。

矩阵、零极点等数值以已有 MATLAB 计算结果为依据；正文打印数值有舍入，继续计算时应使用 MAT 文件。
