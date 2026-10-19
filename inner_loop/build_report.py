"""Build bilingual LaTeX sources from verified MATLAB outputs (standard library)."""
import csv
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA = json.loads((HERE / "results.json").read_text(encoding="utf-8"))
METRICS = DATA["Metrics"]
FINAL = METRICS[2]
CONTROLLERS = json.loads((HERE/'controller.json').read_text(encoding='utf-8'))['Controllers']
LOOP20_DB = [m['L20_dB'] for m in METRICS]


def number(value, places=3):
    return r"$\infty$" if value is None else f"{value:.{places}f}"


def table(headers, rows, caption):
    body = " & ".join(headers) + r"\\\midrule" + "\n"
    body += "\n".join(" & ".join(str(x) for x in row) + r"\\" for row in rows)
    return (r"\begin{table}[H]\centering\small\caption{" + caption + "}\n"
            + r"\begin{tabular}{" + "l" + "r" * (len(headers)-1) + "}\\toprule\n"
            + body + "\n" + r"\bottomrule\end{tabular}\end{table}" + "\n")


def figure(name, caption, width="1.0"):
    return (r"\begin{figure}[H]\centering\includegraphics[width=" + width
            + r"\linewidth]{" + name + r".pdf}\caption{" + caption
            + r"}\end{figure}" + "\n")



def as_list(value):
    return value if isinstance(value,list) else [value]


def zpk_formula(model, remove_integrator=False):
    def factors(values):
        groups={}
        for value in sorted(as_list(values),reverse=True):
            value=round(value,8)
            groups[value]=groups.get(value,0)+1
        parts=[]
        for value,count in groups.items():
            part='s' if abs(value)<1e-12 else '(s'+('+' if value<0 else '-')+f'{abs(value):.8g}'+')'
            parts.append(part+(f'^{{{count}}}' if count>1 else ''))
        return ''.join(parts)
    poles=list(as_list(model['Poles']));gain=model['Gain']
    if remove_integrator:
        poles.remove(0);gain=-gain
    return r'\frac{'+f'{gain:.8g}'+factors(model['Zeros'])+'}{'+factors(poles)+'}'


def current_values():
    m=FINAL
    return {
        'kfinal':zpk_formula(CONTROLLERS[-1]),
        'fformula':zpk_formula(CONTROLLERS[-1],True),
        'bw':number(m['Bandwidth_radps']),'wc':number(m['Crossover_radps']),
        'tsfinal':number(m['SettlingFinal_s']),'tscommand':number(m['SettlingCommand_s']),
        'sse':number(m['SSE_pct']),'os':number(m['Overshoot_pct']),
        'pm':number(m['PhaseMargin_deg']),'gm':number(m['GainMargin_dB']),
        'lnoise':number(m['L20_dB']),'tnoise':number(m['T20_dB']),
        'demin':number(m['StepElevatorMin_deg'],4),'demax':number(m['StepElevatorMax_deg'],4),
        'os50':number(DATA['Actuators'][2]['Overshoot_pct'],2),
        'os100':number(DATA['Actuators'][3]['Overshoot_pct'],2),
        'ts100':number(DATA['Actuators'][3]['SettlingFinal_s']),
        'uncpass':str(sum(x['SSE_pct']<1 and x['Overshoot_pct']<20 and x['SettlingFinal_s']<3 for x in DATA['Uncertainty'])),
        'uncnoise':str(sum(x['NoisePassed'] for x in DATA['Uncertainty'])),
    }


PREAMBLE = r"""
\usepackage[margin=21mm]{geometry}
\usepackage{amsmath,amssymb,booktabs,graphicx,float,longtable,tikz,xcolor}
\usetikzlibrary{arrows.meta,positioning,calc}
\usepackage[colorlinks=true,linkcolor=blue!55!black,urlcolor=blue!55!black]{hyperref}
\setlength{\parskip}{0.4em}\setlength{\emergencystretch}{2em}
\setcounter{tocdepth}{1}
\newcommand{\file}[1]{\texttt{\detokenize{#1}}}
\newcommand{\Kfinal}{@@kfinal@@}
\newcommand{\Gplant}{\frac{-88.196939s(s+4.704978)(s+0.06347365)}{D_a(s)}}
"""

DIAGRAM = r"""
\begin{figure}[H]\centering
\resizebox{\linewidth}{!}{\begin{tikzpicture}[>=Latex,block/.style={draw,minimum height=1.1cm,align=center},font=\small]
\node (r) at (0,0) {$q_c$};
\node[draw,circle,minimum size=6mm] (sum) at (1.3,0) {$\Sigma$};
\node[block,minimum width=2.4cm] (k) at (3.4,0) {$K_q(s)$\\$=-F_q(s)/s$};
\node[block] (a) at (7.1,0) {Actuator\\$A_e(s)=1$ nominal};
\node[block] (g) at (10.3,0) {$G_{q\delta_e}(s)$};
\node (y) at (12.3,0) {$q$};
\draw[->](r)--node[above]{$+$}(sum);
\draw[->](sum)--node[above]{$e_q$}(k);
\draw[->](k)--node[above]{$\Delta\delta_{e,c}$}(a);
\draw[->](a)--node[above]{$\Delta\delta_e$}(g);
\draw[->](g)--(y);
\node[block] (h) at (7.1,-1.8) {Sensor $H_q(s)=1$\\$q_m=q+n_q$};
\draw[->](11.6,0)|-(h);
\draw[->](h)-|node[pos=.3,above]{$q_m$}(sum) node[pos=.95,left]{$-$};
\draw[->](7.1,-3.2)node[below]{$n_q$}--(h.south);
\end{tikzpicture}}
\caption{Pitch-rate feedback. Physical elevator is $\delta_{e0}+\Delta\delta_e$; nominal trim is $+0.766074^\circ$. All dynamic signals use rad and s.}
\end{figure}
"""

EN_START = r"""
\title{AERO5560 Group 8\\Pitch-Rate Inner-Loop Design}
\author{Group 8}\date{20 October 2026}
\begin{document}\maketitle
\begin{abstract}
Three pitch-rate compensators are compared at 200 knots and 300 ft. The selected design achieves a closed-loop bandwidth of @@bw@@ rad/s, open-loop crossover of @@wc@@ rad/s, settling time of @@tsfinal@@ s, steady-state error of @@sse@@\% and overshoot of @@os@@\%. Validation covers adjacent airspeeds, assumed actuator dynamics, parameter uncertainty, nonlinear manoeuvres, measurement noise and digital implementation.
\end{abstract}
\tableofcontents\clearpage
\section{Requirements and Model}
Longitudinal Task 1 requires $G_{q\delta_e}$, a labelled feedback diagram and at least three compensators supported by root-locus, Bode and step-response plots. For a $4^\circ$/s step, the targets are settling within 3 s, steady-state error below 1\%, overshoot below 20\%, elevator within $\pm25^\circ$, frequency in the 2--10 rad/s range, and a stated noise bandwidth of 20 rad/s.

We report open-loop crossover $|L(j\omega_{gc})|=1$ and closed-loop bandwidth $|T(j\omega_{BW})|=10^{-3/20}|T(0)|$ separately [1,2]. Settling uses both final-value and command-centred $\pm2\%$ bands; overshoot is relative to the final value. Tracking metrics use 120 s of linear response, and elevator extrema use 0--10 s. Nonlinear tests use a 5 s step and a 2 s pulse within a 10 s simulation.

Lecture 8-2 (slide 8) and Lecture 9-1 (slides 9 and 17) specify 40 dB attenuation at the noise threshold. At 20 rad/s, the open-loop magnitude is $@@lnoise@@$ dB and the closed-loop noise magnitude is $@@tnoise@@$ dB. Both satisfy the target. A 20--$10^5$ rad/s sweep confirms at least 40 dB attenuation throughout the checked noise band.

\subsection{Aircraft and Longitudinal Dynamics}
The aircraft has $m=750$ kg, $S=11.9$ m$^2$, $c=0.85$ m and $I_{yy}=1766.52$ kg m$^2$. At $V=102.888$ m/s and $h=91.44$ m, density is @@rho@@ kg/m$^3$ and dynamic pressure is @@qbar@@ Pa. Trim throttle is 0.770317 and elevator is $+0.766074^\circ$.

The fixed-altitude model uses $[\Delta u,\Delta w,\Delta q,\Delta\theta]^T$, fixed throttle and zero gust. Its converged central-difference Jacobians give
\begin{align}
G_{q\delta_e}(s)&=\Gplant,\\
D_a(s)&=(s+4.231362-j3.300400)(s+4.231362+j3.300400)\notag\\
&\quad\times(s+0.0303971-j0.0772921)(s+0.0303971+j0.0772921).
\end{align}
Both longitudinal modes are retained. The short-period mode has $\omega_{n,sp}=5.36629$ rad/s and $\zeta_{sp}=0.788508$.

\section{Bandwidth Selection}
Bandwidth is selected using modal frequency, control authority, noise and actuator response. The selected @@bw@@ rad/s is below the short-period frequency; its reduced frequency is $c\omega/(2V)=@@reduced@@$, with useful separation from the 20 rad/s noise threshold.

The previous 6.707 rad/s design missed the noise target. Adding a high-frequency pole and retuning the low-frequency poles and zeros reduces bandwidth while preserving tracking. The following fixed-shape gain sweep shows the cost of raising crossover: weaker attenuation and eventually unstable tracking.

@@trade_table@@
@@trade_figure@@
A 0.5--1 rad/s starting target for the future vertical-speed loop provides useful separation. The complete nested loop will require a coupled stability check.

\section{Feedback and Sensitivity Functions}
@@diagram@@
Negative plant gain requires negative controller gain. For SISOtool, define
\begin{equation}
P_q=\operatorname{minreal}(-G/s),\qquad K_q=-F_q/s,\qquad L=P_qF_q=GK_q.
\end{equation}
The single external integrator cancels the plant's origin zero in the minimal q channel (Lecture 10-1). SISOtool uses plant $P_q$ and compensator $F_q$. Controller poles and zeros are real; the final loop is strictly proper.
\begin{align}
S&=\frac{1}{1+L},& T&=\frac{L}{1+L}=1-S,& U&=K_qS,\\
q&=Tq_c+GSd_e+Sd_q-Tn_q,&\frac{\Delta\delta_e}{q_c}&=U,&\frac{\Delta\delta_e}{n_q}&=-U.
\end{align}
Here $d_e$, $d_q$ and $n_q$ denote plant-input disturbance, output disturbance and measurement noise. $T$ is the course complementary sensitivity $C$. Numerical ZPK forms are in \file{zpk.txt}.

Since $G\sim-132.596s$ at low frequency, the compensated minimal loop is type 0:
\begin{equation}
\frac{e_{ss}}{q_c}=\frac{1}{1+L(0)},\qquad L(0)>99\ \text{for error below }1\%.
\end{equation}
The final DC loop gain is @@ldc@@ (@@ldcdb@@ dB). Lag compensation raises low-frequency gain while limiting crossover gain.

\section{Compensator Iterations}
\subsection{Iteration 1: Integral Gain}
\begin{equation}K_{q,1}(s)=-0.1961/s.\end{equation}
Crossover is approximately 3 rad/s, but steady-state error is 3.70\% and the slow tail gives 10.92 s settling. Both targets require improvement.
@@iteration_1@@
\clearpage\subsection{Iteration 2: Lag and Roll-Off}
\begin{equation}K_{q,2}(s)=\frac{-5.835(s+0.5)}{s(s+0.1)(s+30)}.\end{equation}
The lag zero/pole ratio of five reduces steady-state error to 0.77\%. Overshoot remains below 20\%, but phase loss and slow dynamics leave settling at 4.55 s.
@@iteration_2@@
\clearpage\subsection{Iteration 3: Low-Frequency Compensation and Noise Roll-Off}
\begin{equation}\boxed{K_q(s)=\Kfinal.}\end{equation}
The SISOtool compensator is
\begin{equation}F_q(s)=@@fformula@@.\end{equation}
Low-frequency compensation supplies DC gain; poles at 10.7 and 14.5 rad/s supply the additional noise roll-off. After origin cancellation, the open loop has eight poles and four zeros. Parameters were refined against the nominal model, retrimmed 180-knot model and assumed 50 ms actuator, then simplified, rounded and rechecked.
@@iteration_3@@
\clearpage\subsection{Performance Comparison}
@@comparison_table@@
Steady q is @@qfinal@@ deg/s for a 4 deg/s command. Over 0--10 s, absolute elevator is @@demin@@--@@demax@@ deg and peak rate is @@derate@@ deg/s, leaving substantial position margin.
@@comparison_figure@@

\clearpage\section{Engineering Validation}
\subsection{Airspeed}
The nonlinear model was retrimmed at 180 and 210 knots by solving for level-flight angle of attack, throttle and elevator at 300 ft. Central-difference Jacobians were recomputed with the controller fixed. Throttle remains within $[0,1]$ and trim residuals are below $10^{-6}$.
@@speed_table@@
All three sampled points meet the main time-domain and noise targets.

\subsection{Actuator and Delay}
Servo and sensor dynamics are unspecified. Separate actuator sensitivity tests assume $A_e(s)=1/(1+\tau s)$:
@@actuator_table@@
The 50 ms case passes the main step targets. The 100 ms case is stable but exceeds the settling and overshoot limits. Airspeed and actuator cases were tested separately.

Pure delay reduces phase margin by $\omega_{gc}\tau_d$ radians. Nominal delay margin is @@delaymargin@@ s; 10--40 ms cases are in \file{delay.csv}. Position limiting and conditional integration are implemented [4]. The test manoeuvres remain below the position and assumed 60 deg/s rate limits, so saturation recovery remains untested [5].

\subsection{Uncertainty and Digital Implementation}
Pitch stiffness $A_{32}$, damping $A_{33}$ and elevator effectiveness are varied independently by $\pm20\%$. All 27 minimal q channels remain stable; @@uncpass@@ meet all time-domain targets and @@uncnoise@@ meet the noise criterion. Overshoot spans @@uncosmin@@--@@uncosmax@@\%, settling @@unctsmin@@--@@unctsmax@@ s, and phase margin @@uncpmmin@@--@@uncpmmax@@ deg.

At 100 Hz, a ZOH plant and Tustin controller give tracking bandwidth @@dbw@@ rad/s and maximum dynamic pole magnitude @@dmax@@. The unreduced discrete system retains a neutral pole at $z=1$. Hardware latency, quantisation, jitter and anti-alias filtering remain implementation inputs.
@@robustness_figure@@

\clearpage\section{Nonlinear Response and Internal Dynamics}
The unreduced aircraft/controller system has one zero eigenvalue; all remaining eigenvalues have negative real parts. Because $\dot\theta=q$, sustained nonzero q produces a pitch-angle ramp. The origin pole in $U=K_qS$ also permits control drift under persistent commands or sensor bias. Stable q tracking therefore requires attitude and command management in the outer loop.

The supplied 12-state nonlinear model uses converged implicit aerodynamic rates. A 5 s ideal-actuator step differs from the linear q response by at most @@stepdiff@@ deg/s. For a 2 s rate pulse:
\begin{itemize}
\item Peak q: @@pulseq@@ deg/s; peak absolute elevator: @@pulsede@@ deg; peak rate: @@pulserate@@ deg/s.
\item Maximum pitch angle: @@pulsetheta@@ deg; minimum speed: @@pulsev@@ m/s with fixed throttle.
\item Peak body-normal load: @@pulseload@@ g; maximum linear/nonlinear q difference: @@pulsediff@@ deg/s.
\end{itemize}
At 200 knots and 4 deg/s, $Vq/g\approx0.73$, explaining the significant normal-load increment despite small elevator motion. Stall and structural margins require $C_{L\max}$ and load-limit data. Speed loss motivates auto-throttle integration.
@@nonlinear_figure@@

\subsection{Measurement Noise}
A 10 s sensor-noise test assumes 0.02 deg/s RMS and a 20 rad/s low-pass filter. Physical q RMS is @@noiseq@@ deg/s and elevator-increment RMS is @@noisede@@ deg. Nominal $|T(j20)|=@@noise20@@$ dB. The one-sided equivalent area $\int_0^\infty|T(j\omega)/T(0)|^2d\omega$ is @@enbw@@ rad/s. Elevator RMS is specific to this finite record because its path retains an integrator.

\section{Reproduction and Next Steps}
From the assignment root:
\begin{verbatim}
run('inner_loop/run_inner.m')
open_sisotool(3)
\end{verbatim}
The first command rebuilds controllers, validation, ZPK exports and plots; the second opens the selected SISOtool model. Keep the adjacent \file{Gqde}, \file{A_B_Matrix} and \file{trim} folders. GUI steps are normalised; numerical tests use the physical 4 deg/s input in radians.

The design uses Lectures 4--5 (frequency response and modal scales), 8 (root locus, margins and compensation), 9 (aircraft control structure and digital implementation), and 10 (integrator placement, sign and SCK form). Lectures 6--7 and 11 support the later stochastic gust analysis.

Next derive $q_c\rightarrow V_s$ and design the vertical-speed loop, followed by auto-throttle, Dryden gust analysis and full integration.
"""

ZH_START = r"""
\title{AERO5560 Group 8\\俯仰角速度内环设计}
\author{Group 8}\date{2026年10月20日}
\begin{document}\maketitle
\begin{abstract}
针对约 200 节、300 ft 配平点比较三种俯仰角速度补偿器。最终闭环带宽为 @@bw@@ rad/s，开环交越频率为 @@wc@@ rad/s，调节时间 @@tsfinal@@ s，稳态误差 @@sse@@\%，超调 @@os@@\%。验证包括邻近空速、假设舵机动态、参数扰动、非线性机动、测量噪声和数字实现。
\end{abstract}
\tableofcontents\clearpage
\section{要求与模型}
纵向 Task 1 要求提取 $G_{q\delta_e}$、绘制反馈框图，并比较至少三种补偿器，给出根轨迹、Bode 和阶跃响应。对 $4^\circ$/s 阶跃，要求调节时间不超过 3 s、稳态误差小于 1\%、超调小于 20\%、升降舵位于 $\pm25^\circ$ 内；频率目标为 2--10 rad/s，噪声带宽为 20 rad/s。

分别报告开环交越频率 $|L(j\omega_{gc})|=1$ 与闭环带宽 $|T(j\omega_{BW})|=10^{-3/20}|T(0)|$ [1,2]。调节时间同时按最终值和指令值的 $\pm2\%$ 范围计算，超调以最终值为基准。线性角速度指标检查 120 s，舵角极值检查前 10 s；非线性测试采用 5 s 阶跃，以及总仿真 10 s、持续 2 s 的脉冲。

Lecture 8-2 第 8 页及 Lecture 9-1 第 9、17 页采用噪声阈值处衰减 40 dB 的标准。20 rad/s 开环幅值为 $@@lnoise@@$ dB，闭环噪声传递为 $@@tnoise@@$ dB，均满足要求。20--$10^5$ rad/s 扫描确认整个检查频段的衰减至少为 40 dB。

\subsection{飞机参数与纵向动态}
飞机质量 750 kg、翼面积 11.9 m$^2$、弦长 0.85 m、俯仰惯量 1766.52 kg m$^2$。名义速度 102.888 m/s、高度 91.44 m，密度 @@rho@@ kg/m$^3$、动压 @@qbar@@ Pa；配平油门 0.770317，升降舵 $+0.766074^\circ$。

固定高度模型采用 $[\Delta u,\Delta w,\Delta q,\Delta\theta]^T$，固定油门、零阵风。由收敛后的中心差分矩阵得到
\begin{align}
G(s)&=\Gplant,\\
D_a(s)&=(s+4.231362-j3.300400)(s+4.231362+j3.300400)\notag\\
&\quad\times(s+0.0303971-j0.0772921)(s+0.0303971+j0.0772921).
\end{align}
保留全部纵向模态。短周期固有频率为 5.36629 rad/s，阻尼比为 0.788508。

\section{带宽选择}
综合模态频率、舵效、噪声与执行机构响应，选择闭环带宽 @@bw@@ rad/s，低于短周期固有频率。对应约化频率 $c\omega/(2V)=@@reduced@@$，与 20 rad/s 噪声阈值保留频率间隔。

此前 6.707 rad/s 方案未满足噪声要求。新增高频极点并重新调整低频零极点后，带宽降低，跟踪指标保持达标。下表固定零极点、只改变增益，展示提高交越频率造成的噪声衰减减弱及稳定性损失。

@@trade_table@@
@@trade_figure@@
后续垂直速度外环可从 0.5--1 rad/s 开始设计，并检查串级系统的耦合稳定性。

\section{反馈与灵敏度函数}
@@diagram@@
飞机传递函数增益为负，因此控制器取负增益。SISOtool 使用
\begin{equation}
P_q=\operatorname{minreal}(-G/s),\qquad K_q=-F_q/s,\qquad L=P_qF_q=GK_q.
\end{equation}
按 Lecture 10-1，在外部加入一个积分器，抵消最小 q 通道的原点零点；工具中的 plant 为 $P_q$、compensator 为 $F_q$。控制器零极点均为实数，最终开环严格真有理。
\begin{equation}S=\frac{1}{1+L},\qquad T=\frac{L}{1+L}=1-S,\qquad U=K_qS.\end{equation}
指令到 q 为 $T$，到舵面为 $U$；测量噪声到 q 为 $-T$，到舵面为 $-U$。若 $d_e$、$d_q$ 分别作用于飞机输入和输出，则 $q=Tq_c+GSd_e+Sd_q-Tn_q$。$T$ 对应课程 SCK 中的互补灵敏度 $C$；数值 ZPK 见 \file{zpk.txt}。

低频时 $G\sim-132.596s$，原点抵消后的最小开环为 type 0：
\begin{equation}\frac{e_{ss}}{q_c}=\frac{1}{1+L(0)},\qquad L(0)>99\ \text{时误差小于 }1\%.\end{equation}
最终直流开环增益为 @@ldc@@，即 @@ldcdb@@ dB。滞后补偿提高低频增益，同时控制交越附近的增益。

\section{补偿器迭代}
\subsection{第一轮：积分增益}
\begin{equation}K_{q,1}(s)=-0.1961/s.\end{equation}
开环交越频率约 3 rad/s，但稳态误差为 3.70\%，慢尾部使调节时间达到 10.92 s，两项均需改进。
@@iteration_1@@
\clearpage\subsection{第二轮：滞后与高频滚降}
\begin{equation}K_{q,2}(s)=\frac{-5.835(s+0.5)}{s(s+0.1)(s+30)}.\end{equation}
滞后零极点比为五，稳态误差降至 0.77\%，超调低于 20\%；相位损失与慢动态使调节时间仍有 4.55 s。
@@iteration_2@@
\clearpage\subsection{第三轮：低频补偿与噪声滚降}
\begin{equation}\boxed{K_q(s)=\Kfinal.}\end{equation}
SISOtool 中的补偿器为
\begin{equation}F_q(s)=@@fformula@@.\end{equation}
低频补偿提高直流增益，10.7 与 14.5 rad/s 极点提供额外噪声滚降。原点抵消后开环为八极点、四零点。参数针对名义模型、重新配平的 180 节模型和假设的 50 ms 舵机调整，再简化、取整复核。
@@iteration_3@@
\clearpage\subsection{性能对照}
@@comparison_table@@
对 4 deg/s 指令，稳态 q 为 @@qfinal@@ deg/s。前 10 s 的绝对升降舵为 @@demin@@--@@demax@@ 度，最大舵速 @@derate@@ deg/s，舵面位置有较大余量。
@@comparison_figure@@

\clearpage\section{工程验证}
\subsection{空速}
在 180、210 节及 300 ft 下求解平飞迎角、油门和升降舵，再用中心差分重新线性化，控制器保持不变。油门位于 $[0,1]$，配平残差小于 $10^{-6}$。
@@speed_table@@
三个离散测试点均满足主要时域与噪声指标。

\subsection{舵机与时延}
原始数据未提供舵机与传感器动态。舵机敏感性测试独立采用 $A_e(s)=1/(1+\tau s)$：
@@actuator_table@@
50 ms 工况满足主要阶跃指标；100 ms 工况的最小 q 通道稳定，但调节时间与超调超限。空速与舵机测试分开进行。

纯时延使相位裕度损失 $\omega_{gc}\tau_d$ 弧度，名义时延裕度约 @@delaymargin@@ s；10--40 ms 的结果见 \file{delay.csv}。非线性实现包含位置限幅和条件积分 [4]。当前测试均未触及位置及假设的 60 deg/s 舵速限制，饱和恢复性能仍待验证 [5]。

\subsection{参数扰动与数字实现}
俯仰刚度 $A_{32}$、阻尼 $A_{33}$ 和升降舵效能分别变化 $\pm20\%$，共 27 种组合。所有最小 q 通道稳定，其中 @@uncpass@@ 种满足全部时域指标，@@uncnoise@@ 种满足噪声指标。超调范围 @@uncosmin@@--@@uncosmax@@\%，调节时间 @@unctsmin@@--@@unctsmax@@ s，相位裕度 @@uncpmmin@@--@@uncpmmax@@ 度。

以 100 Hz 采样、飞机 ZOH、控制器 Tustin 离散化，最小闭环带宽为 @@dbw@@ rad/s，最大动态极点模为 @@dmax@@；未约简系统保留一个 $z=1$ 中性极点。实现时还需确定硬件延迟、量化、时钟抖动和抗混叠滤波。
@@robustness_figure@@

\clearpage\section{非线性响应与内部动态}
未约简飞机与控制器系统含一个零特征值，其余特征值实部为负。由 $\dot\theta=q$，持续非零 q 产生俯仰角斜坡；$U=K_qS$ 的原点极点也使持续指令或传感器偏置引起控制漂移。外环需要管理姿态与指令。

采用原始十二状态非线性模型，并迭代收敛隐式气动导数。理想舵机 5 s 阶跃中，线性与非线性 q 最大差为 @@stepdiff@@ deg/s。2 s 角速度脉冲的结果为：
\begin{itemize}
\item 峰值 q：@@pulseq@@ deg/s；最大绝对舵角：@@pulsede@@ 度；最大舵速：@@pulserate@@ deg/s。
\item 最大俯仰角：@@pulsetheta@@ 度；固定油门下最低速度：@@pulsev@@ m/s。
\item 机体法向载荷峰值：@@pulseload@@ g；线性与非线性 q 最大差：@@pulsediff@@ deg/s。
\end{itemize}
200 节、4 deg/s 时 $Vq/g\approx0.73$，小舵角即可产生显著法向载荷。失速和结构裕度需要 $C_{L\max}$ 及载荷限制数据；速度损失说明后续需要自动油门。
@@nonlinear_figure@@

\subsection{测量噪声}
10 s 噪声测试采用假设的 0.02 deg/s RMS 测量噪声及 20 rad/s 低通。实际 q RMS 为 @@noiseq@@ deg/s，舵面增量 RMS 为 @@noisede@@ 度；名义 $|T(j20)|=@@noise20@@$ dB。单侧等效面积 $\int_0^\infty|T/T(0)|^2d\omega$ 为 @@enbw@@ rad/s。舵面路径含积分器，其 RMS 仅适用于该有限记录。

\section{复现与后续工作}
从作业根目录运行：
\begin{verbatim}
run('inner_loop/run_inner.m')
open_sisotool(3)
\end{verbatim}
第一条命令重建控制器、验证结果、ZPK 和图表；第二条打开所选 SISOtool 模型。保留相邻 \file{Gqde}、\file{A_B_Matrix}、\file{trim} 文件夹。GUI 阶跃为归一化响应，数值测试以 rad/s 输入实际 4 deg/s 指令。

相关知识来自 Lecture 4--5（频响与模态）、8（根轨迹、裕度与补偿）、9（飞机控制结构与数字实现）、10（积分器配置、符号与 SCK）；Lecture 6--7、11 用于后续随机阵风分析。

下一步提取 $q_c\rightarrow V_s$，设计垂直速度外环、自动油门，并完成阵风分析与系统集成。
"""

REFERENCES = r"""
\section*{References / 参考资料}
\addcontentsline{toc}{section}{References / 参考资料}
\begin{enumerate}
\item MathWorks, \href{https://www.mathworks.com/help/control/ref/dynamicsystem.bandwidth.html}{bandwidth: frequency response bandwidth}.
\item MathWorks, \href{https://www.mathworks.com/help/control/ref/dynamicsystem.margin.html}{margin: gain/phase margins and crossover frequencies}.
\item MathWorks, \href{https://www.mathworks.com/help/control/ug/bode-diagram-design.html}{Bode diagram design}, and \href{https://www.mathworks.com/help/control/ref/controlsystemdesigner-app.html}{Control System Designer}.
\item MathWorks, \href{https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html}{Anti-windup control using PID Controller block}.
\item R. A. Hess and S. A. Snell, \href{https://ntrs.nasa.gov/citations/19980017538}{Flight Control System Design with Rate Saturating Actuators}, NASA/CR-97-207117, 1997.
\item AERO5560 Group Project, Longitudinal Task 1, pp. 2--3; Lectures 4, 5, 8, 9, 10 and 11, supplied course material. Aircraft data: Group 8 Canvas package.
\end{enumerate}
\end{document}
"""


def make_report(english):
    lang = "en" if english else "zh"
    title = EN_START if english else ZH_START
    with (HERE / "nonlinear_summary.csv").open(newline="", encoding="utf-8") as stream:
        nonlinear = {r["Scenario"]: r for r in csv.DictReader(stream)}
    pulse = nonlinear["pulse"]
    unc = DATA["Uncertainty"]
    values = {"bw": number(FINAL["Bandwidth_radps"]), "wc": number(FINAL["Crossover_radps"]),
              "hz": number(FINAL["Bandwidth_radps"] / (2*3.141592653589793)),
              "rho": number(DATA["Flight"]["Density_kgpm3"], 5),
              "qbar": number(DATA["Flight"]["DynamicPressure_Pa"], 1),
              "reduced": number(DATA["Flight"]["ReducedFrequency"], 5),
              "ldc": number(FINAL["DC"]/(1-FINAL["DC"]), 2),
              "ldcdb": number(20*math.log10(FINAL["DC"]/(1-FINAL["DC"])), 2),
              "qfinal": number(4*FINAL["DC"], 4), "demin": number(FINAL["StepElevatorMin_deg"], 4),
              "demax": number(FINAL["StepElevatorMax_deg"], 4),
              "derate": number(FINAL["StepElevatorRateMax_degps"], 3),
              "delaymargin": number(DATA["Flight"]["NominalDelayMarginEstimate_s"], 3),
              "uncpass": str(sum(x["SSE_pct"]<1 and x["Overshoot_pct"]<20 and x["SettlingFinal_s"]<3 for x in unc)),
              "dbw": number(DATA["Digital"]["MinimalTrackingBandwidth_radps"]),
              "dmax": number(DATA["Digital"]["DynamicMaxPoleMagnitude"], 9),
              "stepdiff": number(float(nonlinear["step"]["MaxLinearQDifference_degps"]), 4),
              "pulseq": number(float(pulse["PeakPitchRate_degps"]), 4),
              "pulsede": number(float(pulse["PeakAbsElevator_deg"]), 4),
              "pulserate": number(float(pulse["PeakElevatorRate_degps"]), 3),
              "pulsetheta": number(float(pulse["MaxAbsTheta_deg"]), 3),
              "pulsev": number(float(pulse["SpeedMin_mps"]), 3),
              "pulseload": number(float(pulse["LoadMax_g"]), 3),
              "pulsediff": number(float(pulse["MaxLinearQDifference_degps"]), 4),
              "noiseq": number(DATA["Noise"]["PitchRateRMS_degps"], 5),
              "noisede": number(DATA["Noise"]["ElevatorRMS_deg"], 6),
              "noise20": number(FINAL["T20_dB"], 2),
              "enbw": number(FINAL["EquivalentNoiseBandwidth_radps"], 3)}
    values.update(current_values())
    for stem, key in [("uncos", "Overshoot_pct"), ("uncts", "SettlingFinal_s"), ("uncpm", "PhaseMargin_deg")]:
        values[stem+"min"] = number(min(x[key] for x in unc), 2)
        values[stem+"max"] = number(max(x[key] for x in unc), 2)
    rows = []
    for key, label, spec in [
        ("Bandwidth_radps", r"$\omega_{BW}$ (rad/s)", "2--10"),
        ("Crossover_radps", r"$\omega_{gc}$ (rad/s)", "2--10"),
        ("SettlingFinal_s", r"$t_s$, final / 最终值 (s)" if not english else r"$t_s$, final (s)", r"$\leq3$"),
        ("SettlingCommand_s", r"$t_s$, command / 指令 (s)" if not english else r"$t_s$, command (s)", r"$\leq3$"),
        ("SSE_pct", "SSE (\\%)", r"$<1$"), ("Overshoot_pct", "Overshoot (\\%)", r"$<20$"),
        ("PhaseMargin_deg", "PM (deg)", "--"), ("GainMargin_dB", "GM (dB)", "--"),
        ("T20_dB", r"$|T(j20)|$ (dB)", r"$\leq-40$")]:
        rows.append([label]+[number(m[key], 3) for m in METRICS]+[spec])
    rows.append([r"$|L(j20)|$ (dB)"]+[number(x,3) for x in LOOP20_DB]+[r"$\leq-40$"])
    values["comparison_table"] = table(["Metric" if english else "指标", "1", "2", "3", "Target" if english else "目标"], rows,
        "Three compensator iterations; a dash denotes no explicit assignment threshold." if english else "三轮补偿器对照；横线表示任务书没有明确数值门槛。")
    trade = [x for x in DATA["Trade"] if x["TargetCrossover_radps"] in [2,3,4,6,8,10]]
    values["trade_table"] = table([r"$\omega_{gc}$", r"$\omega_{BW}$", r"$t_s$ (s)", "OS (\\%)", "PM (deg)", r"$|T(j20)|$ (dB)"],
       [[number(x[k],2) for k in ["Crossover_radps","Bandwidth_radps","SettlingFinal_s","Overshoot_pct","PhaseMargin_deg","T20_dB"]] for x in trade],
       "Gain-only frequency trade; frequencies in rad/s." if english else "只改变增益的频率取舍；频率单位 rad/s。")
    values["speed_table"] = table(["Speed (kt)", "Throttle", "SSE (\\%)", "OS (\\%)", r"$t_s$ (s)", r"$\omega_{BW}$"],
       [[str(x["Speed_knots"])]+[number(x[k],3) for k in ["Throttle","SSE_pct","Overshoot_pct","SettlingFinal_s","Bandwidth_radps"]] for x in DATA["Envelope"]],
       "Fixed controller at separately verified trim points." if english else "同一控制器在独立验证配平点的表现。")
    values["actuator_table"] = table([r"$\tau$ (s)", "SSE (\\%)", "OS (\\%)", r"$t_s$ (s)", "PM (deg)"],
       [[number(x[k],3) for k in ["AssumedTimeConstant_s","SSE_pct","Overshoot_pct","SettlingFinal_s","PhaseMargin_deg"]] for x in DATA["Actuators"]],
       "Assumed first-order actuator dynamics." if english else "假设一阶舵机动态。")
    captions = {
        "trade_figure": ("frequency_trade", "Gain-only frequency trade." if english else "固定补偿器形状时的频率取舍。"),
        "comparison_figure": ("comparison", "Tracking, absolute elevator, complementary sensitivity and sensitivity." if english else "角速度跟踪、绝对舵角、互补灵敏度与灵敏度对照。"),
        "robustness_figure": ("robustness", "Separate speed, actuator, parameter and finite-noise checks." if english else "分别进行的空速、舵机、参数扰动与有限记录噪声检查。"),
        "nonlinear_figure": ("nonlinear", "Finite pitch-rate pulse; assumed actuator is 50 ms and 60 deg/s." if english else "有限角速度脉冲；假设舵机为 50 ms、60 deg/s。")}
    for j in range(1,4):
        captions[f"iteration_{j}"] = (f"iteration_{j}",
            f"Iteration {j}: root-locus detail, Bode and 4 deg/s step; red points mark selected tracking poles."
            if english else f"第 {j} 轮：根轨迹局部、Bode 与 4 deg/s 阶跃；红点为选定闭环极点。")
    for key,(name,caption) in captions.items():
        values[key] = figure(name,caption)
    values["diagram"] = DIAGRAM
    body = title
    for key,value in values.items():
        body = body.replace("@@"+key+"@@",value)
    assert "@@" not in body
    klass = (r"\documentclass[a4paper,11pt]{article}" if english else
             r"\documentclass[UTF8,a4paper,11pt,fontset=windows]{ctexart}")
    references = REFERENCES
    if english:
        references = references.replace("References / 参考资料", "References")
    preamble = PREAMBLE.replace("@@kfinal@@",current_values()["kfinal"])
    output = "% !TeX program = xelatex\n"+klass+preamble+body+references
    name = "inner_loop_report_english.tex" if english else "inner_loop_report.tex"
    (HERE/name).write_text(output,encoding="utf-8",newline="\n")
    print(f"Wrote {name} ({lang})")


def update_progress_reports():
    """Update cumulative summaries from the verified metrics."""
    values=current_values()
    with (HERE/'nonlinear_summary.csv').open(encoding='utf-8') as stream:
        pulse=next(row for row in csv.DictReader(stream) if row['Scenario']=='pulse')
    values.update(pulseload=number(float(pulse['LoadMax_g']),3),pulsev=number(float(pulse['SpeedMin_mps']),3))
    for english,name in [(False,'report.tex'),(True,'report_english_version.tex')]:
        path=HERE.parent/'AI_draft'/name;text=path.read_text(encoding='utf-8')
        if english:
            markers=[r'\clearpage\section{Pitch-Rate Inner-Loop Design}']
            end_marker=r'\section{Files, Reproduction and Sources}'
            abstract=r"Group 8 aircraft data, trim verification, numerical $A/B$ matrices and $G_{q\delta_e}$ support three pitch-rate compensator designs. The selected loop achieves @@bw@@ rad/s bandwidth, @@tsfinal@@ s settling, @@sse@@\% error and @@os@@\% overshoot. At 20 rad/s, open-loop and closed-loop noise magnitudes are @@lnoise@@ and @@tnoise@@ dB. Nominal targets are met; off-nominal and nonlinear tests define the checked range."
            section=r"""
\clearpage\section{Pitch-Rate Inner-Loop Design}
At 200 knots and 300 ft, the selected bandwidth is @@bw@@ rad/s and open-loop crossover is @@wc@@ rad/s. The short-period natural frequency is 5.366 rad/s. Three compensator iterations are:
\begin{align}
K_{q,1}(s)&=-0.1961/s,\\
K_{q,2}(s)&=\frac{-5.835(s+0.5)}{s(s+0.1)(s+30)},\\
K_{q,3}(s)&=@@kfinal@@.
\end{align}
SISOtool uses $P_q=\operatorname{minreal}(-G/s)$ and $F_q=-sK_q$. A single integrator cancels the plant origin zero in the minimal q channel. The type-0 loop requires $L(0)>99$ for error below 1\%. All controller poles and zeros are real, and the loop is strictly proper.
@@summary_table@@
The 4 deg/s step gives absolute elevator @@demin@@--@@demax@@ deg over 0--10 s, phase margin @@pm@@ deg and gain margin @@gm@@ dB. The 20 rad/s noise magnitudes satisfy the $-40$ dB criterion in Lecture 9-1, slides 9 and 17. The checked 20--$10^5$ rad/s band also passes.

Separate retrimmed 180, 200 and 210 knot points pass the main time-domain and noise targets. An assumed 50 ms actuator gives @@os50@@\% overshoot and passes the main nominal targets. At 100 ms, overshoot is @@os100@@\% and settling is @@ts100@@ s, exceeding both limits. Of 27 parameter cases, @@uncpass@@ meet all time-domain thresholds.

The unreduced system retains a neutral mode because $\dot\theta=q$. A 2 s nonlinear pulse reaches @@pulseload@@ g and minimum speed @@pulsev@@ m/s with fixed throttle. Attitude management and auto-throttle remain necessary.
\begin{figure}[H]\centering\includegraphics[width=\linewidth]{../inner_loop/comparison.pdf}
\caption{Three iterations: q, elevator, complementary sensitivity and sensitivity.}\end{figure}
Full plots and validation are in the \href{../inner_loop/inner_loop_report_english.pdf}{inner-loop report}. Run \file{inner_loop/run_inner.m}; inspect using \file{open_sisotool(3)}.

"""
        else:
            markers=[r'\clearpage\section{俯仰角速度内环设计}',r'\clearpage\section{已完成的俯仰角速度内环设计}']
            end_marker=r'\section{文件、复现与来源}'
            abstract=r"本报告给出 Group 8 飞机数据、配平检查、数值 $A/B$ 矩阵、$G_{q\delta_e}$ 与三轮内环设计。所选闭环带宽为 @@bw@@ rad/s，调节时间 @@tsfinal@@ s，稳态误差 @@sse@@\%，超调 @@os@@\%；20 rad/s 开环与闭环噪声幅值分别为 @@lnoise@@、@@tnoise@@ dB。名义指标达标，并完成非名义与非线性验证。"
            section=r"""
\clearpage\section{俯仰角速度内环设计}
200 节、300 ft 时，所选闭环带宽为 @@bw@@ rad/s，开环交越频率为 @@wc@@ rad/s；短周期固有频率为 5.366 rad/s。三轮补偿器为：
\begin{align}
K_{q,1}(s)&=-0.1961/s,\\
K_{q,2}(s)&=\frac{-5.835(s+0.5)}{s(s+0.1)(s+30)},\\
K_{q,3}(s)&=@@kfinal@@.
\end{align}
SISOtool 使用 $P_q=\operatorname{minreal}(-G/s)$ 与 $F_q=-sK_q$。一个积分器抵消最小 q 通道的原点零点；所得 type-0 开环需满足 $L(0)>99$，使误差小于 1\%。控制器零极点均为实数，开环严格真有理。
@@summary_table@@
4 deg/s 阶跃前 10 s 的绝对升降舵为 @@demin@@--@@demax@@ 度，相位裕度 @@pm@@ 度，增益裕度 @@gm@@ dB。20 rad/s 噪声幅值满足 Lecture 9-1 第 9、17 页的 $-40$ dB 标准，20--$10^5$ rad/s 检查频段也全部通过。

独立配平的 180、200、210 节工况满足主要时域和噪声指标。假设 50 ms 舵机的超调为 @@os50@@\%，主要名义指标通过；100 ms 时超调 @@os100@@\%、调节时间 @@ts100@@ s，两项超限。27 种参数扰动中，@@uncpass@@ 种满足全部时域阈值。

由 $\dot\theta=q$，未约简系统保留中性模态。2 s 非线性脉冲的载荷峰值为 @@pulseload@@ g，固定油门时速度降至 @@pulsev@@ m/s，后续仍需姿态管理与自动油门。
\begin{figure}[H]\centering\includegraphics[width=\linewidth]{../inner_loop/comparison.pdf}
\caption{三轮设计的 q、升降舵、互补灵敏度与灵敏度。}\end{figure}
完整图表与验证见\href{../inner_loop/inner_loop_report.pdf}{详细内环报告}。运行 \file{inner_loop/run_inner.m} 复现，使用 \file{open_sisotool(3)} 查看模型。

"""
        rows=[[label]+[number(m[key],3) for m in METRICS] for key,label in [
            ('SettlingFinal_s',r'$t_s$ (s)'),('SettlingCommand_s',r'$t_{s,command}$ (s)'),
            ('SSE_pct',r'SSE (\%)'),('Overshoot_pct',r'Overshoot (\%)'),
            ('Bandwidth_radps',r'$\omega_{BW}$ (rad/s)'),('L20_dB',r'$|L(j20)|$ (dB)'),('T20_dB',r'$|T(j20)|$ (dB)')]]
        section=section.replace('@@summary_table@@',table(['Metric' if english else '指标','1','2','3'],rows,
            'Three compensator iterations' if english else '三轮补偿器指标'))
        for key,value in values.items():section=section.replace('@@'+key+'@@',value);abstract=abstract.replace('@@'+key+'@@',value)
        assert '@@' not in section and '@@' not in abstract
        marker=next(x for x in markers if x in text);start=text.index(marker);end=text.index(end_marker,start)
        text=text[:start]+section.lstrip()+text[end:]
        start=text.index(r'\begin{abstract}')+len(r'\begin{abstract}');end=text.index(r'\end{abstract}',start)
        text=text[:start]+'\n'+abstract+'\n'+text[end:]
        path.write_text(text,encoding='utf-8',newline='\n');print(f'Updated {path.name}')

if __name__ == "__main__":
    make_report(False)
    make_report(True)
    update_progress_reports()
