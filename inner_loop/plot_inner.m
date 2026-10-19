function plot_inner()
% Plot design comparisons and validation results.
folder=fileparts(mfilename('fullpath'));D=load(fullfile(folder,'controller.mat'));
V=load(fullfile(folder,'validation.mat'));R=V.results;
set(groot,'defaultFigureColor','w','defaultAxesColor','w',...
 'defaultAxesXColor','k','defaultAxesYColor','k','defaultTextColor','k',...
 'defaultLegendColor','w','defaultLegendTextColor','k',...
 'defaultAxesFontSize',10,'defaultLineLineWidth',1.5);
col=[0 .447 .741;.85 .325 .098;.15 .5 .2];w=logspace(-3,3,2200);
labels={'1: integral','2: lag + filter','3: noise-constrained design'};

for j=1:3
 L=D.design.Loops{j};T=D.design.Tracking{j};
 fig=figure('Visible','off','Position',[50 50 1150 720]);
 tiledlayout(2,2,'TileSpacing','compact');
 nexttile;[rr,~]=rlocus(L,logspace(-3,3,1200));
 plot(real(rr.'),imag(rr.'),'Color',[.25 .5 .75]);hold on;
 pp=pole(L);zz=zero(L);plot(real(pp),imag(pp),'kx','MarkerSize',7);
 plot(real(zz),imag(zz),'ko','MarkerSize',6);
 cp=pole(T);plot(real(cp),imag(cp),'r.','MarkerSize',15);
 xlim([-15 1]);ylim([-15 15]);grid on;xlabel('Real (s^{-1})');ylabel('Imaginary (s^{-1})');
 title('Root locus: gain multiplier; red = selected poles');
 nexttile;h=squeeze(freqresp(L,w));semilogx(w,20*log10(abs(h)),'Color',col(j,:));
 hold on;yline(0,'k:');grid on;xlim([.01 200]);ylim([-80 60]);
 xlabel('Frequency (rad/s)');ylabel('|L| (dB)');title('Open-loop Bode magnitude');
 nexttile;h=squeeze(freqresp(L,w));semilogx(w,unwrap(angle(h))*180/pi,'Color',col(j,:));
 hold on;yline(-180,'k:');grid on;xlim([.01 200]);
 xlabel('Frequency (rad/s)');ylabel('Phase (deg)');title('Open-loop Bode phase');
 nexttile;H=D.design.Histories{j};plot(H.Time_s,H.PitchRate_degps,'Color',col(j,:));
 hold on;yline(4,'k--');yline(3.92,'k:');yline(4.08,'k:');xline(3,'k:');
 xlabel('Time (s)');ylabel('Pitch rate (deg/s)');title('4 deg/s command; dotted = command +/-2%');grid on;
 sgtitle(sprintf('Compensator iteration %d',j),'Color','k');writeFigure(fig,folder,sprintf('iteration_%d',j));
end

fig=figure('Visible','off','Position',[50 50 1120 720]);tiledlayout(2,2,'TileSpacing','compact');
nexttile;hold on;
for j=1:3,H=D.design.Histories{j};plot(H.Time_s,H.PitchRate_degps,'Color',col(j,:));end
yline(4,'k--');yline(3.92,'k:');yline(4.08,'k:');xline(3,'k:');grid on;
xlabel('Time (s)');ylabel('q (deg/s)');legend(labels,'Location','southeast');title('Pitch-rate tracking');
nexttile;hold on;
for j=1:3,H=D.design.Histories{j};plot(H.Time_s,H.Elevator_deg,'Color',col(j,:));end
grid on;xlabel('Time (s)');ylabel('Absolute elevator (deg)');title('Includes +0.766 deg trim; limits +/-25 deg');
nexttile;hold on;
for j=1:3,h=squeeze(freqresp(D.design.Tracking{j},w));semilogx(w,20*log10(abs(h)),'Color',col(j,:));end
set(gca,'XScale','log');xline(20,'k:');yline(-3,'k:');yline(-40,'k--');grid on;xlim([.01 200]);ylim([-80 10]);
xlabel('Frequency (rad/s)');ylabel('|T| (dB)');title('Closed-loop tracking and sensor-noise transmission');
nexttile;hold on;
for j=1:3,S=minreal(feedback(1,D.design.Loops{j}),1e-8);h=squeeze(freqresp(S,w));semilogx(w,20*log10(abs(h)),'Color',col(j,:));end
set(gca,'XScale','log');grid on;xlim([.001 200]);xlabel('Frequency (rad/s)');ylabel('|S| (dB)');title('Sensitivity: low-frequency error and peak amplification');
writeFigure(fig,folder,'comparison');

fig=figure('Visible','off','Position',[50 50 1120 710]);tiledlayout(2,2,'TileSpacing','compact');
M=struct2table(R.Trade);
nexttile;plot(M.TargetCrossover_radps,M.Bandwidth_radps,'o-');hold on;yline(10,'k:');
xlabel('Selected open-loop crossover (rad/s)');ylabel('Closed-loop -3 dB bandwidth (rad/s)');grid on;
nexttile;plot(M.TargetCrossover_radps,M.PhaseMargin_deg,'o-');hold on;yline(55,'k:');
xlabel('Open-loop crossover (rad/s)');ylabel('Phase margin (deg)');grid on;
nexttile;plot(M.TargetCrossover_radps,M.L20_dB,'o-');hold on;
plot(M.TargetCrossover_radps,M.T20_dB,'s-');yline(-40,'k:');
legend('|L|','|T|','-40 dB','Location','best');
xlabel('Open-loop crossover (rad/s)');ylabel('Magnitude at 20 rad/s (dB)');grid on;
nexttile;plot(M.TargetCrossover_radps,M.PeakElevatorRate_degps,'o-');
xlabel('Open-loop crossover (rad/s)');ylabel('Peak elevator rate (deg/s)');grid on;
sgtitle('Gain sweep with fixed compensator poles and zeros','Color','k');
writeFigure(fig,folder,'frequency_trade');

fig=figure('Visible','off','Position',[50 50 1120 700]);tiledlayout(2,2,'TileSpacing','compact');
H=R.Nonlinear.pulse.History;A=R.Nonlinear.limited.History;
nexttile;plot(H.Time_s,H.Command_degps,'k--');hold on;plot(H.Time_s,H.LinearPitchRate_degps,'b');
plot(H.Time_s,H.PitchRate_degps,'r');plot(A.Time_s,A.PitchRate_degps,'Color',[.2 .6 .2]);
legend('Command','Linear ideal','Nonlinear ideal','Nonlinear assumed servo','Location','best');
xlabel('Time (s)');ylabel('q (deg/s)');grid on;
nexttile;plot(H.Time_s,H.Elevator_deg,'r');hold on;
plot(A.Time_s,A.Elevator_deg,'Color',[.2 .6 .2]);
xlabel('Time (s)');ylabel('Absolute elevator (deg)');grid on;
nexttile;plot(H.Time_s,H.Theta_deg,H.Time_s,H.Alpha_deg);
legend('Pitch angle','Angle of attack');xlabel('Time (s)');ylabel('Angle (deg)');grid on;
nexttile;yyaxis left;plot(H.Time_s,H.Speed_mps);ylabel('Airspeed (m/s)');
yyaxis right;plot(H.Time_s,H.BodyNormalLoad_g);ylabel('Body-normal load (g)');
xlabel('Time (s)');grid on;sgtitle('4 deg/s, 2 s pulse; assumed servo: tau=0.05 s, rate=60 deg/s','Color','k');
writeFigure(fig,folder,'nonlinear');

fig=figure('Visible','off','Position',[50 50 1120 700]);tiledlayout(2,2,'TileSpacing','compact');
nexttile;hold on;t=(0:.005:10)';
for j=1:3
 Pi=minreal(-R.Models{j}.G/tf('s'),1e-7);Ti=minreal(feedback(Pi*D.Fq,1),1e-7);
 plot(t,rad2deg(lsim(Ti,deg2rad(4)*ones(size(t)),t)));
end
yline(4,'k:');grid on;legend('180 knots','200 knots','210 knots');xlabel('Time (s)');ylabel('q (deg/s)');title('Fixed controller, separately retrimmed aircraft');
nexttile;hold on;
for tau=[0 .025 .05 .1]
 Ti=minreal(feedback(D.Lq/(tau*tf('s')+1),1),1e-8);
 plot(t,rad2deg(lsim(Ti,deg2rad(4)*ones(size(t)),t)));
end
legend('Ideal','25 ms','50 ms','100 ms');grid on;xlabel('Time (s)');ylabel('q (deg/s)');title('Assumed first-order actuator lag');
nexttile;U=struct2table(R.Uncertainty);scatter(U.PhaseMargin_deg,U.Overshoot_pct,35,U.Bandwidth_radps,'filled');
colorbar;xlabel('Phase margin (deg)');ylabel('Overshoot (%)');grid on;title('27 parameter cases; colour = bandwidth (rad/s)');
nexttile;nn=readtable(fullfile(folder,'noise.csv'));plot(nn.Time_s,nn.SensorNoise_degps,nn.Time_s,nn.PitchRate_degps);
legend('Assumed sensor noise','Physical q response');grid on;xlabel('Time (s)');ylabel('Rate (deg/s)');title('Finite 10 s noise test: 20 rad/s filter, 0.02 deg/s RMS');
writeFigure(fig,folder,'robustness');
fprintf('PLOTS_COMPLETE\n');
end

function writeFigure(fig,folder,name)
exportgraphics(fig,fullfile(folder,[name '.png']),'Resolution',180,'BackgroundColor','white');
exportgraphics(fig,fullfile(folder,[name '.pdf']),'ContentType','vector','BackgroundColor','white');
close(fig);
end
