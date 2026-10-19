function design = design_inner()
% Build the three compensators and compare step responses.
folder=fileparts(mfilename('fullpath')); root=fileparts(folder);
D=load(fullfile(root,'Gqde','Gqde.mat'),'Gqde'); s=tf('s'); Gq=D.Gqde;
% Pq includes the negative sign and one integrator.
Pq=minreal(-Gq/s,1e-9);
K={-.1961/s; -5.835*(s+.5)/(s*(s+.1)*(s+30)); ...
 zpk(-[.136 .160],[0 -.0015 -.31 -10.7 -14.5],-21.5)};
F=cell(3,1); M=struct([]); loops=cell(3,1); tracking=loops;
models=cell(3,1);
t=(0:.005:10)'; qc=deg2rad(4)*ones(size(t)); histories=cell(3,1);
for j=1:3
 K{j}=zpk(K{j}); F{j}=minreal(-s*K{j},1e-9);
 loops{j}=minreal(Pq*F{j},1e-8); tracking{j}=minreal(feedback(loops{j},1),1e-8);
 metric=inner_metrics(loops{j});
 % Elevator limits apply to absolute deflection, including the trim offset.
 U=minreal(feedback(K{j},Gq),1e-8);
 q=lsim(tracking{j},qc,t); de=lsim(U,qc,t)+.013370521759902757;
 metric.StepElevatorMin_deg=rad2deg(min(de));
 metric.StepElevatorMax_deg=rad2deg(max(de));
 metric.StepElevatorRateMax_degps=max(abs(gradient(rad2deg(de),t)));
 metric.Iteration=j;
 [zz,pp,gg]=zpkdata(K{j},'v');[nn,dd]=tfdata(tf(K{j}),'v');
 models{j}=struct('Gain',gg,'Zeros',zz,'Poles',pp,'Numerator',nn,'Denominator',dd);
 if isempty(M),M=metric;else,M(j)=metric;end
 histories{j}=table(t,rad2deg(qc),rad2deg(q),rad2deg(de),...
  'VariableNames',{'Time_s','Command_degps','PitchRate_degps','Elevator_deg'});
 writetable(histories{j},fullfile(folder,sprintf('step_%d.csv',j)));
end
Kq=K{3}; Fq=F{3}; Lq=loops{3}; Tq=tracking{3};
Sq=minreal(feedback(1,Lq),1e-8); Uq=minreal(feedback(Kq,Gq),1e-8);
design=struct('Gq',Gq,'Pq',Pq,'K',{K},'F',{F},'Loops',{loops},...
 'Tracking',{tracking},'Metrics',M,'Histories',{histories});
save(fullfile(folder,'controller.mat'),'Gq','Pq','Kq','Fq','Lq','Tq','Sq','Uq','design');
writetable(struct2table(M),fullfile(folder,'comparison.csv'));
fid=fopen(fullfile(folder,'controller.txt'),'w');
fprintf(fid,'Error e = qc - q_measured; delta_e = delta_e_trim + Kq*e.\n');
fprintf(fid,'Kq(s) = -21.5(s+0.136)(s+0.160)/[s(s+0.0015)(s+0.31)(s+10.7)(s+14.5)]\n');
fprintf(fid,'Angles in rad; pitch rates in rad/s. Feedback sensor = 1. Nominal actuator = 1.\n');
fprintf(fid,'One origin integrator only. Do not add another integrator in sisotool.\n');
fprintf(fid,'Minimal q-channel stability does not imply all internal states are asymptotically stable.\n');
fclose(fid);
fid=fopen(fullfile(folder,'controller.json'),'w','n','UTF-8');
fwrite(fid,jsonencode(struct('Controllers',{models}),'PrettyPrint',true),'char');fclose(fid);
assert(M(3).Stable && M(3).SSE_pct<1 && M(3).Overshoot_pct<20);
assert(M(3).SettlingFinal_s<3 && M(3).SettlingCommand_s<3);
assert(M(3).Bandwidth_radps>=2 && M(3).Bandwidth_radps<=10);
assert(M(3).Crossover_radps>=2 && M(3).Crossover_radps<=10);
assert(max(abs([M(3).StepElevatorMin_deg M(3).StepElevatorMax_deg]))<25);
assert(M(3).NoisePassed,'The loop must attenuate frequencies >=20 rad/s by at least 40 dB.');
assert(isreal(zero(Kq)) && isreal(pole(Kq)),'Compensator poles and zeros must be real.');
m=M(3);names=["Closed-loop bandwidth";"Open-loop crossover";"Final-value settling";...
 "Command settling";"Steady-state error";"Overshoot";"Absolute elevator 0-10 s";...
 "Open-loop noise band maximum";"Closed-loop noise band maximum"];
actual=[m.Bandwidth_radps;m.Crossover_radps;m.SettlingFinal_s;m.SettlingCommand_s;...
 m.SSE_pct;m.Overshoot_pct;max(abs([m.StepElevatorMin_deg m.StepElevatorMax_deg]));...
 m.LNoiseMax_dB;m.TNoiseMax_dB];
units=["rad/s";"rad/s";"s";"s";"percent";"percent";"deg";"dB";"dB"];
criteria=["2 <= value <= 10";"2 <= value <= 10";"value <= 3";"value <= 3";...
 "value < 1";"value < 20";"value <= 25";"value <= -40";"value <= -40"];
passed=[actual(1)>=2 && actual(1)<=10;actual(2)>=2 && actual(2)<=10;...
 actual(3)<=3;actual(4)<=3;actual(5)<1;actual(6)<20;actual(7)<=25;...
 actual(8)<=-40;actual(9)<=-40];
assert(all(passed));
writetable(table(names,actual,units,criteria,passed,'VariableNames',...
 {'Requirement','Result','Unit','Criterion','Passed'}),fullfile(folder,'requirements.csv'));
disp(struct2table(M)); fprintf('DESIGN_COMPLETE\n');
end
