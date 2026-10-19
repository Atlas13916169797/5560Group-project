function results = validate_inner()
% Validate off-nominal and nonlinear responses.
folder=fileparts(mfilename('fullpath')); root=fileparts(folder);
addpath(fullfile(root,'trim'));
D=load(fullfile(folder,'controller.mat')); N=load(fullfile(root,'A_B_Matrix','linear.mat'));
[F,x0,u0]=init(); s=tf('s'); C=[0 0 1 0];
G=D.Gq; K=D.Kq; Fq=D.Fq;
[ak,bk,ck,dk]=ssdata(ss(K));
Aaug=[N.Alon-N.Blon(:,2)*dk*C,N.Blon(:,2)*ck;-bk*C,ak];
internalPoles=eig(Aaug);
assert(sum(abs(internalPoles)<1e-7)==1,'Expected one neutral kinematic mode.');
assert(all(real(internalPoles(abs(internalPoles)>=1e-7))<0));
writetable(table(real(internalPoles),imag(internalPoles),...
 'VariableNames',{'Real','Imaginary'}),fullfile(folder,'internal_poles.csv'));

% Fixed controller, independent +/-20 percent pitch stiffness, damping and gain.
uncertainty=struct([]); k=0;
for stiffness=[.8 1 1.2]
 for damping=[.8 1 1.2]
  for gain=[.8 1 1.2]
   A=N.Alon; A(3,2)=A(3,2)*stiffness; A(3,3)=A(3,3)*damping;
   Gi=zpk(ss(A,N.Blon(:,2)*gain,C,0)); Pi=minreal(-Gi/s,1e-7);
   m=inner_metrics(minreal(Pi*Fq,1e-7));
   m.StiffnessFactor=stiffness;m.DampingFactor=damping;m.ControlFactor=gain;
   k=k+1;if k==1,uncertainty=m;else,uncertainty(k)=m;end
  end
 end
end
writetable(struct2table(uncertainty),fullfile(folder,'uncertainty.csv'));

% Assumed actuator time constants.
actuators=struct([]); k=0;
for tau=[0 .025 .05 .1]
 L=D.Lq/(tau*s+1);m=inner_metrics(L);m.AssumedTimeConstant_s=tau;
 k=k+1;if k==1,actuators=m;else,actuators(k)=m;end
end
writetable(struct2table(actuators),fullfile(folder,'actuator.csv'));
% Pure-delay phase loss at nominal crossover.
m0=D.design.Metrics(3); delays=[0 .01 .02 .04]';
delayPM=m0.PhaseMargin_deg-rad2deg(delays*m0.Crossover_radps);
writetable(table(delays,delayPM,'VariableNames',{'AssumedDelay_s','PhaseMargin_deg'}),...
 fullfile(folder,'delay.csv'));

% Gain-only comparison at 2--10 rad/s actual open-loop crossover.
trade=struct([]); n=0;
for wc=2:10
 scale=1/abs(evalfr(D.Lq,1i*wc)); m=inner_metrics(D.Lq*scale);
 m.TargetCrossover_radps=wc;m.GainMultiplier=scale;
 U=minreal(feedback(K*scale,G),1e-8);t=(0:.005:10)';de=rad2deg(lsim(U,deg2rad(4)*ones(size(t)),t)+u0(2));
 m.PeakAbsElevator_deg=max(abs(de));m.PeakElevatorRate_degps=max(abs(gradient(de,t)));
 n=n+1;if n==1,trade=m;else,trade(n)=m;end
end
writetable(struct2table(trade),fullfile(folder,'frequency_trade.csv'));

% Retrim and linearize at each airspeed.
speeds=[180 200 210]; envelope=struct([]); models=cell(3,1);
for j=1:numel(speeds)
 V=speeds(j)*.514444444444444;
 if speeds(j)==200
  xt=x0;ut=u0;
 else
  z0=[x0(8)/.02;u0(1)*(V/norm(x0(1:3)))^3;u0(2)/.05];
  z=fminsearch(@(z) trimCost(z,V,F,x0,u0),z0,...
    optimset('Display','off','TolX',1e-11,'TolFun',1e-18,'MaxIter',1500,'MaxFunEvals',3000));
  [xt,ut]=trimState(z,V,x0,u0);
 end
 residual=rates(xt,ut,zeros(6,1),F);
 valid=max(abs(residual([1:9 11:12])))<1e-6 && ut(1)>0 && ut(1)<1 && abs(ut(2))<deg2rad(25);
 [A,B]=localLinear(xt,ut,F); Gi=zpk(ss(A,B,C,0)); Pi=minreal(-Gi/s,1e-7);
 m=inner_metrics(minreal(Pi*Fq,1e-7));m.Speed_knots=speeds(j);
 m.Throttle=ut(1);m.TrimElevator_deg=rad2deg(ut(2));m.TrimAlpha_deg=rad2deg(xt(8));
 m.TrimMaxResidual=max(abs(residual([1:9 11:12])));m.ValidTrim=valid;
 if j==1,envelope=m;else,envelope(j)=m;end
 models{j}=struct('A',A,'B',B,'x0',xt,'u0',ut,'G',Gi);
end
writetable(struct2table(envelope),fullfile(folder,'speed_check.csv'));

% Discrete implementation check: ZOH physical plant and Tustin compensator.
Ts=.01; Gd=c2d(ss(N.Alon,N.Blon(:,2),C,0),Ts,'zoh'); Kd=c2d(ss(K),Ts,'tustin');
% Cancel the known z=1 plant zero/controller pole in the tracking channel.
Ld=minreal(zpk(Gd)*zpk(Kd),1e-7);
Td=minreal(feedback(Ld,1),1e-7);pd=pole(Td);
assert(isstable(Td),'Minimal digital tracking must be asymptotically stable.');
% Separate the neutral pole at z=1.
neutral=abs(pd-1)<1e-6; dynamicPoles=pd(~neutral);
digital=struct('SampleTime_s',Ts,'NeutralPoleCount',sum(neutral),...
 'DynamicMaxPoleMagnitude',max(abs(dynamicPoles)),...
 'MinimalTrackingBandwidth_radps',bandwidth(Td),'Kz',zpk(Kd));
assert(all(abs(dynamicPoles)<1));
[ad,bd]=ssdata(Gd);[acd,bcd,ccd,dcd]=ssdata(Kd);
pdInternal=eig([ad-bd*dcd*C,bd*ccd;-bcd*C,acd]);
neutralInternal=abs(pdInternal-1)<1e-6;
digital.InternalNeutralPoleCount=sum(neutralInternal);
digital.InternalDynamicMaxPoleMagnitude=max(abs(pdInternal(~neutralInternal)));
assert(sum(neutralInternal)==1 && digital.InternalDynamicMaxPoleMagnitude<1);
td=(0:Ts:10)';ud=deg2rad(4)*ones(size(td));
qd=lsim(Td,ud,td);qc=lsim(D.Tq,ud,td);
digital.MaxTrackingDifference_degps=rad2deg(max(abs(qd-qc)));
writetable(table(td,rad2deg(qc),rad2deg(qd),...
 'VariableNames',{'Time_s','ContinuousPitchRate_degps','DiscretePitchRate_degps'}),fullfile(folder,'digital.csv'));

% Nonlinear step and finite-pulse tests.
[af,bf,cf,df]=ssdata(ss(Fq)); controller=struct('A',af,'B',bf,'C',cf,'D',df);
nonlinear=struct(); scenarios={'step','pulse','limited'}; stats=struct([]);
for j=1:3
 name=scenarios{j}; duration=5;if j>1,duration=10;end
 amp=deg2rad(4);tau=0;rate=Inf;
 if j==3,tau=.05;rate=deg2rad(60);end
 trial=simulate(name,duration,amp,tau,rate,controller,F,x0,u0,D);
 nonlinear.(name)=trial;
 if j==1,stats=trial.Summary;else,stats(j)=trial.Summary;end
 writetable(trial.History,fullfile(folder,[name '_nonlinear.csv']));
end
writetable(struct2table(stats),fullfile(folder,'nonlinear_summary.csv'));

% Finite-record sensor-noise test: white samples passed through a 20 rad/s LPF.

rng(5560);tn=(0:.01:10)';white=randn(size(tn));
n=lsim(20/(s+20),white,tn);n=n*deg2rad(.02)/std(n(tn>1));
qn=lsim(-D.Tq,n,tn);den=lsim(-D.Uq,n,tn);
noise=struct('AssumedSensorSigma_degps',.02,'NoiseFilterPole_radps',20,...
 'Duration_s',10,'PitchRateRMS_degps',rad2deg(rms(qn(tn>1))),...
 'ElevatorRMS_deg',rad2deg(rms(den(tn>1))),'ElevatorPeak_deg',rad2deg(max(abs(den))));
writetable(table(tn,rad2deg(n),rad2deg(qn),rad2deg(den),...
 'VariableNames',{'Time_s','SensorNoise_degps','PitchRate_degps','ElevatorIncrement_deg'}),fullfile(folder,'noise.csv'));

[rho,~]=aero5560_atmos(-x0(12));sp=pole(G);fast=sp(abs(imag(sp))>1 & imag(sp)>0);
flight=struct('Speed_mps',norm(x0(1:3)),'Speed_knots',norm(x0(1:3))/.514444444444444,...
 'Altitude_m',-x0(12),'Density_kgpm3',rho,'DynamicPressure_Pa',.5*rho*norm(x0(1:3))^2,...
 'ShortPeriodNaturalFrequency_radps',abs(fast),'ShortPeriodDamping',-real(fast)/abs(fast),...
 'ReducedFrequency',.85*m0.Bandwidth_radps/(2*norm(x0(1:3))),...
 'NominalDelayMarginEstimate_s',deg2rad(m0.PhaseMargin_deg)/m0.Crossover_radps);
results=struct('Flight',flight,'InternalPoles',internalPoles,'Uncertainty',uncertainty,...
 'Actuators',actuators,'Envelope',envelope,'Models',{models},'Trade',trade,...
 'Digital',digital,'Nonlinear',nonlinear,'Noise',noise);
save(fullfile(folder,'validation.mat'),'results');
% LTI objects are kept in MAT only; JSON contains numeric engineering summaries.
summary=rmfield(results,{'Models','Nonlinear','Digital','InternalPoles'});
summary.Digital=rmfield(digital,'Kz');summary.Metrics=D.design.Metrics;
fid=fopen(fullfile(folder,'results.json'),'w','n','UTF-8');
fwrite(fid,jsonencode(summary,'PrettyPrint',true),'char');fclose(fid);
disp(flight);disp(struct2table(envelope));disp(struct2table(stats));disp(noise);
fprintf('VALIDATION_COMPLETE\n');
end

function cost=trimCost(z,V,F,x0,u0)
[x,u]=trimState(z,V,x0,u0);r=rates(x,u,zeros(6,1),F);
cost=sum((r([1 3 5])./[1;1;.1]).^2);
end
function [x,u]=trimState(z,V,x0,u0)
a=z(1)*.02;x=x0;x(1)=V*cos(a);x(3)=V*sin(a);x(8)=a;
u=u0;u(1)=z(2);u(2)=z(3)*.05;
end
function [A,B]=localLinear(x,u,F)
ix=[1 3 5 8];A=zeros(4);g=zeros(6,1);
for j=1:4
 h=1e-5;if j<3,h=h*norm(x(1:3));end
 xp=x;xm=x;xp(ix(j))=xp(ix(j))+h;xm(ix(j))=xm(ix(j))-h;
 fp=rates(xp,u,g,F);fm=rates(xm,u,g,F);A(:,j)=(fp(ix)-fm(ix))/(2*h);
end
up=u;um=u;up(2)=up(2)+1e-5;um(2)=um(2)-1e-5;
fp=rates(x,up,g,F);fm=rates(x,um,g,F);B=(fp(ix)-fm(ix))/(2e-5);
end
function trial=simulate(name,duration,amp,tau,rate,Fc,F,x0,u0,D)
nf=size(Fc.A,1);z0=[x0;zeros(nf+2,1)];t=(0:.01:duration)';
isPulse=~strcmp(name,'step');cut=2;
opts=odeset('RelTol',2e-8,'AbsTol',1e-10,'MaxStep',.01);
rhs=@(tt,z,cmd) closedRates(z,cmd,tau,rate,Fc,F,u0,nf);
if isPulse
 [t1,z1]=ode45(@(tt,z)rhs(tt,z,amp),t(t<=cut),z0,opts);
 [t2,z2]=ode45(@(tt,z)rhs(tt,z,0),t(t>=cut),z1(end,:)',opts);
 z=[z1;z2(2:end,:)];t=[t1;t2(2:end)];
else
 [t,z]=ode45(@(tt,z)rhs(tt,z,amp),t,z0,opts);
end
qc=amp*ones(size(t));if isPulse,qc(t>=cut)=0;end
de=u0(2)+max(deg2rad(-25)-u0(2),min(deg2rad(25)-u0(2),z(:,12+nf+1)));
if tau>0,de=u0(2)+z(:,12+nf+2);end
q=z(:,5);v=sqrt(sum(z(:,1:3).^2,2));theta=z(:,8);alpha=atan2(z(:,3),z(:,1));
linearq=lsim(D.Tq,qc,t);linearde=lsim(D.Uq,qc,t)+u0(2);
% Body-normal load factor from acceleration after removing gravity/rotation.
loadFactor=zeros(size(t));
for k=1:numel(t)
 x=z(k,1:12)';u=u0;u(2)=de(k);dx=rates(x,u,zeros(6,1),F);
 loadFactor(k)=-(dx(3)-x(5)*x(1)+x(4)*x(2)-9.81*cos(x(7))*cos(x(8)))/9.81;
end
H=table(t,rad2deg(qc),rad2deg(q),rad2deg(linearq),rad2deg(de),rad2deg(linearde),...
 rad2deg(theta),rad2deg(alpha),v,-z(:,12),loadFactor,...
 'VariableNames',{'Time_s','Command_degps','PitchRate_degps','LinearPitchRate_degps',...
 'Elevator_deg','LinearElevator_deg','Theta_deg','Alpha_deg','Speed_mps','Altitude_m','BodyNormalLoad_g'});
first=t<2;maxQ=max(q(first));ratePeak=max(abs(gradient(rad2deg(de),t)));
summary=struct('Scenario',string(name),'Duration_s',duration,'AssumedServoTau_s',tau,...
 'AssumedRateLimit_degps',rad2deg(rate),'PeakPitchRate_degps',rad2deg(maxQ),...
 'PeakAbsElevator_deg',max(abs(rad2deg(de))),'PeakElevatorRate_degps',ratePeak,...
 'MaxLinearQDifference_degps',max(abs(rad2deg(q-linearq))),...
 'MaxAbsTheta_deg',max(abs(rad2deg(theta))),'SpeedMin_mps',min(v),'SpeedMax_mps',max(v),...
 'LoadMin_g',min(loadFactor),'LoadMax_g',max(loadFactor),'FinalPitchRate_degps',rad2deg(q(end)));
trial=struct('History',H,'Summary',summary);
end
function dz=closedRates(z,qc,tau,rate,Fc,F,u0,nf)
x=z(1:12);xf=z(13:12+nf);xi=z(13+nf);xa=z(14+nf);
e=qc-x(5);yd=Fc.C*xf+Fc.D*e;dxi=-yd;
lo=deg2rad(-25)-u0(2);hi=deg2rad(25)-u0(2);cmd=max(lo,min(hi,xi));
% Conditional integration at the position limit prevents further wind-up.
if (xi>=hi && dxi>0)||(xi<=lo && dxi<0),dxi=0;end
if tau>0
 actual=xa;dxa=max(-rate,min(rate,(cmd-xa)/tau));
else
 actual=cmd;dxa=0;
end
u=u0;u(2)=u0(2)+actual;
dz=[rates(x,u,zeros(6,1),F);Fc.A*xf+Fc.B*e;dxi;dxa];
end
