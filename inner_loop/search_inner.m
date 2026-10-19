function design = search_inner()
% Bode/root-locus loop-shaping search with real lead/lag poles and zeros.
folder=fileparts(mfilename('fullpath'));
D=load(fullfile(fileparts(folder),'Gqde','Gqde.mat'),'Gqde');
G=D.Gqde;
% Pull the one required integrator outside sisotool, as in Lecture 10.
P=minreal(-G/tf('s'),1e-9);
[np,dp]=tfdata(tf(P),'v'); np=trimPoly(np);
grid=logspace(-5,4,2200);
times=unique([0:.005:5,5.05:.05:30,30.2:.2:120]);
base=struct('np',np,'dp',dp,'w',grid,'t',times);
iteration1=assess([],0,base,3.0);
rows=struct([]); rng(5560,'twister');
fprintf('Searching lead/lag designs...\n');
% Iteration 2: one lag stage and a high-frequency pole.
for wc=[2 3 4 5]
 for z=[.25 .5 1 2]
  for p=[.005 .015 .04 .1]
   for f=[12 20 30]
    item=assess([z p f],1,base,wc);
    if isempty(rows), rows=item; else, rows(end+1)=item; end %#ok<AGROW>
   end
  end
 end
end
simpleRows=rows;
% Iteration 3 family: lag, lead and one high-frequency filter pole.
for j=1:1800
 wc=exp(log(2)+rand*log(3));
 z=exp(log(.2)+rand*log(15));
 p=exp(log(.003)+rand*log(60));
 zl=exp(log(.5)+rand*log(12));
 ratio=exp(log(1.5)+rand*log(6));
 f=exp(log(10)+rand*log(3));
 item=assess([z p zl zl*ratio f],2,base,wc);
 rows(end+1)=item; %#ok<AGROW>
end
T=struct2table(rows); [~,order]=sort(T.Score); seedRows=rows(order(1:12));
for j=1:numel(seedRows)
 seed=seedRows(j);
 if seed.Family~=2,continue;end
 params=log([seed.Wc seed.Zlag seed.Plag seed.Zlead seed.Plead seed.Filter]);
 objective=@(v) objectiveValue(v,base);
 options=optimset('Display','off','MaxIter',180,'MaxFunEvals',450,'TolX',1e-4);
 best=fminsearch(objective,params,options);
 item=assess(exp(best(2:end)),2,base,exp(best(1)));
 rows(end+1)=item; %#ok<AGROW>
end
T=struct2table(rows); T=sortrows(T,'Score');
simpleT=sortrows(struct2table(simpleRows),'Score');
finalT=T(T.Family==2,:);
iteration2=table2struct(simpleT(1,:)); iteration3=table2struct(finalT(1,:));
iterations=[iteration1;iteration2;iteration3];
fprintf('Selected design iterations:\n'); disp(struct2table(iterations));
controllers=cell(3,1); filters=cell(3,1); metrics=cell(3,1);
for j=1:3
 [nf,df]=filterPolys(iterations(j));
 F=tf(nf,df); K=-F/tf('s');
 filters{j}=zpk(F); controllers{j}=zpk(K);
 metrics{j}=evaluateExact(G,P,F,K);
 fprintf('Iteration %d K(s):\n',j); disp(controllers{j});
 disp(metrics{j});
end
design=struct('G',G,'P',P,'Iterations',iterations,'Controllers',{controllers},...
 'Filters',{filters},'Metrics',{metrics},'SearchTable',T);
save(fullfile(folder,'search.mat'),'design');
writetable(T,fullfile(folder,'search.csv'));
writetable(struct2table(iterations),fullfile(folder,'iterations_search.csv'));
fprintf('SEARCH_COMPLETE\n');
end

function score=objectiveValue(v,base)
 values=exp(v); wc=values(1); par=values(2:end);
 if any(~isfinite(values)) || wc<1.5 || wc>7 || any(par<.001) || any(par>100) ...
    || par(1)<=par(2) || par(4)<=par(3)
    score=1e6+sum(abs(v)); return;
 end
 r=assess(par,2,base,wc); score=r.Score;
end

function r=assess(par,family,b,wc)
r=struct('Family',family,'Wc',wc,'Gain',NaN,'Zlag',NaN,'Plag',NaN,...
 'Zlead',NaN,'Plead',NaN,'Filter',NaN,'Bandwidth',NaN,'Settling',Inf,...
 'Overshoot',Inf,'SSE',Inf,'PhaseMargin',NaN,'Noise20dB',NaN,'PeakSensitivity',NaN,'Score',1e6);
if family==0
 nf=1;df=1;
elseif family==1
 r.Zlag=par(1);r.Plag=par(2);r.Filter=par(3);
 nf=[1 par(1)]*par(3);df=conv([1 par(2)],[1 par(3)]);
else
 r.Zlag=par(1);r.Plag=par(2);r.Zlead=par(3);r.Plead=par(4);r.Filter=par(5);
 nf=conv([1 par(1)],[1 par(3)])*par(5);
 df=conv(conv([1 par(2)],[1 par(4)]),[1 par(5)]);
end
nl0=conv(b.np,nf); dl=conv(b.dp,df);
k=abs(polyval(dl,1i*wc)/polyval(nl0,1i*wc)); r.Gain=k;
nl=nl0*k; cl=addPoly(dl,nl); poles=roots(cl);
if any(real(poles)>=-1e-8),return;end
dc=nl(end)/cl(end); r.SSE=100*abs(1-dc);
jw=1i*b.w; L=polyval(nl,jw)./polyval(dl,jw); C=L./(1+L); S=1./(1+L);
ph=unwrap(angle(L))*180/pi;
r.PhaseMargin=180+interp1(log(b.w),ph,log(wc));
mag=abs(C); ix=find(mag<abs(dc)/sqrt(2),1);
if ~isempty(ix) && ix>1
 r.Bandwidth=exp(interp1(log(mag(ix-1:ix)),log(b.w(ix-1:ix)),log(abs(dc)/sqrt(2))));
end
r.Noise20dB=20*log10(abs(polyval(nl,20i)/polyval(cl,20i)));
r.PeakSensitivity=max(abs(S));
if r.SSE>2 || r.PhaseMargin<25 || isnan(r.Bandwidth),return;end
[res,pp,kk]=residue(nl,conv(cl,[1 0]));
if ~isempty(kk),return;end
y=real(res.'*exp(pp*b.t));
r.Overshoot=100*max(0,max(y)/dc-1);
outside=find(abs(y-1)>.02,1,'last');
if isempty(outside),r.Settling=0;elseif outside<numel(b.t),r.Settling=b.t(outside+1);end
outsideFinal=find(abs(y-dc)>.02*abs(dc),1,'last');
tsFinal=Inf;
if isempty(outsideFinal),tsFinal=0;elseif outsideFinal<numel(b.t),tsFinal=b.t(outsideFinal+1);end
r.Settling=max(r.Settling,tsFinal);
if ~isfinite(r.Settling),r.Score=1e5+r.SSE;return;end
penalty=200*max(0,r.SSE-0.8)^2+200*max(0,r.Settling-2.85)^2 ...
 +2*max(0,r.Overshoot-18)^2+0.2*max(0,55-r.PhaseMargin)^2 ...
 +20*max(0,r.Noise20dB+21)^2+10*max(0,r.PeakSensitivity-1.7)^2 ...
 +30*max(0,2-r.Bandwidth)^2+30*max(0,r.Bandwidth-10)^2;
r.Score=penalty+2*(r.Bandwidth-4.5)^2+.15*r.Settling+.02*r.Overshoot;
end

function [nf,df]=filterPolys(r)
if r.Family==0,nf=r.Gain;df=1;
elseif r.Family==1
 nf=r.Gain*r.Filter*[1 r.Zlag]; df=conv([1 r.Plag],[1 r.Filter]);
else
 nf=r.Gain*r.Filter*conv([1 r.Zlag],[1 r.Zlead]);
 df=conv(conv([1 r.Plag],[1 r.Plead]),[1 r.Filter]);
end
end
function m=evaluateExact(~,P,F,~)
 L=minreal(P*F,1e-8); C=minreal(feedback(L,1),1e-8); S=minreal(feedback(1,L),1e-8);
 t=(0:.002:120)'; y=squeeze(step(C,t)); dc=dcgain(C);
 info=stepinfo(y,t,dc,'SettlingTimeThreshold',.02);
 outside=find(abs(y-1)>.02,1,'last'); ts=Inf;
 if isempty(outside),ts=0;elseif outside<numel(t),ts=t(outside+1);end
 [gm,pm,wgm,wpm]=margin(L);
 w=logspace(-5,5,12000); H=squeeze(freqresp(C,w));
 enbw=trapz(w,abs(H).^2)/dc^2;
 m=struct('DC',dc,'SSE_pct',100*abs(1-dc),'Bandwidth',bandwidth(C),...
 'Rise10to90',info.RiseTime,'SettlingFinal2pct',info.SettlingTime,...
 'SettlingCommand2pct',ts,'Overshoot_pct',info.Overshoot,...
 'GainMargin_dB',20*log10(gm),'PhaseMargin_deg',pm,'GainCrossover',wpm,...
 'PhaseCrossover',wgm,'T20_dB',20*log10(abs(evalfr(C,20i))),...
 'EquivalentNoiseBandwidth_radps',enbw,'PeakSensitivity',norm(S,inf));
end
function p=addPoly(a,b)
n=max(numel(a),numel(b));p=[zeros(1,n-numel(a)),a]+[zeros(1,n-numel(b)),b];
end
function p=trimPoly(p)
first=find(abs(p)>1e-12*max(abs(p)),1);p=p(first:end);
end
