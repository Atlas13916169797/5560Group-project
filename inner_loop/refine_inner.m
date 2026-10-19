function best = refine_inner()
% Refine against assumed servo lag and the independently retrimmed 180 kt model.
folder=fileparts(mfilename('fullpath'));D=load(fullfile(folder,'controller.mat'));
V=load(fullfile(folder,'validation.mat'));s=tf('s');
plants={D.Pq,D.Pq/(.05*s+1),minreal(-V.results.Models{1}.G/s,1e-7)};
base=cell(3,1);
for j=1:3
 [n,d]=tfdata(tf(plants{j}),'v');n=n(find(abs(n)>1e-12*max(abs(n)),1):end);
 base{j}=struct('n',n,'d',d);
end
times=unique([0:.01:8,8.1:.1:40,41:120]);
seed=log([35.5 1.72 .214 2.95 7.77 90]);rng(5561);
records=struct([]);bestScore=Inf;
for j=1:12
 start=seed;if j>1,start=start+.5*randn(size(seed));end
 options=optimset('Display','off','MaxIter',650,'MaxFunEvals',1400,'TolX',1e-5);
 [p,score]=fminsearch(@(v) objective(v,base,times),start,options);
 r=struct('Parameters',exp(p),'Score',score);
 if j==1,records=r;else,records(j)=r;end
 if score<bestScore,bestScore=score;selected=p;end
 fprintf('Robust refinement %d score %.5g\n',j,score);
end
v=exp(selected);K=-v(1)*(s+v(2))*(s+v(4))/(s*(s+v(3))*(s+v(5))*(s+v(6)));
best=struct('Parameters',v,'K',zpk(K),'Score',bestScore,'Records',records);
for j=1:3
 best.Metrics{j}=inner_metrics(minreal(plants{j}*(-s*K),1e-7));
end
save(fullfile(folder,'refinement.mat'),'best');disp(best.Parameters);
for j=1:3,disp(best.Metrics{j});end
end
function score=objective(v,base,t)
a=exp(v);score=1e8;
if any(~isfinite(a))||any(a<.002)||any(a>500)||a(2)<=a(3)||a(5)<=a(4)||a(6)>100,return;end
nf=a(1)*conv([1 a(2)],[1 a(4)]);df=conv(conv([1 a(3)],[1 a(5)]),[1 a(6)]);
metrics=zeros(3,6);
for j=1:3
 b=base{j};n=conv(b.n,nf);d=conv(b.d,df);nn=[zeros(1,numel(d)-numel(n)),n];cl=d+nn;
 if any(real(roots(cl))>=0),return;end
 dc=n(end)/cl(end);err=100*abs(1-dc);
 [r,p,k]=residue(n,conv(cl,[1 0]));if ~isempty(k),return;end
 y=real(r.'*exp(p*t));os=max(0,100*(max(y)/dc-1));
 outside=find(abs(y-dc)>.02*abs(dc)|abs(y-1)>.02,1,'last');ts=120;
 if isempty(outside),ts=0;elseif outside<numel(t),ts=t(outside+1);end
 noise=20*log10(abs(polyval(n,20i)/polyval(cl,20i)));
 w=logspace(-2,2,350);L=polyval(n,1i*w)./polyval(d,1i*w);T=L./(1+L);
 ix=find(abs(T)<abs(dc)*10^(-3/20),1);bw=120;
 if ~isempty(ix),bw=w(ix);end
 wcIndex=find(abs(L)<1,1);pm=0;
 if ~isempty(wcIndex),ph=unwrap(angle(L));pm=180+rad2deg(ph(wcIndex));end
 metrics(j,:)=[err,os,ts,noise,bw,pm];
end
m=metrics(1,:);a50=metrics(2,:);slow=metrics(3,:);
score=500*max(m(1)-.60,0)^2+100*max(m(2)-13,0)^2+300*max(m(3)-2.8,0)^2 ...
 +40*max(m(4)+21,0)^2+4*(m(5)-6)^2+2*max(60-m(6),0)^2 ...
 +80*max(a50(2)-19,0)^2+200*max(a50(3)-3,0)^2 ...
 +500*max(slow(1)-.95,0)^2+80*max(slow(2)-19,0)^2+200*max(slow(3)-3,0)^2 ...
 +.01*m(2)+.02*m(3)+.003*m(5);
end
