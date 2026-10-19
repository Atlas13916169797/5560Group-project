function result = extract_Gqde()
folder=fileparts(mfilename('fullpath'));
D=load(fullfile(fileparts(folder),'A_B_Matrix','linear.mat'),'Alon','Blon','x0','u0');
diary(fullfile(folder,'Gqde_log.txt'));
guard=onCleanup(@() diary('off'));
assert(license('test','Control_Toolbox'),'Control System Toolbox is required.');
Aq=D.Alon; Bq=D.Blon(:,2); Cq=[0 0 1 0]; Dq=0;
sys_q=ss(Aq,Bq,Cq,Dq);
sys_q.StateName={'u','w','q','theta'};
sys_q.InputName={'elevator'}; sys_q.OutputName={'pitch_rate'};
Gqde_raw=tf(sys_q);
[zeros_raw,poles_q,gain_q]=zpkdata(zpk(Gqde_raw),'v');
zeros_q=zeros_raw;
% q = d(theta)/dt guarantees a zero at s=0 for this nonsingular A.
% Only snap roundoff at the origin, with independent structural/DC checks.
assert(norm(Aq(4,:)-Cq,inf)<1e-12 && Bq(4)==0 && rcond(Aq)>eps);
assert(abs(dcgain(sys_q))<1e-10);
[nearest,index]=min(abs(zeros_q));
assert(nearest<1e-9,'Expected zero at origin was not found.');
zeros_q(index)=0;
Gqde=zpk(zeros_q,poles_q,gain_q);
Gqde.InputName={'elevator'}; Gqde.OutputName={'pitch_rate'};
[num,den]=tfdata(tf(Gqde),'v');
% Independent numerator from the Cayley-Hamilton / Markov recurrence.
denDirect=poly(Aq); Q=eye(4); numDirect=zeros(1,5);
for k=1:4
    numDirect(k+1)=Cq*Q*Bq;
    Q=Aq*Q+denDirect(k+1)*eye(4);
end
w=logspace(-4,4,501)';
Hss=complex(zeros(size(w))); Htf=Hss; Hdirect=Hss;
for k=1:numel(w)
    s=1i*w(k);
    Hss(k)=Cq*((s*eye(4)-Aq)\Bq);
    Htf(k)=evalfr(Gqde,s);
    Hdirect(k)=polyval(numDirect,s)/polyval(denDirect,s);
end
maxRelativeError=max(abs(Htf-Hss)./max(abs(Hss),1e-12));
directRelativeError=max(abs(Hdirect-Hss)./max(abs(Hss),1e-12));
assert(maxRelativeError<1e-8 && directRelativeError<1e-8,'Transfer-function check failed.');
checks=table(w,real(Hss),imag(Hss),real(Htf),imag(Htf),...
    abs(Htf-Hss)./max(abs(Hss),1e-12),...
    'VariableNames',{'Frequency_radps','StateSpaceReal','StateSpaceImag',...
    'TransferReal','TransferImag','RelativeError'});
writetable(checks,fullfile(folder,'Gqde_check.csv'));
rootsTable=table([repmat("zero",numel(zeros_q),1);repmat("pole",numel(poles_q),1)],...
    real([zeros_q;poles_q]),imag([zeros_q;poles_q]),...
    'VariableNames',{'Type','Real','Imaginary'});
writetable(rootsTable,fullfile(folder,'Gqde_roots.csv'));
units='q in rad/s; elevator increment in rad; s in 1/s';
result=struct('Numerator',num,'Denominator',den,'ZerosReal',real(zeros_q),...
    'ZerosImag',imag(zeros_q),'PolesReal',real(poles_q),'PolesImag',imag(poles_q),...
    'Gain',gain_q,'RawZerosReal',real(zeros_raw),'RawZerosImag',imag(zeros_raw),...
    'MaxRelativeError',maxRelativeError,'DirectRelativeError',directRelativeError,...
    'DCGain',dcgain(Gqde),'Units',units,'A',Aq,'B',Bq,'C',Cq,'D',Dq);
save(fullfile(folder,'Gqde.mat'),'Gqde','Gqde_raw','sys_q','Aq','Bq','Cq','Dq',...
    'zeros_q','poles_q','gain_q','num','den','numDirect','denDirect','result','units');
fid=fopen(fullfile(folder,'Gqde.json'),'w','n','UTF-8');
assert(fid>=0,'Cannot write JSON'); fwrite(fid,jsonencode(result,'PrettyPrint',true),'char'); fclose(fid);
realZeroFactors=sort(-zeros_q(abs(zeros_q)>0),'descend');
upperPoles=poles_q(imag(poles_q)>0);
assert(numel(realZeroFactors)==2 && numel(upperPoles)==2,'Unexpected plant factors.');
displayText=sprintf(['Gqde(s) = %.12g * s * (s + %.12g) * (s + %.12g) /\n' ...
    '  ((s^2 + %.12g*s + %.12g) * (s^2 + %.12g*s + %.12g))\n'],...
    gain_q,realZeroFactors(1),realZeroFactors(2),...
    -2*real(upperPoles(1)),abs(upperPoles(1))^2,...
    -2*real(upperPoles(2)),abs(upperPoles(2))^2);
fid=fopen(fullfile(folder,'Gqde.txt'),'w');
fprintf(fid,'Group 8, 200 knots / 300 ft, longitudinal four-state model\n%s\n',units);
fprintf(fid,'%s\n',displayText);
fprintf(fid,'Numerator (descending powers):\n'); fprintf(fid,'%.15g ',num); fprintf(fid,'\n');
fprintf(fid,'Denominator (descending powers):\n'); fprintf(fid,'%.15g ',den); fprintf(fid,'\n');
fprintf(fid,'ZPK gain: %.15g\n',gain_q);
fprintf(fid,'Max frequency-domain relative error: %.8g\n',maxRelativeError);
fclose(fid);
disp(Gqde); disp(rootsTable); disp(result);
end
