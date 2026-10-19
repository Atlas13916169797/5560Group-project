function result = calc_linear()
% Central-difference Jacobians at the supplied Group 8 trim point.
folder = fileparts(mfilename('fullpath'));
trimFolder = fullfile(fileparts(folder),'trim');
addpath(trimFolder);
[F,x0,u0] = init();
diary(fullfile(folder,'linear_log.txt'));
guard = onCleanup(@() diary('off'));
fprintf('Group 8 linearisation | %s\n',version);
fprintf('Model: %s\n',which('FlightDataAircraft8'));
g = zeros(6,1);
f0 = rates(x0,u0,g,F);
states = ["u","v","w","p","q","r","phi","theta","psi","Xe","Ye","Ze"];
controls = ["throttle","elevator","aileron","rudder","flap"];
V0 = norm(x0(1:3));
xScale = [V0;V0;V0;ones(6,1);max(abs(x0(10:12)),1)];
uScale = ones(5,1);
relativeSteps = [1e-4,3e-5,1e-5,3e-6,1e-6];
Agrid = zeros(12,12,numel(relativeSteps));
Bgrid = zeros(12,5,numel(relativeSteps));
for k=1:numel(relativeSteps)
    hx=xScale*relativeSteps(k); hu=uScale*relativeSteps(k);
    for j=1:12
        xp=x0; xm=x0; xp(j)=xp(j)+hx(j); xm(j)=xm(j)-hx(j);
        Agrid(:,j,k)=(rates(xp,u0,g,F,f0)-rates(xm,u0,g,F,f0))/(2*hx(j));
    end
    for j=1:5
        up=u0; um=u0; up(j)=up(j)+hu(j); um(j)=um(j)-hu(j);
        Bgrid(:,j,k)=(rates(x0,up,g,F,f0)-rates(x0,um,g,F,f0))/(2*hu(j));
    end
end
% Select the middle step; compare with both larger and smaller perturbations.
selected=3; A=Agrid(:,:,selected); B=Bgrid(:,:,selected);
Arelative=zeros(numel(relativeSteps),1); Brelative=Arelative;
Amax=Arelative; Bmax=Arelative;
for k=1:numel(relativeSteps)
    Arelative(k)=norm(Agrid(:,:,k)-A,'fro')/max(norm(A,'fro'),eps);
    Brelative(k)=norm(Bgrid(:,:,k)-B,'fro')/max(norm(B,'fro'),eps);
    Amax(k)=max(abs(Agrid(:,:,k)-A),[],'all');
    Bmax(k)=max(abs(Bgrid(:,:,k)-B),[],'all');
end
convergence=table(relativeSteps',Arelative,Brelative,Amax,Bmax,...
    'VariableNames',{'RelativeStep','A_RelativeDifference','B_RelativeDifference',...
    'A_MaxElementDifference','B_MaxElementDifference'});
disp(convergence);
assert(max(Arelative)<1e-5 && max(Brelative)<1e-5,'Jacobian is not step-converged.');
writetable(convergence,fullfile(folder,'convergence.csv'));

lon=[1 3 5 8]; lat=[2 4 6 7 9]; lon5=[lon 12];
Alon=A(lon,lon); Blon=B(lon,[1 2]);
Alat=A(lat,lat); Blat=B(lat,[3 4]);
Alon5=A(lon5,lon5); Blon5=B(lon5,[1 2]);
Bcontrols=B(:,1:4);
lambdaLon=eig(Alon); lambdaLat=eig(Alat); lambdaFull=eig(A);
writeMatrix(A,states,states,fullfile(folder,'A.csv'));
writeMatrix(B,states,controls,fullfile(folder,'B.csv'));
writeMatrix(Alon,states(lon),states(lon),fullfile(folder,'A_lon.csv'));
writeMatrix(Blon,states(lon),controls([1 2]),fullfile(folder,'B_lon.csv'));
writeMatrix(Alat,states(lat),states(lat),fullfile(folder,'A_lat.csv'));
writeMatrix(Blat,states(lat),controls([3 4]),fullfile(folder,'B_lat.csv'));
writeMatrix(Alon5,states(lon5),states(lon5),fullfile(folder,'A_lon5.csv'));
writeMatrix(Blon5,states(lon5),controls([1 2]),fullfile(folder,'B_lon5.csv'));
eigenvalues=table([repmat("longitudinal",4,1);repmat("lateral",5,1);repmat("full",12,1)],...
    real([lambdaLon;lambdaLat;lambdaFull]),imag([lambdaLon;lambdaLat;lambdaFull]),...
    'VariableNames',{'Model','Real','Imaginary'});
writetable(eigenvalues,fullfile(folder,'poles.csv'));

% A local Taylor check is independent of the column-by-column differences.
% The exact +/- averaged residual should decrease quadratically as scale halves.
xDirection=xScale.*[.4;-.2;.3;.1;-.2;.15;.2;-.1;.2;0;0;-.2];
uDirection=[.2;-.3;.1;-.15;.1];
amplitudes=[1e-3;5e-4;2.5e-4];
taylorError=zeros(3,1);
for k=1:3
    dx=amplitudes(k)*xDirection; du=amplitudes(k)*uDirection;
    fp=rates(x0+dx,u0+du,g,F,f0);
    fm=rates(x0-dx,u0-du,g,F,f0);
    taylorError(k)=max(norm(fp-f0-A*dx-B*du,inf),norm(fm-f0+A*dx+B*du,inf));
end
ratios=[NaN;taylorError(1:2)./taylorError(2:3)];
taylor=table(amplitudes,taylorError,ratios,'VariableNames',{'Amplitude','MaxResidual','PreviousErrorRatio'});
writetable(taylor,fullfile(folder,'taylor.csv'));
disp(taylor);
assert(all(ratios(2:3)>3 & ratios(2:3)<5),'Local Taylor check did not show quadratic convergence.');

% Dynamic validation: small elevator/throttle steps, nonlinear deviations vs expm.
% No Control System Toolbox is needed; earth-position reference is moving.
t=(0:.05:10)';
testNames=["elevator","throttle"];
testInputs=zeros(5,2); testInputs(2,1)=deg2rad(.05); testInputs(1,2)=.001;
Cq=zeros(1,12); Cq(5)=180/pi;
Cv=zeros(1,12); Cv(1:3)=x0(1:3)'/V0;
Cvs=zeros(1,12); theta=x0(8);
Cvs(1)=sin(theta); Cvs(3)=-cos(theta);
Cvs(8)=x0(1)*cos(theta)+x0(3)*sin(theta);
Cout=[Cv;Cq;Cvs];
baseline=observables(x0');
maxError=zeros(2,3); peak=zeros(2,3);
validation=struct();
for j=1:2
    du=testInputs(:,j); step=.05;
    augmented=[A,B*du;zeros(1,13)];
    transition=expm(augmented*step);
    z=zeros(13,1); z(end)=1; Xlinear=zeros(numel(t),12);
    for k=2:numel(t), z=transition*z; Xlinear(k,:)=z(1:12)'; end
    opts=odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',.025);
    [~,Xnonlinear]=ode45(@(time,x) rates(x,u0+du,g,F),t,x0,opts);
    Ylinear=Xlinear*Cout'; Ynonlinear=observables(Xnonlinear)-baseline;
    maxError(j,:)=max(abs(Ylinear-Ynonlinear),[],1);
    peak(j,:)=max(abs(Ynonlinear),[],1);
    validation(j).Name=testNames(j); validation(j).Input=du;
    validation(j).Time=t; validation(j).Linear=Ylinear; validation(j).Nonlinear=Ynonlinear;
    history=array2table([t,Ylinear,Ynonlinear], 'VariableNames',...
        {'Time','LinearSpeed','LinearPitchRate','LinearVerticalSpeed',...
        'NonlinearSpeed','NonlinearPitchRate','NonlinearVerticalSpeed'});
    writetable(history,fullfile(folder,testNames(j)+"_check.csv"));
end
validationSummary=table(testNames',testInputs(1,:)',testInputs(2,:)',...
    maxError(:,1),maxError(:,2),maxError(:,3),peak(:,1),peak(:,2),peak(:,3),...
    'VariableNames',{'Test','ThrottleStep','ElevatorStepRad','SpeedError_mps',...
    'PitchRateError_degps','VerticalSpeedError_mps','SpeedPeak_mps',...
    'PitchRatePeak_degps','VerticalSpeedPeak_mps'});
writetable(validationSummary,fullfile(folder,'validation.csv')); disp(validationSummary);

result=struct('A',A,'B',B,'Alon',Alon,'Blon',Blon,'Alat',Alat,'Blat',Blat,...
    'Alon5',Alon5,'Blon5',Blon5,'X0',x0,'U0',u0,'F0',f0,...
    'States',states,'Controls',controls,'SelectedRelativeStep',relativeSteps(selected),...
    'MaxA_RelativeDifference',max(Arelative),'MaxB_RelativeDifference',max(Brelative),...
    'TaylorErrorRatios',ratios(2:3),'ValidationErrors',maxError,'ValidationPeaks',peak);
save(fullfile(folder,'linear.mat'),'A','B','Bcontrols','Alon','Blon','Alat','Blat',...
    'Alon5','Blon5','x0','u0','f0','F','states','controls','relativeSteps','Agrid','Bgrid',...
    'convergence','lambdaLon','lambdaLat','lambdaFull','taylor','Cout','validation','validationSummary','result');
fid=fopen(fullfile(folder,'linear.json'),'w','n','UTF-8');
assert(fid>=0,'Cannot write JSON');
fwrite(fid,jsonencode(result,'PrettyPrint',true),'char'); fclose(fid);
fid=fopen(fullfile(folder,'matrices.txt'),'w');
fprintf(fid,'Units: SI; all angular states and control angles in radians.\n');
printMatrix(fid,'A (u v w p q r phi theta psi Xe Ye Ze)',A);
printMatrix(fid,'B (throttle elevator aileron rudder flap)',B);
printMatrix(fid,'A_lon (u w q theta)',Alon); printMatrix(fid,'B_lon (throttle elevator)',Blon);
printMatrix(fid,'A_lat (v p r phi psi)',Alat); printMatrix(fid,'B_lat (aileron rudder)',Blat);
fclose(fid);
format long g; disp('A_lon'); disp(Alon); disp('B_lon'); disp(Blon);
disp('Longitudinal poles'); disp(lambdaLon); disp('Lateral poles'); disp(lambdaLat);
fprintf('All outputs saved to %s\n',folder);
end

function writeMatrix(M,rows,columns,file)
T=array2table(M,'VariableNames',cellstr(columns),'RowNames',cellstr("d_"+rows));
writetable(T,file,'WriteRowNames',true);
end
function printMatrix(fid,name,M)
fprintf(fid,'\n%s\n',name);
for row=1:size(M,1), fprintf(fid,'% .12g ',M(row,:)); fprintf(fid,'\n'); end
end
function Y=observables(X)
u=X(:,1); v=X(:,2); w=X(:,3); phi=X(:,7); theta=X(:,8);
vs=u.*sin(theta)-v.*sin(phi).*cos(theta)-w.*cos(phi).*cos(theta);
Y=[sqrt(u.^2+v.^2+w.^2),X(:,5)*180/pi,vs];
end
