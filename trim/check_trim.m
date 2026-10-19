function result = check_trim()
% Check the supplied initial point without changing trim or controls.
[F, X0, U0, root] = init();
out = root;
diary(fullfile(out, 'trim_log.txt'));
diaryGuard = onCleanup(@() diary('off'));
fprintf('Group 8 trim check | MATLAB %s\n', version);
fprintf('Aircraft: %s\n', which('FlightDataAircraft8'));
fprintf('Aero: %s\n', which('aero5560_aero'));
gust = zeros(6,1);
[dx0, force0, moment0, iterations, iterationError] = rates(X0,U0,gust,F);
% Diagnostic tolerances, selected for this check, not assignment criteria.
names = ["u";"v";"w";"p";"q";"r";"phi";"theta";"psi";"Xe";"Ye";"Ze"];
units = ["m/s^2";"m/s^2";"m/s^2";"rad/s^2";"rad/s^2";"rad/s^2";...
    "rad/s";"rad/s";"rad/s";"m/s";"m/s";"m/s"];
tol = [1e-4*ones(3,1);1e-6*ones(6,1);NaN;1e-4;1e-4];
checked = true(12,1); checked(10) = false;
within = abs(dx0) <= tol; within(~checked) = true;
residuals = table(names, units, dx0, tol, checked, within, ...
    'VariableNames', {'State','DerivativeUnit','Derivative','Tolerance','Checked','WithinTolerance'});
writetable(residuals, fullfile(out,'trim_residuals.csv'));
disp(residuals);
fprintf('Rate iteration: %d iterations, update norm %.6g\n', iterations, iterationError);
fprintf('Xe rate %.9g m/s is expected forward motion, not a trim error.\n', dx0(10));

% No gust, no input step, no feedback. Integrate all 12 nonlinear states.
t = (0:0.1:60)';
rhs = @(time,x) rates(x,U0,gust,F);
options = odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.05);
[t, X] = ode45(rhs,t,X0,options);
% One tighter solve checks that observed drift is not solver inaccuracy.
tight = odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',0.025);
[~, Xfine] = ode45(rhs,t,X0,tight);
O = outputs(X); Ofine = outputs(Xfine); O0 = outputs(X0');
outputNames = ["Speed";"Altitude";"VerticalSpeed";"Alpha";"Theta";...
    "PitchRate";"Bank";"Heading";"RollRate";"YawRate"];
outputUnits = ["m/s";"m";"m/s";"deg";"deg";"deg/s";"deg";"deg";"deg/s";"deg/s"];
initial = O0(:); final = O(end,:)';
maxChange = max(abs(O-O0),[],1)';
solverDifference = max(abs(O-Ofine),[],1)';
summary = table(outputNames,outputUnits,initial,final,final-initial,maxChange,solverDifference,...
    'VariableNames',{'Output','Unit','Initial','Final','FinalChange','MaxChange','SolverDifference'});
writetable(summary,fullfile(out,'trim_summary.csv'));
disp(summary);
history = array2table([t,X,O], 'VariableNames', ...
    [{'Time'},cellstr("state_"+names'),cellstr("output_"+outputNames')]);
writetable(history, fullfile(out,'trim_history.csv'));

result = struct('MATLAB',version,'X0',X0,'U0',U0,'Xdot0',dx0,...
    'Force0',force0,'Moment0',moment0,'Iterations',iterations,...
    'IterationError',iterationError,'ResidualCheckPassed',all(within),...
    'Duration',t(end),'OutputNames',outputNames,'OutputUnits',outputUnits,...
    'InitialOutputs',initial,'FinalOutputs',final,'MaxChange',maxChange,...
    'SolverDifference',solverDifference);
save(fullfile(out,'trim.mat'),'F','X0','U0','dx0','force0','moment0',...
    't','X','O','Xfine','summary','residuals','result');
fid=fopen(fullfile(out,'trim.json'),'w','n','UTF-8');
assert(fid>=0,'Cannot write summary JSON.');
fwrite(fid,jsonencode(result,'PrettyPrint',true),'char'); fclose(fid);
plot_trim();
fprintf('Initial residual check passed: %d\n',result.ResidualCheckPassed);
fprintf('Results saved: %s\n',out);
end

function Y = outputs(X)
u=X(:,1); v=X(:,2); w=X(:,3);
phi=X(:,7); theta=X(:,8);
speed=sqrt(u.^2+v.^2+w.^2);
% NED earth position: altitude = -Ze; exact upward velocity from rotation.
vs=u.*sin(theta)-v.*sin(phi).*cos(theta)-w.*cos(phi).*cos(theta);
Y=[speed,-X(:,12),vs,atan2(w,u)*180/pi,theta*180/pi,...
   X(:,5)*180/pi,phi*180/pi,X(:,9)*180/pi,X(:,4)*180/pi,X(:,6)*180/pi];
end
