%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Aircraft simulation example
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

%% Simulation Paramter
DT = 0.01;          % Integration time interval
T0 = DT;            % Simulation start time
TF = 15;            % Termination time for simulation

% Aircraft specifcation
[ FlightData ] = aero5560_LoadFlightData();

% Initial condition (usually trimmed values at equilibrium condition)
load ICs_FC1.mat

%% Trim condition
% Extract usesful trim values
V_trim  = sqrt(X0(1)^2+X0(2)^2+X0(3)^2);     % (m/s)
alt0    = -X0(12); % (m)
alt0ft  = alt0/0.3048;        % (ft)
alpha_0 = X0(3)/V_trim; % (rad)
beta_0  = 0/57.3; % (rad)
gamma   = 0/57.3; % (rad)
phi_0   = 0/57.3; % (rad)
dp      = U0(1);     % (fraction of travel)
de      = U0(2); % (rad)
da      = 0/57.3; % (rad)
dr      = 0/57.3; % (rad)
df      = U0(5); % (rad)

disp(' ')
disp(' Trimmed State and Controls')
long_trim = ([' V = ',num2str(V_trim),' m/s, ',' Alpha = ',num2str(X0(3)/V_trim*57.3),' deg, ',' Theta = ',num2str(X0(8)*57.3),' deg. ']);
lat_trim  = ([' Beta = ',num2str(X0(2)/V_trim*57.3),' deg. ',' Phi = ',num2str(X0(7)*57.3),' deg, ',' Psi = ',num2str(X0(9)*57.3),' deg. ']);
Cont_trim = ([' Throttle = ',num2str(U0(1)),' (0-1) ',' Elevator = ',num2str(U0(2)*57.3),' deg, ',' Aileron = ',num2str(U0(3)*57.3),' deg, ',' Rudder = ',num2str(U0(4)*57.3),' deg. ',' Flap = ',num2str(U0(5)*57.3),' deg. ']);
disp(' ')
disp(long_trim)
disp(' ')
disp(lat_trim)
disp(' ')
disp(Cont_trim)
disp(' ')

%% Simulation initialisation
% expand vectors to speed up simulation
n_pts = round(TF/DT+1);
X = ones(12,n_pts).*X0;
U = ones(5,n_pts).*U0;
V = ones(1,n_pts).*V_trim;
T = ones(1,n_pts).*DT;

%% Main simulation loop
for i = 2:n_pts % Start Simulation loop

    % Current Time
    T(i) = i*DT;

    % Control Input
    U(:,i-1) = U0;

    % Gust Input ([u,v,w,p,q,r]^T gust components)
    Xg = [0;0;0;0;0;0];

    % disturbance (leave blank if no disturbace)
    % 1 degree rudder impulse at T = 1s
    if T(i)>=1 && T(i)<=2
        U(4,i-1) = U(4,1)+deg2rad(1);
    end

    % Integration
    [X_out] = aero5560_euler(DT,X(:,i-1),Xg,U(:,i-1), FlightData);
    X(:,i) = X_out;
    V(i) = sqrt(X(1,i)^2+X(2,i)^2+X(3,i)^2);
end     % End Simulation loop

U(:,i) = U(:,i-1); % Save final control inputs

%% Animation initialisation
% Note!!! Animation will slow down your script a lot!
% change the falg to 0 if you don't want to see the animation every time
animation_flag = 1;

if animation_flag == 1
    % time interval for animation, bigger interval = faster script = less smooth
    animation_interval = 0.1; %(s)

    % initialise video writer
    video_saver = VideoWriter('visualisation.avi');
    video_saver.FrameRate = 1/DT*animation_interval;
    open(video_saver);

    animation_handle = figure();

    animation_handle.Position = [100 100 1820 980];
        subplot(2,1,1); % aircraft attitude visualisation
            equilibrium_handle = uav(0,0,0,'roll',rad2deg(X(7,1)),'pitch',rad2deg(X(8,1)),'yaw',rad2deg(X(9,1)),'color','none');
            hold on;
            visual_handle = uav(0,0,0,'roll',rad2deg(X(7,1)),'pitch',rad2deg(X(8,1)),'yaw',rad2deg(X(9,1)),'color','r');
            axis equal; axis tight;
            xlabel('x');ylabel('y');zlabel('z');
            % choose a view (only one)
            % veiw 1: Isometric
            view_vector = [140 30];
            % view 2: Topdown x-y
            % view_vector =[0 90];
            % view 3: Side x-z
            % view_vector = [180 0];
            % view 4: Nose y-z
%             view_vector = [90 0];

            view(view_vector);
            grid on;

        subplot(2,1,2); % aircraft attitude time series plot
            lineplot_handle = plot(T(1), rad2deg(X(2,1)./V(1)),'or','linewidth',2);
            axis([0, max(T), rad2deg(min(X(2,:)./V)), rad2deg(max(X(2,:)./V))]);
            legend('beta (deg)','AutoUpdate','off');
            xlabel('Time (s)');
            ylabel('Response');
            title('Response to unit rudder impulse');
            grid on;
end

%% Animation loop
if animation_flag == 1
    figure(animation_handle);
    for i = 2:n_pts
        if mod(T(i),animation_interval)==0
            subplot(2,1,1);
            hold off;
            equilibrium_handle = uav(0,0,0,'roll',rad2deg(X(7,1)),'pitch',rad2deg(X(8,1)),'yaw',rad2deg(X(9,1)),'color','none');
            hold on;
            visual_handle = uav(0,0,0,'roll',rad2deg(X(7,i)),'pitch',rad2deg(X(8,i)),'yaw',rad2deg(X(9,i)),'color','r');

            subplot(2,1,2);
            set(lineplot_handle, 'XData', T(1,1:i), 'YData', rad2deg(asin(X(2,1:i)./V(1,1:i))));

            current_frame = getframe(gcf);
            writeVideo(video_saver, current_frame);
        end
    end
    close(video_saver);
end


%% Plot simulation results
figure(2);
hold on; grid on;
% plot beta
plot(T,rad2deg(asin(X(2,:)./V)),'-g','linewidth',2);
% plot r yaw rate
plot(T,rad2deg(X(6,:)),'-b','linewidth',2);
legend('beta (deg)','r (deg/s)');
xlabel('Time (s)');
ylabel('Response');
title('Response to unit rad rudder impulse')
axis tight

