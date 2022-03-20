%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script to evaluate ACC control strategy for car-following scenario and
% reference tracking scenario
% Author: Sai Krishna Chada
% Date: 28.11.2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;   % removes all variables from the current workspace
clc;         % clear command window
%% ICE Vehicle Parameters
A_f = 2.27; %cross-sectional area
C_a = 0.29; %coefficient of aerodynamic resistance
c_r = 0.011; %coefficient of rolling resistance
g = 9.81; %acceleration due to gravity 
m_eq = 1800; %equivalent mass in kg (vehicle mass + mass of rotating components)
Rho = 1.225; %density of air
T_s = 0.2;  % sampling time
N = 20; % prediction horizon

%% Use the below code if Sampling time is 0.2
load('Preceding_Speed'); % preceding vehicle velocity
load('Vmax'); % Road maximum speed limit
load('Vmin'); % Road minimum speed limit
load('Vset'); % Driver Set speed
load('Elevation') ; % Road elevation
V_p_speed= V_p;
Vmax= V_max;
Vmin = V_min;
theta = Elevation; %%Is this theta in rad or deg?
clear Elevation V_min V_max V_p

% system dynamics
V_h_speed(1) = V_p_speed(1); % initial host vehicle velocity, Unit:[m/s]
h_start = 1;  % starting time of host vehicle, Unit:[s]
p_start = 5; % starting time of preceding vehicle, Unit:[s]
h_dist(h_start) = 0;  % starting position of host vehicle, Unit:[m]
p_dist(1) = V_p_speed(p_start)*p_start;  % starting position of preceding vehicle, Unit:[m]
rel_dist(1) =  p_dist(1)- h_dist(h_start); %starting relative distance, Unit:[m]
V_p_speed_new = V_p_speed(p_start:end); %shifted V_p_speed array
v_preceding(1) = V_p_speed_new(1);
Simulation_length = length(V_p_speed_new)-N; % simulation length, Unit:[s]
a_Prev(1) = 0; %Previous time step acceleration in initial step
tic         % start stopwatch timer

for k=1:Simulation_length    
    % percentage of processing finished
    if rem(k, round(Simulation_length/10)) == 0
        text = k/round(Simulation_length/10);
        disp([num2str(text*10),'% finished']);  
    end
    
    horizon = k:1:k+N-1;
    theta_hor = theta(horizon); % Units[Deg]
    Vmax_hor = Vmax(horizon)./3.6;% Units[m/s]
    Vmin_hor = Vmin(horizon)./3.6; % Units[m/s]
    V_p = V_p_speed_new(k)*ones(N+1,1);% Units[m/s]
    V_set = Vset(k)./3.6; % Units[m/s] 
        
    [a,epsilon_1,v_predict,d_predict,exitflag,mode]= SetSpeed_Combined ...
        (k,N,V_p,theta_hor,T_s,V_h_speed(k),rel_dist(k),Vmax_hor,Vmin_hor,V_set,a_Prev(k));
    control_mode(k) = mode;
    a_Prev(k+1) = a;
    v_mpc(k)=v_predict(1);
    d_mpc(k)=d_predict(1);
    v_max=Vmax_hor(1);
    % Apply acceleration and update states
    v_preceding(k+1) = V_p_speed_new(k+1);
    V_h_speed(k+1) = V_h_speed(k) + T_s*a;
    %   Distance between host and preceding vehicles
    h_dist(h_start+k) = h_dist(h_start+k-1)+T_s*(V_h_speed(k)+V_h_speed(k+1))/2;
    h_dist(h_start+k) = round(h_dist(h_start+k)*10)/10;
      
    p_dist(k+1) = p_dist(k)+T_s*(v_preceding(k)+v_preceding(k+1))/2;
    p_dist(k+1) = round(p_dist(k+1)*10)/10;
    
    rel_dist(k+1) = p_dist(k+1)- h_dist(h_start+k);
    
end

% Plots
figure;
ax(1)=subplot(3,1,1);
plot(rel_dist);
grid on;
hold on;
xlabel('Time [s]','FontSize',16,'FontName','Times');
set(gca,'fontsize',16);
set(gca,'fontname','times');
title('Relative distance plot','FontSize',16,'FontName','Times');

ax(2)=subplot(3,1,2);
plot(h_dist);
grid on;
hold on;
xlabel('Time [s]','FontSize',16,'FontName','Times');
set(gca,'fontsize',16);
set(gca,'fontname','times');
title('Distance traveled by host car [m]','FontSize',16,'FontName','Times');

ax(3)=subplot(3,1,3);
plot(p_dist);
grid on;
hold on;
xlabel('Time [s]','FontSize',16,'FontName','Times');
set(gca,'fontsize',16);
set(gca,'fontname','times');
title('Distance traveled by preceeding car [m]','FontSize',16,'FontName','Times');
linkaxes(ax,'x');

figure;
ax(1) = subplot(2,1,1);
grid on;
hold on;
plot(V_h_speed);
plot(Vmax/3.6);
plot(Vset/3.6);
plot(v_preceding);
xlabel('Time [s]','FontSize',16,'FontName','Times');
title('Velocity Plot','FontSize',16,'FontName','Times');
legend(ax(1),{'Host vehicle', 'Max speed', 'Set speed', 'Preceeding vehicle'});
set(gca,'fontsize',16);
set(gca,'fontname','times');
set(gcf,'color','w');

ax(2) = subplot(2,1,2);
grid on;
hold on;
plot(a_Prev);
xlabel('Time [s]','FontSize',16,'FontName','Times');
title('Acceleration Plot','FontSize',16,'FontName','Times');
set(gca,'fontsize',16);
set(gca,'fontname','times');
set(gcf,'color','w');
linkaxes(ax,'x');

figure;
grid on;
plot(control_mode);
xlabel('Time [s]','FontSize',16,'FontName','Times');
title('Control Mode (0 indicates distance control and 1 indicates speed control)','FontSize',16,'FontName','Times');
set(gca,'fontsize',16);
set(gca,'fontname','times');
set(gcf,'color','w');






