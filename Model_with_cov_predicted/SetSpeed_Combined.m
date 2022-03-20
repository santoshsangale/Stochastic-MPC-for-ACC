%% Script to obtain state space model for speed control and distance control

function [a,epsilon_1,v_predict,d_predict,exitflag,mode] = SetSpeed_Combined...
    (k,N,V_p,theta,T_s,V_h_speed,rel_dist,Vmax,Vmin,V_set,a_Prev)

% ICE Vehicle Parameters
A_f = 2.27; %cross-sectional area
C_a = 0.29; %coefficient of aerodynamic resistance
c_r = 0.011; %coefficient of rolling resistance
g = 9.81; %acceleration due to gravity 
m_eq = 1800; %equivalent mass in kg (vehicle mass + mass of rotating components)
Rho = 1.225; %density of air in kg/m3
F_hydr_max = -14000; %Maximum braking force

%State vector
x_k = [V_h_speed; rel_dist];

num_state = 2;     % number of states
num_control = 2;   % number of control inputs

% component of aerodynamic resistance force
p0 = 0.5*Rho*A_f*C_a;

% Aerodynamic force parameters
p1 = 47.2222; %obtained through approximation (refer Lin's paper)
p2 = -504.3596; %obtained through approximation (refer Lin's paper)
 
% Approximation of F,V curve
p3 = -144.4; %obtained through approximation (refer Lin's paper)
p4 = 6712; %obtained through approximation (refer Lin's paper)

a_min = -3.5;       % Maximum deceleration
a_max = 2;          % Maximum acceleration    
delta_amax = 0.5;   % Maximum change in acceleration
sensor_range = 120; % in [m]
%% Switching between distance and speed control

if rel_dist>=sensor_range  % Condition to switch to speed control
    mode = 1;     % Indication of speed control mode
    q_1 = 10000;    % penalty for acceleration
    q_2 = 0;      % penatly for epsilon_1
    q_3 = 1000;   % penalty for deviation from set speed tracking
    % Constraints Parameters
    h_min = 0;
    h_max = 0;
    d_min = 0; % Minimum distance at standstill
    
        
    % A matrix
    A=[1 0; 0 0];
    % B matrix
    B=[T_s 0; 0 0];
    % Dimensions
    [n, m] = size(B);                

    % Calculate disturbances w for prediction horizon
    w_k = zeros(num_state*N,1);
    Gamma_w = zeros(N*num_state,N*num_state);
    
elseif rel_dist<=sensor_range % Condition to switch to distance control
    mode = 0;       % Indication of distance control mode
    q_1 = 100;        % penalty for acceleration
    q_2 = 1000;     % penatly for epsilon_1
    q_3 = 0;        % penalty for deviation from set speed tracking
    % Constraints Parameters
    h_min = 2;
    h_max = 4;
    d_min = 3; % Minimum distance at standstill
    
    % A matrix
    A=[1 0; -T_s 1];
    % B matrix
    B=[T_s 0; -0.5*T_s^2 0];
    % Dimensions
    [n, m] = size(B);    
    
    % Gamma_w & disturbance w
    % Calculate disturbances w for prediction horizon
    w_k = zeros(num_state*N,1);
    w_k(1:num_state:num_state*N) = 0; 
    w_k(num_state:num_state:num_state*N) = T_s/2*(V_p(1:end-1)+ V_p(2:end));
    %w_k(num_state:num_state:num_state*N)=T_s*(V_p_bar(k+1));        %OR: T_s*(V_p_bar(k+i))

    Gamma_w = zeros(N*num_state,N*num_state);
    I = eye(num_state,num_state);
    for r = 1:N
        for c = 1:N
            if r >= c
                Gamma_w((r-1)*n+1:r*n,(c-1)*n+1:c*n) = A^(r-c)*I;
            end
        end
    end
end

%% Defining the prediction matrices Phi and Gamma
Phi = zeros(N*n,n);
Gamma = zeros(N*n,N*m);
for r = 1:N
    Phi((r-1)*n+1:r*n,1:n) = A^r;
    for c = 1:N
        if r >= c
            Gamma((r-1)*n+1:r*n,(c-1)*m+1:c*m) = A^(r-c)*B;
        end
    end
end
 
%% Variable c4
c4 = zeros(1,N);
c4(1:end) = q_3*V_set.^2;

%% Vectors c3 and C3:
c3 = zeros(1, num_state);
c3(1)= -2*q_3*V_set;
Big_c3 = zeros(1, num_state*N);
Big_c3(1:num_state:end-num_state) = c3(1);

%% Matrices c1 and matrix C1
c1{1} = [q_1 0; 0 q_2]; 
Big_c1 = zeros(num_control*N, num_control*N);
for i = 1:num_control:num_control*N-num_control
    Big_c1(i:i+num_state-1, i:i+num_state-1) = c1{1};
end

%% Matrices c2 and C2
c2{1} = [q_3 0; 0 0];
Big_c2 = zeros(num_control*N, num_control*N);
for i = 1:num_control:num_control*N-num_control
    Big_c2(i:i+num_control-1, i:i+num_control-1) = c2{1};
end

f_c = sum(c4) + c3(1:num_state)*x_k + x_k'*c2{1}*x_k;
L = Phi*x_k + Gamma_w*w_k;
% Matrices for QP problem
H = 2*(Big_c1 + Gamma'*Big_c2*Gamma);
H = (H+H')/2;
f = 2*L'*Big_c2*Gamma + Big_c3*Gamma;

M_k=[0 0; 0 0; p0*p1-p3 0; -p0*p1 0; -p0*p1-p3 0];  % Also known as M_0 
M_i=[-1 0;1 0; h_min -1; -h_max 1; 0 0;0 0; p0*p1-p3 0; -p0*p1 0; -p0*p1-p3 0];
%M_last=[-1 0;1 0;h_min -1];  % Also known as M_N
%chance constraint at step k
G_1=[0 0;0 0; 0 0;0 0;0 0];

E_1=[-1 0; 1 0; m_eq 0; -m_eq 0; -m_eq 0];
E_rest=[0 0; 0 0; 0 0; 0 -1; -1 0; 1 0; m_eq 0; -m_eq 0; -m_eq 0];

%sig=linspace(0.5,5,N);  %kmph

%sig_ms=sig./3.6;        %m/s

%var_vp=zeros(2*N,1);
%var_vp(2:2:end)=sig_ms.^2;

%Aa=eye(2*N,2*N);

%Bw=eye(2*N,2*N);


%var=zeros(2*N,1);
%var(4:2:end)=cov(i);
%cov_x=[];
%cov_x=Aa.*var.*Aa'+Bw.*var_vp.*Bw';
%varr_rel_dist=cov_x(logical(eye(size(cov_x))));
%var_rel_dist=zeros(2*N,1);
%var_rel_dist(1:2:end-1)=varr_rel_dist(2:2:end);




num_var=2;
alpha_min=0.06;
alpha_max=0.05;

sig=0.5/3.6;
%additional term for chance constraint G_k*d_k
   
%d_k=zeros(N*num_var,1);
%d_k(1:num_var:end-1)=%var_rel_dist(1:2:end-1).^0.5;

d_k=zeros(2*N,2);
sigma_x=cov_x(N,sig);
d_k(1:2,1:2)=sigma_x(:,:,1);
d_k(3:4,1:2)=sigma_x(:,:,2);
d_k(5:6,1:2)=sigma_x(:,:,3);
d_k(7:8,1:2)=sigma_x(:,:,4);
d_k(9:10,1:2)=sigma_x(:,:,5);
d_k(11:12,1:2)=sigma_x(:,:,6);
d_k(13:14,1:2)=sigma_x(:,:,7);
d_k(15:16,1:2)=sigma_x(:,:,8);
d_k(17:18,1:2)=sigma_x(:,:,9);
d_k(19:20,1:2)=sigma_x(:,:,10);
d_k(21:22,1:2)=sigma_x(:,:,11);
d_k(23:24,1:2)=sigma_x(:,:,12);
d_k(25:26,1:2)=sigma_x(:,:,13);
d_k(27:28,1:2)=sigma_x(:,:,14);
d_k(29:30,1:2)=sigma_x(:,:,15);
d_k(31:32,1:2)=sigma_x(:,:,16);
d_k(33:34,1:2)=sigma_x(:,:,17);
d_k(35:36,1:2)=sigma_x(:,:,18);
d_k(37:38,1:2)=sigma_x(:,:,19);
d_k(39:40,1:2)=sigma_x(:,:,20);

G_i=[0 0;0 0;norminv(1-alpha_min) 0;norminv(1-alpha_max) 0;0 0;0 0;0 0;0 0;0 0];    %alpha_min and alpha_max are constant 

B_1=[-a_min; a_max; p4-c_r*m_eq*g*cos(theta(1))-p0*p2-m_eq*g*sin(theta(1));...
    c_r*m_eq*g*cos(theta(1))+p0*p2+m_eq*g*sin(theta(1))-F_hydr_max; p4+c_r*m_eq*g*cos(theta(1))+p0*p2+m_eq*g*sin(theta(1))];
for j=1:N-1
B_i{j}=[-Vmin(j+1); Vmax(j+1); -d_min; d_min; -a_min; a_max; p4-c_r*m_eq*g*cos(theta(j+1))-p0*p2-m_eq*g*sin(theta(j+1));...
    c_r*m_eq*g*cos(theta(j+1))+p0*p2+m_eq*g*sin(theta(j+1))-F_hydr_max; p4+c_r*m_eq*g*cos(theta(j+1))+p0*p2+m_eq*g*sin(theta(j+1))];
end 
%B_last=[-Vmin(end);Vmax(end);-d_min];

E_k_dash = zeros(length(E_1)+(N-1)*length(E_rest),num_control*N);
M_k_dash = zeros(length(M_k)+(N-1)*length(M_i),num_state*N);
D_k_dash = zeros(length(M_k_dash),num_state);
B_k_dash = zeros(length(E_k_dash),1);

% G_k_dash for chance constraints
G_k_dash = zeros(length(G_1)+(N-1)*length(G_i),num_var*N);


% Formulate D_k_dash
D_k_dash(1:length(M_k),num_state-1:num_state) = M_k; 

% Formulate M_k_dash
for r = 1:N-1
    M_k_dash((length(M_k)+1)+(r-1)*length(M_i):(length(M_k)+1)+(r*length(M_i)-1),num_state*(r-1)+1:num_state*r) = M_i;  
end
%M_k_dash((length(M_k)+1)+r*length(M_i):(length(M_k)+1)+r*length(M_i)+length(M_last)-1,num_state*r+1:num_state*N) = M_last;  
       
% Formulate E_k_dash
E_k_dash(1:length(E_1),1:num_control) = E_1; 
for r = 1:N-1
    E_k_dash((length(E_1)+1)+(r-1)*length(E_rest):(length(E_1)+1)+(r*length(E_rest)-1),num_control*r+1:num_control*(r+1)) = E_rest;  
end

% Formulate G_k_dash
G_k_dash(1:length(G_1),1:num_var) = G_1; 
for r = 1:N-1
    G_k_dash((length(G_1)+1)+(r-1)*length(G_i):(length(G_1)+1)+(r*length(G_i)-1),num_var*r+1:num_var*(r+1)) = G_i;  
end

% Formulate B_k_dash
B_k_dash(1:length(B_1),1) = B_1; 
for r = 1:N-1  
    B_k_dash(length(B_1)+1+(r-1)*length(B_i{1}):length(B_1)+1+(r*length(B_i{1})-1),1) = B_i{r};  
end
%B_k_dash(length(B_1)+1+(r*length(B_i{1})):length(B_1)+1+(r*length(B_i{1}))+length(B_last)-1,1) = B_last;  

% constraints for change in acceleration
h_1 = [1 0 0 0; -1 0 0 0];
h_2 = [-1 0 1 0; 1 0 -1 0];
% Constraints for change in acceleration
B_k_dash_2 = delta_amax * ones(2*N, 1);
D_k_dash_2 = zeros(2*N, num_state);
M_k_dash_2 = zeros(2*N, num_state*N);
E_k_dash_2 = zeros(2*N, num_control*N);

G_k_dash_2=zeros(2*N,num_var*N);
%there will be no change here as we are not changing comfort constraint

B_k_dash_2(1:2, 1) = [a_Prev + delta_amax; -a_Prev + delta_amax];
E_k_dash_2(1:2, 1:4) = h_1;
for i = 3:2:2*N-1
    E_k_dash_2(i:i+1, 1+(i-3):length(h_2)+(i-3)) = h_2;
end

% Merging '_dash' and '_dash_2' matrices
B_dash = [B_k_dash; B_k_dash_2];
M_dash = [M_k_dash; M_k_dash_2];
D_dash = [D_k_dash; D_k_dash_2];
E_dash = [E_k_dash; E_k_dash_2];

%no need to to merge matrices for chance constraints?
G_dash=[G_k_dash; G_k_dash_2];


A_ieq = M_dash*Gamma+E_dash;
B_ieq = B_dash-D_dash*x_k -G_dash*d_k-M_dash*(Phi*x_k + Gamma_w*w_k);      %G_dash*d_i added here

%options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','iter');
options.Display = 'none';
[control, cost, exitflag] = quadprog(H,f,A_ieq,B_ieq,[],[],[],[],[],options);

a = control(1);
epsilon_1 = control(2);

X_predict = Phi*x_k + Gamma*control + Gamma_w*w_k;
v_predict = X_predict(1:num_state:end);
d_predict = X_predict(2:num_state:end);

end
