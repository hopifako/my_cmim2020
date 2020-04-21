% Dynamic analysis for Pendulum

clear
end_time = 2.5;
time_step = 0.1;
start_angle = deg2rad(70);
%% Coordinates
% ground
q1 = [0; 0; 0];
% crank
q2 = [0.1 * sin(start_angle)
    -0.1 * cos(start_angle)
    start_angle];

q_0 = [q1; q2]; % initial coordinates
qp_0 = zeros(length(q_0),1);

%% Revolute joints
% 1 connects ground and crank
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [0; 0.1];

%% Simple constraints

% Three simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

%% Empty for dynamic analysis
driving = [];

%% Define bodies
body(1).m = 2; % mass equals to one kg
a = 0.1;
b = 0.1; 
body(1).Ic = body(1).m * (a^2 + b^2) / 12;
%body(1).q = [0;0;0];  %body coordinates 

body(2).m = 0.5; % mass equals to one kg
l = 0.2; 
body(2).Ic = body(1).m * l^2 / 12;
% body(2).q = [0;-1;0]; %body coordinates

%% Get mass matrix

M = mass_matrix(body);
%q0 = system_coordinates(body);

grav = [0; -9.81]; % gravitational acceleration

%% Add single force to the system
sforce(1).f = [0; 0];   %force vector in global coordinate system
sforce(1).i = 1;        %index for body which is attacked by force
sforce(1).u_i = [0; 0]; %vector to force application point in local coordinate system

F = @(t, q, qip) force_vector(grav, sforce, body, q_0);

%% Create constraint equations
C_fun = @(t, q, qip) constraint(revolute, simple, driving, t, q, qip);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q, qip) constraint_dt_complete(revolute, simple, driving, t, q, qip);
Ctt_fun = @(t, q, qip) constraint_dtdt_new(revolute, simple, driving, t, q, qip);

alpha = 10;
beta = 10;

master_matrix = @(t, q, qip) [M , (Cq_fun(t, q))' ; Cq_fun(t, q) , zeros(length(C_fun(0,q_0,qp_0)))];
master_rhs = @(t, q, qip) [F(t, q, qip) ;... 
                            Ctt_fun(t, q, qip) - (2*alpha*Ct_fun(t,q, qip)) - (beta^2*C_fun(t,q,qip))];

                        
acc_f = @(t, q, qip) (master_matrix(t, q, qip) \ master_rhs(t, q, qip));
truncate_matrix = [eye(length(M)),zeros(6,5)];
acc_f = @(t, q, qip) truncate_matrix * acc_f(t, q, qip);
                
acc_f_ode = @(t,y) [y(length(M)+1:end) ;...
                    acc_f(t, y(1:length(M)), y(length(M)+1:end))];

opts = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);
y0 = [q_0,qp_0];
disp('start ode45 solver')
[t,solution_ode] = ode45(acc_f_ode,[0,end_time],y0,opts);
%[t, u, v] = EulerCromer(acc_f, 5, q_0, qp_0, 0.00001);

u = solution_ode(:,1:length(M));
v = solution_ode(:,length(M)+1:end);
acc = zeros(size(u));
for ii = 1:length(t)
    acc(ii,:) = acc_f(t(ii),u(ii,:)',v(ii,:)')';
end

figure; 
plot(u(:,4),u(:,5),...
     0,0,'*', 'LineWidth', 2);       
axis equal
xlabel('x-postion [m]')
ylabel('y-postion [m]')

figure;
plot(t,u(:,4),t,u(:,5))
legend('x-Position','y-Position')

figure;
plot(t,u(:,6),t,v(:,6),t,acc(:,6), 'LineWidth',1)
legend('angle theta_2', 'angular velocity', 'angular acceleration')
xlabel('time [s]')
ylabel('angular pos./vel./acc. [rad]/[rad/s]/[rad/s^2]')

%figure; plot(t,u(:,4),t,v(:,4),t,acc(:,4))
%figure; plot(t,u(:,6),t,v(:,6),t,acc(:,6))
