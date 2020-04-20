%% Kinematic and dynamic analysis of slider-crank system
clear

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
% Slider crank kinematic analysis
end_time = 12;
time_step = 0.1;

%% Coordinates
% ground
q1 = [0; 0; 0];
% crank
start_angle = 0; %in degree
q2 = [-0.1 * cosd(start_angle)
    0.1 * sind(start_angle)
    -deg2rad(start_angle)];
% link     %%%%%% NOTE: postion of point B changed in the middle of link 
h_B = 0.2 * sind(start_angle); % y coordinate of point B
phi_l = asin(h_B / 0.5); % link's angle
q3 = [-0.2 * cosd(start_angle) - 0.25 * cos(phi_l)
    h_B - 0.25 * sin(phi_l)
    phi_l];
% slider
q4 = [-0.2 * cosd(start_angle) - 0.5 * cos(phi_l)
    0
    0];

q_0 = [q1; q2; q3; q4]; % initial coordinates

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Define bodies
body(1).m = 10; % mass equals to one kg
r = 0.1; 
body(1).Ic = body(1).m * (2/5) * r^2;
body(1).q = q1;  %body coordinates 

body(2).m = 0.4; % mass equals to one kg
l = 0.2; 
body(2).Ic = body(2).m * l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(2).q = q2; %body coordinates

body(3).m = 1; % mass equals to one kg
l = 0.5; 
body(3).Ic = body(3).m * l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(3).q = q3; %body coordinates

body(4).m = 2; % mass equals to one kg
a = 0.05;
b = 0.1; 
body(4).Ic = body(4).m * (a^2 + b^2) / 12; % mass moment of inertia along center of mass in kgm2
body(4).q = q4; %body coordinates

%% Get mass matrix

M = mass_matrix(body);
q0 = system_coordinates(body);

grav = [0; 0]; % gravitational acceleration

%% Add single force to the system
% sforce(1).f = [0; -2*grav(2)];   %force vector in global coordinate system
% sforce(1).i = 1;        %index for body which is attacked by force
% sforce(1).u_i = [0; 0]; %vector to force application point in local coordinate system

% sforce(1).f = @(t,qi,qip)[-20*exp(-(t));0];%@(t) 5*exp(-2*t).*[sin(t); cos(t)];
% sforce(1).i = 4;
% sforce(1).u_i = [0; 0];

sforce(1).f = [0; 0];
sforce(1).i = 1;
sforce(1).u_i = [0; 0];

%F = force_vector(grav, sforce, body, q0);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

%% We need two constraint types (geometric ones)
% - revolute
% - simple constraints

%% Revolute joints
% 1 connects ground and crank
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [0.1; 0];

% 2 connects crank and link
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [-0.1; 0];
revolute(2).s_j = [0.25; 0]; % changed: [0.3; 0]

% 3 connects link and slider
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [-0.25; 0]; %changed: [-0.2; 0];
revolute(3).s_j = [0; 0];

% % Check revolute joint constraints
% r = revolute(3);
% C_r_i = revolute_joint(r.i, r.j, r.s_i, r.s_j, q_0)

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

% slider - use simple joints instead of translational
% simple(4).i = 4;
% simple(4).k = 1;
% simple(4).c_k = 0;

simple(4).i = 4;
simple(4).k = 2;
simple(4).c_k = 0;

simple(5).i = 4;
simple(5).k = 3;
simple(5).c_k = 0;

% % check simple constraints
% for s = simple
%     C_s_i = simple_joint(s.i, s.k, s.c_k, q_0)
% end

%% Add some driving constraints
driving.i = 2;
driving.k = 3;
% driving.d_k = @(t,q) (start_angle/180)*pi - 1.2 * t;
% driving.d_k_t = @(t,q) -1.2;
% driving.d_k_tt = @(t,q) 0;     (start_angle/180)*pi
driving.d_k = @(t,q) (start_angle/180)*pi;
driving.d_k_t = @(t,q) 0;
driving.d_k_tt = @(t,q) 0;
%%
% % Verify
% d = driving(1);
% C_d_i = driving_joint(d.i, d.k, d.d_k, 0, q_0)

% %% Verify constraint function
% clc
% C = constraint(revolute, simple, driving, 0, q_0)

% %% Solve constraint equation using fsolve
% C_fun = @(t, q) constraint(revolute, simple, driving, t, q)
% [T, Q] = position_fsolve(C_fun, end_time, q_0, time_step);
% 
% %% Some verification plots
% figure;
% plot(Q(:, 4), Q(:, 5), ...
%     Q(:, 7), Q(:, 8), ...
%     Q(:, 10), Q(:, 11), ...c
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% 
% %% Jacobian of our constraints
% Cq = constraint_dq(revolute, simple, driving, 0, q_0)
% 
% %% Solve constraint equation using NR
% C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
% Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
% [T, Q] = position_NR(C_fun, Cq_fun, end_time, q_0, time_step);
% 
% %% Some verification plots
% figure;
% plot(Q(:, 4), Q(:, 5), ...
%     Q(:, 7), Q(:, 8), ...
%     Q(:, 10), Q(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal

% %% Verify Ct
% Ct = constraint_dt(revolute, simple, driving, 0, q_0)
% 
% %% Solve constraint equation using NR for position and velocity
% C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
% Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
% Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
% [T, Q, QP] = pos_vel_NR(C_fun, Cq_fun, Ct_fun, end_time, q_0, time_step);

% %% Some verification plots
% figure;
% plot(Q(:, 4), Q(:, 5), ...
%     Q(:, 7), Q(:, 8), ...
%     Q(:, 10), Q(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% 
% 
% %% Some verification plots
% figure;
% plot(QP(:, 4), QP(:, 5), ...
%     QP(:, 7), QP(:, 8), ...
%     QP(:, 10), QP(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal

%%
%.................................................................................................
% Acceleration analyses 
%.................................................................................................
% %% Verify Ctt
% qip_0 = QP(1,:)';
% Ctt = constraint_dtdt_new(revolute, simple, driving, 0, q_0, qip_0)

%% Solve constraint equation using NR for position, velocity and acceleration
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
Ctt_fun = @(t, q, qip) constraint_dtdt_new(revolute, simple, driving, t, q, qip);

[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun, Ctt_fun, end_time, q_0, time_step);

%acc_fun = @(t, q, qip) system_accelerations(t, q, qip, M, sforce, grav, body);
alpha = 10; %coefficent for solving dynamic equation (Baumgarte method)
beta = 5;  %coefficent for solving dynamic equation (Baumgarte method)

F = @(t, q, qip) force_vector(grav, sforce, body, q);

master_matrix = @(t, q, qip) [M , (Cq_fun(t, q))' ; Cq_fun(t, q) , zeros(length(M))];
master_rhs = @(t, q, qip) [F(t, q, qip) ;... 
                            Ctt_fun(t, q, qip) - (2*alpha*Ct_fun(t,q)) - (beta^2*C_fun(t,q))];

% Probable bullshit
% [T, Q, QP, QPP, acc_dynamic] =... 
%             solve_master_matrix(C_fun, Cq_fun, Ct_fun, Ctt_fun, end_time, q_0, time_step, M, F);                        
% vel_dynamic = cumtrapz(acc_dynamic);
% pos_dynamic = cumtrapz(vel_dynamic); 
                        
acc_f = @(t, q, qip) (master_matrix(t, q, qip) \ master_rhs(t, q, qip));
truncate_matrix = [eye(length(M)),zeros(length(M))];
acc_f = @(t, q, qip) truncate_matrix * acc_f(t, q, qip);
acc_f_ode = @(t,y) [acc_f(t,y(1:length(M)),y(length(M)+1:end)) ; y(1:length(M))];

opts = odeset('AbsTol', 1e-12, 'RelTol', 1e-9);
y0 = [QP(1,:)';q_0];
disp('start ode45 solver')
[t_ode,u_ode] = ode45(acc_f_ode,[0,end_time],y0,opts);
% disp('start euler-cromer')
% [t_euler, u_euler, v_euler] = EulerCromer(acc_f, end_time, q0, QP(1,:)', 0.001);

acc_value = zeros(length(u_ode),length(M));
for ii = 1:length(u_ode)
    acc_value(ii,:) = acc_f(t_ode(ii),u_ode(ii,1:length(M))',u_ode(ii,length(M)+1:end)');
end

figure; plot(t_ode,acc_value);
title('Acceleration');
figure;plot(t_ode,u_ode(:,1:length(M)));
title('Velocity');
figure;plot(t_ode,u_ode(:,length(M)+1:end));
title('Position');

% [T, Q, QP, QPP, master_sol] = ...
%     solve_master_matrix(C_fun, Cq_fun, Ct_fun, Ctt_fun, end_time, q_0, time_step, M, acc_fun);


%% Some verification plots
%Position
% figure;
% plot(Q(:, 4), Q(:, 5), ...
%     Q(:, 7), Q(:, 8), ...
%     Q(:, 10), Q(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% title('Position')
% 
% % Velocity
% figure;
% plot(QP(:, 4), QP(:, 5), ...
%     QP(:, 7), QP(:, 8), ...
%     QP(:, 10), QP(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% title('Velocity')
% 
% % Acceleration
% figure;
% plot(QPP(:, 4), QPP(:, 5), ...
%     QPP(:, 7), QPP(:, 8), ...
%     QPP(:, 10), QPP(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% title('Accelertion')
% 
% Pos., Vel., Acc. at point 4 in x-direction
figure;
plot(T,Q(:,10) , T,QP(:,10) , T,QPP(:,10))
legend('Position','Velocity','Acceleration');
title('Pos., Vel., Acc. at point 4 in x-direction')



