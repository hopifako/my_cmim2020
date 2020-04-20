% Slider crank kinematic analysis
clear
end_time = 6;
time_step = 0.1;

%% Coordinates
% ground
q1 = [0; 0; 0];
% crank
q2 = [-0.1 * cosd(30)
    0.1 * sind(30)
    -deg2rad(30)];
% link
h_B = 0.2 * sind(30); % y coordinate of point B
phi_l = asin(h_B / 0.5); % link's angle
q3 = [-0.2 * cosd(30) - 0.3 * cos(phi_l)
    h_B - 0.3 * sin(phi_l)
    phi_l];
% slider
q4 = [-0.2 * cosd(30) - 0.5 * cos(phi_l)
    0
    0];

q_0 = [q1; q2; q3; q4]; % initial coordinates

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
revolute(2).s_j = [0.3; 0];

% 3 connects link and slider
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [-0.2; 0];
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
driving.d_k = @(t,q) -pi/6 - 1.2 * t;
driving.d_k_t = @(t,q) -1.2;
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

%% Verify Ct
Ct = constraint_dt(revolute, simple, driving, 0, q_0)

%% Solve constraint equation using NR for position and velocity
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
[T, Q, QP] = pos_vel_NR(C_fun, Cq_fun, Ct_fun, end_time, q_0, time_step);

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
%% Verify Ctt
qip_0 = QP(1,:)';
Ctt = constraint_dtdt_new(revolute, simple, driving, 0, q_0, qip_0)

%% Solve constraint equation using NR for position, velocity and acceleration
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
Ctt_fun = @(t, q, qip) constraint_dtdt_new(revolute, simple, driving, t, q, qip);

[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun, Ctt_fun, end_time, q_0, time_step);

%% Some verification plots
%Position
figure;
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
title('Position')

% Velocity
figure;
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    QP(:, 10), QP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
title('Velocity')

% Acceleration
figure;
plot(QPP(:, 4), QPP(:, 5), ...
    QPP(:, 7), QPP(:, 8), ...
    QPP(:, 10), QPP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
title('Accelertion')

% Pos., Vel., Acc. at point 4 in x-direction
figure;
plot(T,Q(:,10) , T,QP(:,10) , T,QPP(:,10))
legend('Position','Velocity','Acceleration');
title('Pos., Vel., Acc. at point 4 in x-direction')