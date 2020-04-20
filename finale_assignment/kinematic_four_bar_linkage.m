
% Kinamatic analysis of four-bar linkage

clear
end_time = 4.2;
time_step = 0.01;

%% Coordinates
l_crank = 0.2;
l_coupler = 0.5;
l_follower = 0.4;
frame_pos_1 = [0;0];
frame_pos_2 = [0.4;0.2];
x_AD = 0.4;
y_AD = 0.2;
theta2_0 = 0;
q_0 = calculate_start_values(l_crank,l_coupler,l_follower,theta2_0,frame_pos_1,frame_pos_2);

%% Revolute joints
% 1 connects frame and crank
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [0; -0.1];

% 2 connects crank and coupler
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [0; 0.1];
revolute(2).s_j = [-0.25; 0];

% 3 connects coupler and follower
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [0.25; 0];
revolute(3).s_j = [0; 0.2];

% 4 connects follower and frame
revolute(4).i = 4;
revolute(4).j = 5;
revolute(4).s_i = [0; -0.2];
revolute(4).s_j = [0; 0];

%% Simple constraints

% Three simple joints to fix frame 1 (left side)
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

% Three simple joints to fix frame 2 (right side)
simple(4).i = 5;
simple(4).k = 1;
simple(4).c_k = frame_pos_2(1);

simple(5).i = 5;
simple(5).k = 2;
simple(5).c_k = frame_pos_2(2);

simple(6).i = 5;
simple(6).k = 3;
simple(6).c_k = 0;

%% Add driving constraints
driving.i = 2;
driving.k = 3;
driving.d_k = @(t,q) pi/9 - 1.5 * t;
driving.d_k_t = @(t,q) -1.5;
driving.d_k_tt = @(t,q) 0;

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
    Q(1,1), Q(1,2),'*',...
    Q(1,13), Q(1,14),'*', 'LineWidth', 2);
legend('Crank','Coupler','Collower','Frame joint 1','Frame joint 2','Location', 'southeast')
axis equal
%title('Position Trajectory')

% Velocity
figure;
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    QP(:, 10), QP(:, 11), ...
    QP(1,1), QP(1,2),'*',...
    QP(1,13), QP(1,14),'*', 'LineWidth', 2);
legend('Crank','Coupler','Collower','Frame joint 1','Frame joint 2','Location', 'southeast')
axis equal
%title('Velocity')

% Acceleration
figure;
plot(QPP(:, 4), QPP(:, 5), ...
    QPP(:, 7), QPP(:, 8), ...
    QPP(:, 10), QPP(:, 11), ...
    QPP(1,1), QPP(1,2),'*',...
    QPP(1,13), QPP(1,14),'*', 'LineWidth', 2);
legend('Crank','Coupler','Collower','Frame joint 1','Frame joint 2','Location', 'southeast')
axis equal
%title('Accelertion')


