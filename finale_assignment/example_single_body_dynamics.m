clear
%% Define bodies
body(1).m = 2; % mass equals to one kg
a = 0.05;
b = 0.1; 
body(1).Ic = body(1).m * (a^2 + b^2) / 12;
%l = 1; %length
%body(1).Ic = body(1).m * l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(1).q = [0;1;0];  %body coordinates 

% body(2).m = 2; % mass equals to one kg
% a = 0.05;
% b = 0.1; 
% body(2).Ic = body(2).m * (a^2 + b^2) / 12;; % mass moment of inertia along center of mass in kgm2
% body(2).q = [0;-1;0]; %body coordinates

%% Get mass matrix

M = mass_matrix(body);
q0 = system_coordinates(body);

grav = [0; -9.81]; % gravitational acceleration

%% Add single force to the system
sforce(1).f = [0; -2*grav(2)];   %force vector in global coordinate system
sforce(1).i = 1;        %index for body which is attacked by force
sforce(1).u_i = [0; 0]; %vector to force application point in local coordinate system

% sforce(2).f = [0; 1];
% sforce(2).i = 1;
% sforce(2).u_i = [1; 0];
% 
% sforce(3).f = [1; 0];
% sforce(3).i = 2;
% sforce(3).u_i = [0; 1];

F = force_vector(grav, sforce, body, q0);

%% Time to integrate it
% Note that M is constant, but F, in general, no
% We can use one of the following:
%   ode45 from Matlab
%   Euler-Cromer as introduced some time ago
%   Lets try Euler-Cromer

% acc_f = @(~, ~, ~) M\F;
% [t, u, v] = EulerCromer(acc_f, 2, q0, zeros(size(q0)), 0.01);

%% Second integration of the equations of motion

acc_f = @(t, q, qp) system_accelerations(t, q, qp, M, sforce, grav, body);
[t, u, v] = EulerCromer(acc_f, 2, q0, zeros(size(q0)), 0.001);

%% Verification plots part 2
figure; plot(t, u(:, 2), t, u(:, 1))
%figure; plot(t, u(:, 1), t, u(1, 1) + 0.25 * t .^ 2)
%figure; plot(t, u(:, 3))