clear
clc

% start values 
theta0 = 0.5;
d0 = 2;
start_value_displacement = [theta0 ; d0];
dtheta_dt0 = 1;
v0 = 0.5;
start_value_velocity = [dtheta_dt0 ; v0];

% Error and time definition 
eps = 1e-6; %maximum error 
max_number_iteration = 1000; %maximum number of iteration for each time step
T = 10; %endtime
timesteps = 100;

time_vec  = linspace(0,T,timesteps);
theta = zeros(1,timesteps);

%Calculation of displacement d and theta
[solution_d , number_iteration_d] = newton_system(@F_d,@J_d,start_value_displacement,time_vec,eps,max_number_iteration,theta);
theta = solution_d(1,:); %Radiant
d = solution_d(2,:);
number_iteration_d

%Calculation of velocity v and dtheta/dt
[solution_v , number_iteration_v] = newton_system(@F_v,@J_v,start_value_velocity,time_vec,eps,max_number_iteration,theta);
dtheta_dt = solution_v(1,:); %Radiant/s
v = solution_v(2,:);
number_iteration_v

%% Plots
figure 
plot(time_vec,(theta/pi * 180)) %plot in [°]
xlabel('Time t [s]')
ylabel('Angle theta [°]')
title('Angle theta over time')
figure 
plot(time_vec,d)
xlabel('Time t [s]')
ylabel('Displacement d [m]')
title('Displacement d over time')

figure 
plot(time_vec,dtheta_dt,'Color',[0.85,0.33,0.1]) %plot in [rad/s]
xlabel('Time t [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity dtheta/dt over time')
figure 
plot(time_vec,v,'Color',[0.85,0.33,0.1])
xlabel('Time t [s]')
ylabel('Velocity v [m/s]')
title('Velocity v over time')

%% functions for distance 
function F_vector = F_d(x,t,theta)
a = 0.1;
b = 0.2;
omega = 1;
phi = pi/6 + omega*t;
F_vector = [a*cos(phi) + b*cos(x(1)) - x(2);...
            a*sin(phi) - b*sin(x(1))];
end
function J_matrix = J_d(x,t,theta)
b = 0.2;
J_matrix = [-b*sin(x(1)) -1;...
            -b*cos(x(1)) 0];
end

%% functions for velocity 
function F_vector = F_v(x,t,theta)
a = 0.1;
b = 0.2;
omega = 1;
phi = pi/6 + omega*t;

F_vector = [-a*sin(phi)*omega - b*sin(theta)*x(1) - x(2);...
            a*cos(phi)*omega - b*cos(theta)*x(1)];
end
function J_matrix = J_v(x,t,theta)
a = 0.1;
b = 0.2;
omega = 1;
phi = pi/6 + omega*t;

J_matrix = [-b*sin(theta) -1;...
            -b*cos(theta) 0];
end
