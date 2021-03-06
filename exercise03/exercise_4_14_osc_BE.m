%% Exercise 4.14
%Solution of Exercise 4.14 from S. Lingeand H. P. Langtangen book
%
clear
omega = 2;
P = 2*pi/omega; %Period duration 
dt = P/2000; %timestep
T = 3*P; %end time


t = 0:dt:T; %time vector
u = zeros(1 , length(t)); %position vector
v = zeros(1 , length(t)); %velocity vector

% Initial condition
X_0 = 2;
u(1) = X_0;
v(1) = 0;

for n = 2:length(t)
    u(n) = (u(n-1) + dt*v(n-1)) / (1 + (dt*omega)^2);
    v(n) = v(n-1) - ((dt*omega^2) * ((u(n-1) + dt*v(n-1)) / (1 + (dt*omega)^2)));
end

exact_sol = X_0*cos(omega*t); %analytic solution 

figure
plot(t,u,t,exact_sol)
title(sprintf('Euler Backward with dt = %g',dt));
legend('Euler Backward' , 'Exact Solution')
xlabel('Time t');
ylabel('Position u')

