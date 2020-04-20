% Calculates start positions for four-bar linkage system
%     l_crank: length crank
%     l_coupler: length coupler
%     l_follower: length follower
%     theta2_0: start angle crank, input in degree [°]
%     frame_point_A: global position of revolute joint frame/crank
%     frame_point_D: global position of revolute joint frame/follower
    
function [q0] = calculate_start_values...
        (l_crank,l_coupler,l_follower,theta2_0,frame_point_A,frame_point_D)
theta2_0 = theta2_0 / 180 * pi;
x_AD = frame_point_D(1);
y_AD = frame_point_D(2);
q0 = zeros(15,1);
q0(1:2,1) = frame_point_A;
q0(3,1) = 0;
q0(4,1) = -0.5 * l_crank * sin(theta2_0);
q0(5,1) = 0.5 * l_crank * cos(theta2_0);
q0(6,1) = theta2_0;
    alpha = theta2_0 + pi/2 - atan(y_AD/x_AD);
    c = sqrt((x_AD^2) + (y_AD^2));
    e = sqrt(l_crank^2 + c^2 - (2*l_crank*c*cos(alpha)));
    beta1 = acos((l_crank^2+e^2-c^2) / (2*l_crank*e));
    beta2 = acos((l_coupler^2+e^2-l_follower^2) / (2*l_coupler*e));
    beta = beta1 + beta2;
q0(9,1) = theta2_0 + beta - (pi/2);
    delta = acos((l_follower^2+l_coupler^2-e^2) / (2*l_follower*l_coupler));
q0(12,1) = q0(9,1) + delta - (pi/2);
q0(7,1) = -l_crank * sin(theta2_0) + 0.5*l_coupler*cos(q0(9,1));
q0(8,1) = l_crank * cos(theta2_0) + 0.5*l_coupler*sin(q0(9,1));
q0(10,1) = q0(7,1) + 0.5*l_coupler*cos(q0(9,1)) + 0.5*l_follower*sin(q0(12,1));
q0(11,1) = q0(8,1) + 0.5*l_coupler*sin(q0(9,1)) - 0.5*l_follower*cos(q0(12,1));
q0(13:14,1) = frame_point_D;
q0(15,1) = 0;
end