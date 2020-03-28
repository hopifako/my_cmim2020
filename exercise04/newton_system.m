%% function solves non-linear algebraic systems by means of Newton's Method

function [x1 , sum_iteration] = newton_system(F,J,x0,time,eps,max_number_iteration,theta)

number_iteration = zeros(1,length(time));
x1 = zeros(length(x0),length(time));

for ii = 1:length(time)
    
    F_value = F(x0,time(ii),theta(ii));

    while abs(norm(F_value)) > eps && number_iteration(ii) <= max_number_iteration
        try 
            delta = J(x0,time(ii),theta(ii)) \ -F_value;
        catch 
            fprintf('Error! at timestep t = %.g \n', time(ii));
            exit(1)
        end
        x1(:,ii) = x0 + delta;
        F_value = F(x1(:,ii),time(ii),theta(ii));
        x0 = x1(:,ii);
        number_iteration(ii) = number_iteration(ii) + 1;
    end

    if number_iteration > max_number_iteration
        fprintf('Abort calculation, reached maximum number of iterations at timepoint t = %.g \n', time(ii));
        exit(1)
    end
end

sum_iteration = sum(number_iteration);
end