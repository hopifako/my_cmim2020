%% Exercise 4.4
%Solution of Exercise 4.4 from S. Lingeand H. P. Langtangen book
clear
delta_t0 = 8;
end_time = 40;
N_0 = 10; %start value
M = 500; %carrying capacity
r_start = 0.3; %start growing rate
k = 0; %loop index k 

while true
    
    delta_t = 2^(-k) * delta_t0; %Calculate new delta_t
    t = 0 : delta_t : end_time; %time vector
    N = zeros(1,length(t)); %vector size of population
    r = r_start * (1 - (N_0/M)); 
    N(1) = N_0;


    for n = 1:length(t)-1
        N(n+1) = N(n) + delta_t * r * N(n); %Forward Euler to calculate size of population
        r = r_start * (1 - (N(n)/M)); %calculate new r-value for next step n+1
    end
    
    if k >= 1
        figure %Plot N for k and k_-1
        plot(t,N,t_old,N_old)
        legendInfo1 = sprintf('N for dt = %g (k = %.g)', delta_t , k);
        legendInfo2 = sprintf('N for dt = %g (k = %.g)',delta_t_old , k-1);
        legend(legendInfo1 , legendInfo2)
        title('Logisitc growth of population');
        xlabel('Time t');
        ylabel('Population N')
          
        user_input = input('continue? [y/n] \n','s');
        if strcmp(user_input,'n')
            fprintf('Program has stopped \n');
            break;            
        elseif strcmp(user_input,'y')
        else
            fprintf('Invalid answer, program proceeds \n')
        end
    end
    
    N_old = N; %Save current N(k) for plot with next N(k+1) 
    t_old = t; %Save current t(k) for plot with next t(k+1) 
    delta_t_old = delta_t;
    k = k + 1; %raise loop index 

end
