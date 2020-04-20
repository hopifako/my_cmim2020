function [T, Q, QP, QPP, master_sol] =... 
            solve_master_matrix(C_fun, Cq_fun, Ct_fun, Ctt_fun, tend, q_0, dt, M, F)
% M: mass matrix
% F: force vector

alpha = 10; %coefficent for solving dynamic equation (Baumgarte method)
beta = 10;  %coefficent for solving dynamic equation (Baumgarte method)
N_t = floor(round(tend/dt));
T = linspace(0, N_t*dt, N_t+1)';
Q = zeros(N_t+1, length(q_0));
QP = zeros(N_t+1, length(q_0));
QPP = zeros(N_t+1, length(q_0));
master_sol = zeros(N_t+1, 2*length(q_0));
% [x, iteration_counter] = NR_method(F, J, x, eps)

[qi, icnt] = NR_method(@(q) C_fun(0, q), @(q) Cq_fun(0, q), q_0, 1e-8);
Cqi = Cq_fun(0, qi);
qip = -Cqi\Ct_fun(0, qi);
qipp = -Cqi\Ctt_fun(0, qi, qip); %qipp = g (from lecture slides)
Q(1, :) = qi';
QP(1, :) = qip';
QPP(1, :) = qipp';


% Step equations forward in time
for n = 1 : N_t
    [qi, icnt] = NR_method(@(q) C_fun(T(n + 1), q), ...
        @(q) Cq_fun(T(n + 1), q), ...
        qi, 1e-8);
    Cqi = Cq_fun(T(n + 1), qi);
    qip = -Cqi\Ct_fun(T(n + 1), qi);
    qipp = -Cqi\Ctt_fun(T(n + 1), qi, qip); 
    disp(icnt)
    Q(n + 1, :) = qi';
    QP(n + 1, :) = qip';
    QPP(n + 1, :) = qipp';
    
    master_matrix = [M , Cqi' ; Cqi , zeros(length(M))];
    master_rhs = [F(T(n + 1), qi, qip) ; qipp - (2*alpha*Ct_fun(T(n + 1),qip)) - (beta^2*Ct_fun(T(n + 1),qi))];
    master_sol(n+1,:) = (master_matrix \ master_rhs)';
end
master_sol = master_sol(:,1:length(q_0));