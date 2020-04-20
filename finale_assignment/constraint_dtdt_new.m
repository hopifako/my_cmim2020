function Ctt = constraint_dtdt_new(revolute, simple, driving, t, q, qip)

r_len = length(revolute);
s_len = length(simple);
d_len = length(driving);

n_constr = 2 * r_len + s_len + d_len;

Ctt = zeros(n_constr, 1);

c_idx = 0;

for r = revolute
    Ctt(c_idx + (1:2), 1) = ...
        revolute_joint_acc(r.i, r.j, r.s_i, r.s_j, q, qip);
    c_idx = c_idx + 2;
end

% for s = simple
%     c_idx = c_idx + 1;
%     Ctt(c_idx, body_idx(s.i)) = simple_joint_dq(s.k);
% end
% 
% for d = driving
%     c_idx = c_idx + 1;
%     Ctt(c_idx, body_idx(d.i)) = driving_joint_dq(d.k);
% end