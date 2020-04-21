function C_t = revolute_joint_vel(i, j, s_i, s_j, q, qp)

idx_i = body_idx(i);
%r_i = q(idx_i(1:2));
phi_i = q(idx_i(3));
r_i_t = qp(idx_i(1:2));
phi_i_t = qp(idx_i(3));

idx_j = body_idx(j);
%r_j = q(idx_j(1:2));
phi_j = q(idx_j(3));
r_j_t = qp(idx_j(1:2));
phi_j_t = qp(idx_j(3));

C_t = r_i_t + phi_i_t*omega*rot(phi_i)*s_i - r_j_t - phi_j_t*omega*rot(phi_j)*s_j;

