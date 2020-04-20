function C_r_dq = revolute_joint_acc(i, j, s_i, s_j, q, qp)

idx_i = body_idx(i);
%r_i = q(idx_i(1:2));
phi_i = q(idx_i(3));
phi_i_t = qp(idx_i(3));

idx_j = body_idx(j);
%r_j = q(idx_j(1:2));
phi_j = q(idx_j(3));
phi_j_t = qp(idx_j(3));

C_r_dq = rot(phi_i)*s_i*phi_i_t^2 - (rot(phi_j)*s_j*phi_j_t^2);

