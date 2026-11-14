function [q, qd, qdd] = traj_cubique(q0, qf, T, t)
% Cubique repos-repos. q0, qf : n x 1 [rad] ; t : 1 x N [s]
% Retourne q, qd, qdd : n x N
q0 = q0(:); qf = qf(:); d = qf - q0;
tau = min(max(t(:).'/T, 0), 1);
q   = q0 + d * (3*tau.^2 - 2*tau.^3);
qd  = d * (6*tau - 6*tau.^2) / T;
qdd = d * (6 - 12*tau) / T^2;
end