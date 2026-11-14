function [q, qd, qdd] = traj_minjerk(q0, qf, T, t)
% Minimum jerk repos-repos. Solution : polynome de degre 5 (voir traj_quintique)
[q, qd, qdd] = traj_quintique(q0, qf, T, t);
end