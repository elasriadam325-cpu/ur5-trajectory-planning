function [q, p] = traj_cartesienne(pA, C, pB, R, T, t, q0, robot, P, s)
% Trajectoire cartesienne : chemin de Bezier parcouru avec une loi horaire s(t),
% orientation constante R, MGI a chaque pas (solution la plus proche du pas precedent)
% s : loi horaire normalisee 1xN de 0 a 1 (optionnelle, defaut : minimum jerk)
% Retourne q (6xN) [rad] et p (3xN) [m]
if nargin < 10 || isempty(s)
    s = traj_minjerk(0, 1, T, t);
end
p = bezier2(pA, C, pB, s);

Pik = P;
Pik.tol_ik_pos = P.tol_traj_pos;
Pik.tol_ik_rot = P.tol_traj_rot;

N = numel(t);
q = zeros(6, N);
qc = q0(:);
for k = 1:N
    Tc = [R, p(:,k); 0 0 0 1];
    [qn, ok] = ik_ur5(Tc, qc, robot, Pik);
    if ~ok
        error('traj_cartesienne:ikEchec', ...
              'MGI non convergente au pas %d (t = %.3f s) : singularite ou point hors espace de travail ?', k, t(k));
    end
    qn = qc + mod(qn - qc + pi, 2*pi) - pi;   % evite les sauts de 2*pi
    q(:,k) = qn;
    qc = qn;
end
end