function [q, ok] = ik_ur5(T_cible, q_init, robot, P)
% MGI numerique du UR5 : pose cible (4x4) -> q (6x1) [rad]
% 100% pur MATLAB - Ne necessite AUCUNE toolbox (remplace inverseKinematics)
% Methode : Damped Least Squares (Levenberg-Marquardt)

if nargin < 4 || isempty(P)
    P.tol_ik_pos = 1e-4;
    P.tol_ik_rot = 1e-3;
end

p_cible = T_cible(1:3, 4);
R_cible = T_cible(1:3, 1:3);

q = q_init(:);
lambda = 0.01;       % Amortissement
max_iter = 150;
ok = false;

for iter = 1:max_iter
    [p_cur, R_cur] = fk_ur5(q, robot, P);
    
    e_pos = p_cible - p_cur;
    e_rot = 0.5 * (cross(R_cur(:,1), R_cible(:,1)) + ...
                   cross(R_cur(:,2), R_cible(:,2)) + ...
                   cross(R_cur(:,3), R_cible(:,3)));
    
    err_pos = norm(e_pos);
    err_rot = norm(R_cible - R_cur, 'fro');
    
    if (err_pos < P.tol_ik_pos) && (err_rot < P.tol_ik_rot)
        ok = true;
        break;
    end
    
    % Jacobien numerique (6x6)
    e = [e_rot; e_pos];
    J = zeros(6, 6);
    delta = 1e-6;
    for j = 1:6
        q_pert = q;
        q_pert(j) = q_pert(j) + delta;
        [p_pert, R_pert] = fk_ur5(q_pert, robot, P);
        dp = (p_pert - p_cur) / delta;
        dR = 0.5 * (cross(R_cur(:,1), R_pert(:,1)) + ...
                    cross(R_cur(:,2), R_pert(:,2)) + ...
                    cross(R_cur(:,3), R_pert(:,3))) / delta;
        J(:, j) = [dR; dp];
    end
    
    % Mise a jour amortie (Damped Least Squares)
    dq = (J' * J + (lambda^2) * eye(6)) \ (J' * e);
    q = q + dq;
end

% Normalisation des angles dans [-pi, pi]
q_ref = q_init(:);
q = q_ref + mod(q - q_ref + pi, 2*pi) - pi;   % solution la plus proche de q_init
end
