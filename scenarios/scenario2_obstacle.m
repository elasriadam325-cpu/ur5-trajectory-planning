function R = scenario2_obstacle(P, robot)
% Scenario 2 : trajet cartesien A -> B avec un obstacle spherique sur la droite A-B
t = 0:P.dt:P.T;
[pA, RA] = fk_ur5(P.qA, robot, P);
pB = fk_ur5(P.qB, robot, P);
c = pA + P.obs_frac * (pB - pA);                  % centre de l'obstacle [m]

[C_cont, info] = planif_obstacle(pA, pB, c, P.obs_r, P.obs_marge, P);
Cs   = {(pA + pB)/2, C_cont};                     % point de controle au milieu = ligne droite
noms = {'Ligne droite', 'Contournement'};

L = cell(1,2); I = cell(1,2);
for k = 1:2
    [L{k}.q, L{k}.p] = traj_cartesienne(pA, Cs{k}, pB, RA, P.T, t, P.qA, robot, P);
    L{k}.qd   = gradient(L{k}.q,  P.dt);
    L{k}.qdd  = gradient(L{k}.qd, P.dt);
    L{k}.dist = distance_sphere(L{k}.p, c, P.obs_r);
    I{k} = indicateurs_traj(L{k}.q, L{k}.qd, L{k}.qdd, P.dt, P, robot);
    I{k}.dist_min = min(L{k}.dist);
    if I{k}.qd_max > P.qd_max
        warning('%s : vitesse articulaire max depassee (%.3f rad/s).', noms{k}, I{k}.qd_max);
    end
end

% Tableau
fprintf('\n%-15s %11s %12s %11s %8s %9s %10s %10s\n', 'Trajet', 'longueur[m]', 'dist_min[m]', ...
        'ecart[m]', 'qd_max', 'qdd_max', 'jerk_max', 'collision');
for k = 1:2
    if I{k}.dist_min < 0, txt = 'OUI'; else, txt = 'non'; end
    fprintf('%-15s %11.4f %12.4f %11.4f %8.3f %9.3f %10.1f %10s\n', noms{k}, I{k}.longueur, ...
            I{k}.dist_min, I{k}.ecart_droite, I{k}.qd_max, I{k}.qdd_max, I{k}.jerk_max, txt);
end
fprintf('\nContournement : decalage = %.3f m, marge exigee = %.3f m\n', info.decalage, P.obs_marge);

if ~exist(P.dossier_resultats, 'dir'), mkdir(P.dossier_resultats); end
coul = {[0.85 0.2 0.2], [0.1 0.4 0.8]};

% Figure 3 : trajets 3D et obstacle
figure('Name', 'S2 - trajets et obstacle', 'Position', [100 100 650 520]);
[X, Y, Z] = sphere(24);
surf(c(1) + P.obs_r*X, c(2) + P.obs_r*Y, c(3) + P.obs_r*Z, ...
     'FaceAlpha', 0.35, 'EdgeColor', 'none', 'FaceColor', [0.8 0.3 0.3]);
hold on; grid on; axis equal;
for k = 1:2
    plot3(L{k}.p(1,:), L{k}.p(2,:), L{k}.p(3,:), 'LineWidth', 2, 'Color', coul{k});
end
plot3(pA(1), pA(2), pA(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
plot3(pB(1), pB(2), pB(3), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Scenario 2 - trajets de l''effecteur et obstacle (depart vert, arrivee noir)');
legend([{'Obstacle'}, noms], 'Location', 'best'); view(35, 25);
saveas(gcf, fullfile(P.dossier_resultats, 'fig_s2_trajets_obstacle.png'));

% Figure 4 : distance a l'obstacle au cours du temps
figure('Name', 'S2 - distance obstacle', 'Position', [150 150 700 400]);
hold on; grid on;
for k = 1:2
    plot(t, L{k}.dist, 'LineWidth', 1.8, 'Color', coul{k});
end
plot([0 P.T], [0 0], 'k--', 'LineWidth', 1);
plot([0 P.T], [P.obs_marge P.obs_marge], ':', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.5);
xlabel('Temps [s]'); ylabel('Distance a la surface de l''obstacle [m]');
title('Scenario 2 - distance effecteur / obstacle (< 0 : collision)');
legend([noms, {'Contact (0)', 'Marge de securite'}], 'Location', 'best');
saveas(gcf, fullfile(P.dossier_resultats, 'fig_s2_distance_obstacle.png'));

% Figure 5 : vitesses articulaires des 6 axes
figure('Name', 'S2 - vitesses articulaires', 'Position', [200 100 700 650]);
etiq = arrayfun(@(j) sprintf('qd_%d', j), 1:6, 'UniformOutput', false);
for k = 1:2
    subplot(2,1,k); hold on; grid on;
    plot(t, L{k}.qd.', 'LineWidth', 1.3);
    xlabel('Temps [s]'); ylabel('Vitesse articulaire [rad/s]');
    title(['Scenario 2 - ' noms{k}]);
    if k == 1, legend(etiq, 'Location', 'eastoutside'); end
end
saveas(gcf, fullfile(P.dossier_resultats, 'fig_s2_vitesses_articulaires.png'));

R.noms = noms; R.t = t; R.L = L; R.I = I; R.c = c; R.info = info;
end