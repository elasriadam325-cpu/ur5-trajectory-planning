function R = scenario1_sans_obstacle(P, robot)
% Scenario 1 : mouvement qA -> qB sans obstacle, comparaison des 3 lois
t    = 0:P.dt:P.T;
noms = {'Cubique', 'Quintique / min-jerk', 'LSPB'};
L = cell(1,3);
[L{1}.q, L{1}.qd, L{1}.qdd] = traj_cubique(P.qA, P.qB, P.T, t);
[L{2}.q, L{2}.qd, L{2}.qdd] = traj_minjerk(P.qA, P.qB, P.T, t);
[L{3}.q, L{3}.qd, L{3}.qdd] = traj_lspb(P.qA, P.qB, P.T, t, P.qdd_max);

% Verifications
for k = 1:3
    assert(norm(L{k}.q(:,1)   - P.qA) < 1e-9, '%s : q(0) faux', noms{k});
    assert(norm(L{k}.q(:,end) - P.qB) < 1e-9, '%s : q(T) faux', noms{k});
    assert(max(abs(L{k}.qd(:))) <= P.qd_max,  '%s : qd_max depassee', noms{k});
end

% Indicateurs
I = cell(1,3);
for k = 1:3
    I{k} = indicateurs_traj(L{k}.q, L{k}.qd, L{k}.qdd, P.dt, P, robot);
end
fprintf('\n%-22s %8s %9s %11s %12s %12s %12s\n', 'Loi', 'qd_max', 'qdd_max', 'jerk_max', 'int jerk^2', 'longueur[m]', 'ecart[m]');
for k = 1:3
    fprintf('%-22s %8.3f %9.3f %11.1f %12.1f %12.4f %12.4f\n', noms{k}, ...
            I{k}.qd_max, I{k}.qdd_max, I{k}.jerk_max, I{k}.J2, I{k}.longueur, I{k}.ecart_droite);
end

if ~exist(P.dossier_resultats, 'dir'), mkdir(P.dossier_resultats); end

% Figure 1 : articulation 1 (rad, rad/s, rad/s^2)
figure('Name', 'S1 - lois de mouvement', 'Position', [100 100 700 800]);
etiq  = {'q_1 [rad]', 'qd_1 [rad/s]', 'qdd_1 [rad/s^2]'};
champ = {'q', 'qd', 'qdd'};
for r = 1:3
    subplot(3,1,r); hold on; grid on;
    for k = 1:3
        plot(t, L{k}.(champ{r})(1,:), 'LineWidth', 1.5);
    end
    xlabel('Temps [s]'); ylabel(etiq{r});
    if r == 1
        title('Scenario 1 - articulation 1 : comparaison des lois');
        legend(noms, 'Location', 'best');
    end
end
saveas(gcf, fullfile(P.dossier_resultats, 'fig_s1_lois_articulaire.png'));

% Figure 2 : trace 3D de l'effecteur (identique pour les 3 lois)
p = I{1}.p;
figure('Name', 'S1 - trace effecteur', 'Position', [150 150 600 500]);
plot3(p(1,:), p(2,:), p(3,:), 'LineWidth', 2); hold on; grid on; axis equal;
plot3(p(1,1),   p(2,1),   p(3,1),   'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(p(1,end), p(2,end), p(3,end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Scenario 1 - trajet de l''effecteur (depart vert, arrivee rouge)');
saveas(gcf, fullfile(P.dossier_resultats, 'fig_s1_trace_effecteur.png'));

R.noms = noms; R.t = t; R.L = L; R.I = I;
end