function R = scenario3_singularite(P, robot)
% Scenario 3 : approche du bras tendu (singularite du coude), cartesien vs articulaire
t = 0:P.dt:P.T;
[pA, RA] = fk_ur5(P.qA, robot, P);
d = P.s3_dir(:) / norm(P.s3_dir);
lois  = {'Quintique', 'LSPB'};
s_loi = {traj_minjerk(0, 1, P.T, t), traj_lspb(0, 1, P.T, t, P.s3_acc_s)};

nL = numel(P.s3_L);
M = cell(nL, 2, 2);                 % M{iL, loi, espace} ; espace 1 = cartesien, 2 = articulaire
for iL = 1:nL
    pS = pA + P.s3_L(iL) * d;
    for k = 1:2
        s   = s_loi{k};
        q_c = traj_cartesienne(pA, (pA + pS)/2, pS, RA, P.T, t, P.qA, robot, P, s);
        q_a = P.qA + (q_c(:,end) - P.qA) * s;     % meme loi s, interpolation des angles
        M{iL,k,1} = metriques(q_c, P, robot);
        M{iL,k,2} = metriques(q_a, P, robot);
    end
end

% Tableaux (qd en rad/s, qdd en rad/s^2)
for k = 1:2
    fprintf('\nLoi %s (meme loi horaire dans les deux espaces)\n', lois{k});
    fprintf('%7s %10s %9s %9s | %8s %8s | %9s %9s\n', 'L[m]', 'q3_min', 'det_min', 'cond_max', ...
            'qd_cart', 'qd_art', 'qdd_cart', 'qdd_art');
    for iL = 1:nL
        c = M{iL,k,1}; a = M{iL,k,2};
        fprintf('%7.3f %10.3f %9.4f %9.1f | %8.3f %8.3f | %9.3f %9.3f\n', P.s3_L(iL), ...
                c.q3_min, c.det_min, c.cond_max, c.qd_max, a.qd_max, c.qdd_max, a.qdd_max);
    end
end

if ~exist(P.dossier_resultats, 'dir'), mkdir(P.dossier_resultats); end
[~, iref] = min(abs(P.s3_L - P.s3_L_ref));

% Figure 6 : determinant et conditionnement de la jacobienne (cas de reference, cartesien)
figure('Name', 'S3 - jacobienne', 'Position', [100 100 700 600]);
subplot(2,1,1); hold on; grid on;
for k = 1:2, plot(t, M{iref,k,1}.det, 'LineWidth', 1.6); end
ylabel('|det J| [-]'); legend(lois, 'Location', 'best');
title(sprintf('Scenario 3 - jacobienne le long du trajet de %.3f m vers le bras tendu', P.s3_L(iref)));
subplot(2,1,2); hold on; grid on;
for k = 1:2, plot(t, M{iref,k,1}.cond, 'LineWidth', 1.6); end
xlabel('Temps [s]'); ylabel('cond(J) [-]');
saveas(gcf, fullfile(P.dossier_resultats, 'fig_s3_jacobienne.png'));

% Figure 7 : pics de vitesse et d'acceleration en fonction de la longueur du trajet
QD = zeros(nL,2,2); QDD = zeros(nL,2,2);
for iL = 1:nL
    for k = 1:2
        for e = 1:2
            QD(iL,k,e)  = M{iL,k,e}.qd_max;
            QDD(iL,k,e) = M{iL,k,e}.qdd_max;
        end
    end
end
leg = {'Cartesien - Quintique', 'Articulaire - Quintique', 'Cartesien - LSPB', 'Articulaire - LSPB'};
sty = {'-o', '--o', '-s', '--s'};
cfg = [1 1; 1 2; 2 1; 2 2];
figure('Name', 'S3 - balayage', 'Position', [150 100 1000 450]);
subplot(1,2,1); hold on; grid on;
for j = 1:4, plot(P.s3_L, QD(:,cfg(j,1),cfg(j,2)), sty{j}, 'LineWidth', 1.5); end
plot([P.s3_L(1) P.s3_L_lim], [P.qd_max P.qd_max], 'k:', 'LineWidth', 1.5);
xlim([P.s3_L(1) P.s3_L_lim]);
xlabel('Longueur du trajet depuis A [m]'); ylabel('Vitesse articulaire max [rad/s]');
title('Pic de vitesse articulaire'); legend([leg, {'Limite'}], 'Location', 'northwest');
subplot(1,2,2); hold on; grid on;
for j = 1:4, semilogy(P.s3_L, QDD(:,cfg(j,1),cfg(j,2)), sty{j}, 'LineWidth', 1.5); end
set(gca, 'YScale', 'log'); xlim([P.s3_L(1) P.s3_L_lim]);
xlabel('Longueur du trajet depuis A [m]'); ylabel('Acceleration articulaire max [rad/s^2]');
title('Pic d''acceleration articulaire');
saveas(gcf, fullfile(P.dossier_resultats, 'fig_s3_balayage.png'));

% Figure 8 : vitesses des 6 axes (cas de reference, cartesien)
figure('Name', 'S3 - vitesses', 'Position', [200 100 700 650]);
etiq = arrayfun(@(j) sprintf('qd_%d', j), 1:6, 'UniformOutput', false);
for k = 1:2
    subplot(2,1,k); hold on; grid on;
    plot(t, M{iref,k,1}.qd.', 'LineWidth', 1.3);
    xlabel('Temps [s]'); ylabel('Vitesse articulaire [rad/s]');
    title(sprintf('Scenario 3 - %s, cartesien, trajet de %.3f m', lois{k}, P.s3_L(iref)));
    if k == 1, legend(etiq, 'Location', 'eastoutside'); end
end
saveas(gcf, fullfile(P.dossier_resultats, 'fig_s3_vitesses.png'));

R.lois = lois; R.t = t; R.M = M;
end

function m = metriques(q, P, robot)
% Vitesses, accelerations et indicateurs de singularite d'une trajectoire articulaire q (6xN)
N  = size(q, 2);
qd = gradient(q, P.dt);
qdd = gradient(qd, P.dt);
dets = zeros(1, N); conds = zeros(1, N);
for k = 1:N
    J = jacobienne_ur5(q(:,k), robot);
    dets(k)  = abs(det(J));
    conds(k) = cond(J);
end
m.qd = qd;
m.qd_max   = max(abs(qd(:)));
m.qdd_max  = max(abs(qdd(:)));
m.det      = dets;      m.det_min  = min(dets);
m.cond     = conds;     m.cond_max = max(conds);
m.q3_min   = min(abs(q(3,:)));
end