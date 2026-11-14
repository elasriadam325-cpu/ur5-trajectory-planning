% Tests des lois de mouvement (valeurs theoriques, sans robot)
t = 0:0.001:1;

[q, qd] = traj_cubique(0, 1, 1, t);
assert(abs(q(end) - 1) < 1e-9 && abs(qd(1)) < 1e-9 && abs(qd(end)) < 1e-9);
assert(abs(max(qd) - 1.5) < 1e-3, 'Cubique : vitesse max attendue 1.5');

[q, qd] = traj_quintique(0, 1, 1, t);
assert(abs(q(end) - 1) < 1e-9 && abs(qd(1)) < 1e-9 && abs(qd(end)) < 1e-9);
assert(abs(max(qd) - 1.875) < 1e-3, 'Quintique : vitesse max attendue 1.875');

[q, qd] = traj_lspb(0, 1, 1, t, 8);
assert(abs(q(end) - 1) < 1e-6 && abs(qd(1)) < 1e-9);

disp('Lois de mouvement validees.');