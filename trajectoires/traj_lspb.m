function [q, qd, qdd] = traj_lspb(q0, qf, T, t, qdd_max)
% Profil trapezoidal (LSPB) de duree T avec acceleration de blend qdd_max
% qdd_max : scalaire ou n x 1 [rad/s^2]. Condition : qdd_max >= 4|d|/T^2
n = numel(q0); q0 = q0(:); qf = qf(:); d = qf - q0;
a = qdd_max(:) .* ones(n,1);
tc = min(max(t(:).', 0), T);
N = numel(tc);
q = zeros(n,N); qd = zeros(n,N); qdd = zeros(n,N);

for i = 1:n
    di = abs(d(i)); s = sign(d(i));
    disc = T^2 - 4*di/a(i);
    if disc < 0
        error('traj_lspb:infaisable', ...
              'Articulation %d : qdd_max trop faible (min %.3f rad/s^2).', i, 4*di/T^2);
    end
    tb = (T - sqrt(disc)) / 2;          % duree d'acceleration
    V  = a(i) * tb;                     % vitesse de croisiere

    m1 = tc < tb;
    m3 = tc > T - tb;
    m2 = ~m1 & ~m3;

    q(i,m1)   = q0(i) + s*0.5*a(i)*tc(m1).^2;
    qd(i,m1)  = s*a(i)*tc(m1);
    qdd(i,m1) = s*a(i);

    q(i,m2)   = q0(i) + s*(0.5*a(i)*tb^2 + V*(tc(m2) - tb));
    qd(i,m2)  = s*V;
    qdd(i,m2) = 0;

    tau = T - tc(m3);
    q(i,m3)   = qf(i) - s*0.5*a(i)*tau.^2;
    qd(i,m3)  = s*a(i)*tau;
    qdd(i,m3) = -s*a(i);
end
end