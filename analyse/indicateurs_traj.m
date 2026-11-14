function I = indicateurs_traj(q, qd, qdd, dt, P, robot)
% Indicateurs d'une trajectoire articulaire du UR5
I.duree    = (size(q,2) - 1) * dt;          % [s]
I.qd_max   = max(abs(qd(:)));               % [rad/s]
I.qdd_max  = max(abs(qdd(:)));              % [rad/s^2]

% Jerk : l'acceleration vaut 0 avant le depart et apres l'arrivee,
% donc les sauts aux extremites sont comptes
n = size(qdd, 1);
jerk       = diff([zeros(n,1), qdd, zeros(n,1)], 1, 2) / dt;   % [rad/s^3]
I.jerk_max = max(abs(jerk(:)));             % [rad/s^3]
I.J2       = sum(jerk(:).^2) * dt;          % integrale du jerk^2

N = size(q,2);
p = zeros(3, N);
for k = 1:N
    p(:,k) = fk_ur5(q(:,k), robot, P);
end
I.p        = p;                                      % trace de l'effecteur [m]
I.longueur = sum(sqrt(sum(diff(p,1,2).^2, 1)));      % [m]

% Ecart maximal de l'effecteur a la droite A-B
u = p(:,end) - p(:,1);
L = norm(u);
if L > 0
    u = u / L;
    v = p - p(:,1);
    dev = v - u * (u.' * v);
    I.ecart_droite = max(sqrt(sum(dev.^2, 1)));      % [m]
else
    I.ecart_droite = 0;
end
end