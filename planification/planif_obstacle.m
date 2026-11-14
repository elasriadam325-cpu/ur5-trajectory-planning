function [C, info] = planif_obstacle(pA, pB, c, r, marge, P)
% Cherche le plus petit decalage de la courbe de Bezier qui evite la sphere
% C : point de controle de la courbe de Bezier quadratique
u = pB(:) - pA(:);
u = u / norm(u);
z = P.dir_contour(:);
n = z - (z.'*u)*u;                 % direction de decalage, perpendiculaire a A-B
n = n / norm(n);
M = (pA(:) + pB(:)) / 2;
s_ev = linspace(0, 1, 400);

trouve = false;
for dl = 0:P.contour_pas:P.contour_max
    C = M + 2*dl*n;                % le milieu de la courbe est decale de dl
    d = distance_sphere(bezier2(pA, C, pB, s_ev), c, r);
    if min(d) >= marge
        trouve = true;
        break;
    end
end
if ~trouve
    error('planif_obstacle:echec', 'Aucun contournement trouve (augmenter P.contour_max).');
end
info.decalage = dl;                % [m]
info.dist_min = min(d);            % [m]
end