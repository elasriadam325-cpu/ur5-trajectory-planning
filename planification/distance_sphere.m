function d = distance_sphere(p, c, r)
% Distance signee de points p (3xN) a la surface d'une sphere (centre c, rayon r)
% d < 0 : le point est a l'interieur de l'obstacle (collision)
d = sqrt(sum((p - c(:)).^2, 1)) - r;
end