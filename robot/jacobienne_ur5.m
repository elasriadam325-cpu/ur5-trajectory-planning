function J = jacobienne_ur5(q, robot)
% Jacobienne geometrique 6x6 du UR5, exprimee dans le repere de base
% Lignes 1-3 : vitesse lineaire de l'effecteur ; lignes 4-6 : vitesse angulaire
DH = robot.DH;
q  = q(:);
T  = eye(4);
z  = zeros(3,7);
o  = zeros(3,7);
z(:,1) = [0; 0; 1];
for i = 1:6
    a = DH(i,1); alpha = DH(i,2); d = DH(i,3); th = q(i);
    ct = cos(th); st = sin(th); ca = cos(alpha); sa = sin(alpha);
    T = T * [ct, -st*ca,  st*sa, a*ct;
             st,  ct*ca, -ct*sa, a*st;
              0,     sa,     ca,    d;
              0,      0,      0,    1];
    z(:,i+1) = T(1:3,3);
    o(:,i+1) = T(1:3,4);
end
pe = o(:,7);
J = zeros(6,6);
for i = 1:6
    J(1:3,i) = cross(z(:,i), pe - o(:,i));
    J(4:6,i) = z(:,i);
end
end