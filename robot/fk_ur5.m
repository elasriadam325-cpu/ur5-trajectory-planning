function [p, R, T] = fk_ur5(q, robot, P)
% MGD du UR5 : q (6x1) [rad] -> p (3x1) [m], R (3x3) orientation de l'effecteur
% 100% pur MATLAB - Ne necessite AUCUNE toolbox (remplace getTransform)

q = q(:);

if nargin < 2 || isempty(robot) || ~isfield(robot, 'DH')
    DH = [
        0,         pi/2,   0.089159;
       -0.42500,   0,      0;
       -0.39225,   0,      0;
        0,         pi/2,   0.10915;
        0,        -pi/2,   0.09465;
        0,         0,      0.08230
    ];
else
    DH = robot.DH;
end

T = eye(4);
for i = 1:6
    a     = DH(i, 1);
    alpha = DH(i, 2);
    d     = DH(i, 3);
    theta = q(i);
    
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    
    Ti = [ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1];
    T = T * Ti;
end

p = T(1:3, 4);
R = T(1:3, 1:3);
end
