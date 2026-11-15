% Validation de la jacobienne : J*dq doit egaler la variation de position de l'effecteur
P = params();
robot = charger_robot(P);
rng(0);
for k = 1:5
    q  = -1 + 2*rand(6,1);
    dq = -1 + 2*rand(6,1);
    h  = 1e-7;
    p0 = fk_ur5(q, robot, P);
    p1 = fk_ur5(q + h*dq, robot, P);
    J  = jacobienne_ur5(q, robot);
    err = norm(J(1:3,:)*dq - (p1 - p0)/h);
    fprintf('Test %d : erreur sur la vitesse lineaire = %.2e\n', k, err);
    assert(err < 1e-5, 'Jacobienne incorrecte');
end
J = jacobienne_ur5(P.qA, robot);
fprintf('Configuration A : |det J| = %.4f, cond(J) = %.2f\n', abs(det(J)), cond(J));
disp('Jacobienne validee.');