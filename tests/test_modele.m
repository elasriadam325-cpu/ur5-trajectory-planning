% Validation du modele UR5 : MGD puis MGI doivent redonner la meme pose
P = params();
robot = charger_robot(P);

pts = {P.qA, P.qB};
for k = 1:numel(pts)
    q = pts{k};
    [p, R] = fk_ur5(q, robot, P);
    T = [R, p; 0 0 0 1];
    [qs, ok] = ik_ur5(T, q + P.ik_perturb, robot, P);
    [p2, R2] = fk_ur5(qs, robot, P);
    err_pos = norm(p2 - p);
    err_rot = norm(R2 - R, 'fro');
    fprintf('Config %d : succes IK = %d | erreur position = %.2e m | erreur orientation = %.2e\n', ...
            k, ok, err_pos, err_rot);
    assert(ok, 'IK : pas de solution');
    assert(err_pos < P.tol_ik_pos, 'IK : erreur de position trop grande');
    assert(err_rot < P.tol_ik_rot, 'IK : erreur d''orientation trop grande');
end
disp('Modele UR5 valide.');