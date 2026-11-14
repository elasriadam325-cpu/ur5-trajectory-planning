function robot = charger_robot(P)
% Charge le modele geometrique du robot UR5
% 100% pur MATLAB - Ne necessite AUCUNE toolbox (remplace loadrobot)

robot.nom = P.robot_nom;
robot.ee_name = P.ee_name;
robot.gravite = P.gravite;

% Parametres standard Denavit-Hartenberg du UR5 : [a_i, alpha_i, d_i]
robot.DH = [
    0,         pi/2,   0.089159;  % Axe 1 (Base -> Epaule)
   -0.42500,   0,      0;         % Axe 2 (Epaule -> Coude)
   -0.39225,   0,      0;         % Axe 3 (Coude -> Poignet 1)
    0,         pi/2,   0.10915;   % Axe 4 (Poignet 1 -> Poignet 2)
    0,        -pi/2,   0.09465;   % Axe 5 (Poignet 2 -> Poignet 3)
    0,         0,      0.08230    % Axe 6 (Poignet 3 -> Effecteur)
];

robot.BodyNames = {'base', 'shoulder_link', 'upper_arm_link', 'forearm_link', ...
                   'wrist_1_link', 'wrist_2_link', 'wrist_3_link', P.ee_name};
end
