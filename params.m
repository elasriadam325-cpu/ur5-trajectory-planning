function P = params()
% Tous les parametres du projet (aucune valeur en dur ailleurs)

% Robot UR5 (Robotics System Toolbox)
P.robot_nom = 'universalUR5';
P.ee_name   = 'tool0';              % corps de l'effecteur
P.gravite   = [0 0 -9.81];          % [m/s^2]

% Limites articulaires (6 axes)
P.qd_max  = pi;                     % vitesse max [rad/s]
P.qdd_max = 8;                      % acceleration max [rad/s^2]

% Mouvement du scenario 1 (sans obstacle), espace articulaire
P.T  = 2.0;                         % duree [s]
P.dt = 0.01;                        % pas de temps [s]
P.qA = [0; -pi/2;    pi/2;   -pi/2; -pi/2; 0];   % configuration de depart [rad]
P.qB = [pi/2; -2*pi/3; 2*pi/3; -pi/2; -pi/2; 0]; % configuration d'arrivee [rad]

% Cinematique inverse numerique
P.ik_poids   = [1 1 1 1 1 1];       % [orientation(3) position(3)]
P.ik_perturb = 0.1;                 % perturbation de l'estimation initiale (test) [rad]
P.tol_ik_pos = 1e-4;                % tolerance erreur de position [m]
P.tol_ik_rot = 1e-3;                % tolerance erreur d'orientation (norme de Frobenius)

% Sorties
P.dossier_resultats = 'resultats';
end