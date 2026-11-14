# UR5 Trajectory Planning

Planification de trajectoire pour le robot manipulateur UR5 sous MATLAB.

**Thème** : planification de trajectoire pour robots manipulateurs.


## Problématique

Comment choisir la loi de mouvement et l'espace de planification (articulaire ou cartésien) pour un déplacement point à point d'un robot UR5, avec ou sans obstacle, en comparant durée, fluidité, précision géométrique et sécurité ?

## Prérequis

- **MATLAB** : version <R20XX a/b> (à compléter avec la sortie de la commande `version`)
- **Toolboxes** : **aucune**. Le modèle du UR5 (MGD par paramètres de Denavit-Hartenberg) et la MGI numérique sont écrits en MATLAB pur.
- Système : Windows, Linux ou macOS (chemins relatifs uniquement)

## Lancer la démonstration

1. Cloner le dépôt : `git clone <url_du_depot>`
2. Ouvrir MATLAB et se placer dans le dossier du projet.
3. Exécuter dans la fenêtre de commandes :

```matlab
main
```

`main.m` reproduit **toutes les figures du diaporama**, affiche les tableaux d'indicateurs et enregistre les figures dans `resultats/`.

Tests de validation (facultatifs) :

```matlab
test_trajectoires   % lois de mouvement : conditions aux limites et vitesses max théoriques
test_modele         % MGD puis MGI : la pose cible doit être retrouvée
```

## Structure du dépôt

```
.
├── main.m                          Point d'entrée : lance les scénarios, produit les figures
├── params.m                        Tous les paramètres du projet (aucune valeur en dur ailleurs)
├── robot/
│   ├── charger_robot.m             Paramètres DH du UR5
│   ├── fk_ur5.m                    MGD : q (6x1) -> position et orientation de l'effecteur
│   └── ik_ur5.m                    MGI numérique (Damped Least Squares)
├── trajectoires/
│   ├── traj_cubique.m              Polynôme de degré 3 (repos-repos)
│   ├── traj_quintique.m            Polynôme de degré 5 (position, vitesse, accélération imposées)
│   ├── traj_minjerk.m              Minimum jerk (= quintique en repos-repos)
│   ├── traj_lspb.m                 Profil trapézoïdal (LSPB)
│   ├── bezier2.m                   Courbe de Bézier quadratique 3D
│   └── traj_cartesienne.m          Chemin cartésien + MGI à chaque pas, orientation constante
├── planification/
│   ├── distance_sphere.m           Distance signée effecteur / obstacle sphérique
│   └── planif_obstacle.m           Contournement : décalage minimal de la courbe de Bézier
├── analyse/
│   └── indicateurs_traj.m          Durée, vitesses, accélérations, jerk, longueur, écart à la droite
├── scenarios/
│   ├── scenario1_sans_obstacle.m   Comparaison des lois en espace articulaire
│   ├── scenario2_obstacle.m        Trajet cartésien avec obstacle sphérique
│   └── scenario3_singularite.m     Proximité d'une singularité (à venir)
├── tests/
│   ├── test_modele.m
│   └── test_trajectoires.m
└── resultats/                      Figures PNG générées par main.m
```

## Scénarios

Les trois scénarios partent de la même configuration de départ qA et visent le même point d'arrivée qB, en 2 s. Chacun répond à une question différente.

### Scénario 1 : sans obstacle, espace articulaire
- **Question** : quelle loi de mouvement choisir ?
- **Méthode** : interpolation des 6 angles avec la cubique, la quintique (minimum jerk) et le LSPB.
- **Indicateurs** : vitesse, accélération et jerk maximaux, longueur du trajet, écart à la droite A-B.
- **Constat** : la quintique est la plus douce, le LSPB respecte directement la limite d'accélération et a le pic de vitesse le plus bas. L'effecteur décrit un arc qui s'écarte de 0.148 m de la droite A-B.
- **Fichier** : `scenarios/scenario1_sans_obstacle.m`

### Scénario 2 : avec obstacle, espace cartésien
- **Question** : comment suivre un chemin imposé et éviter un obstacle sphérique, et à quel coût ?
- **Méthode** : chemin de Bézier quadratique parcouru avec la loi minimum jerk, orientation de l'outil constante, MGI à chaque pas. Le décalage de la courbe est le plus petit qui respecte la marge de sécurité.
- **Indicateurs** : distance minimale à l'obstacle, longueur du trajet, vitesses articulaires, collision oui ou non.
- **Constat** : la ligne droite traverse l'obstacle (distance minimale -0.050 m). Le contournement respecte la marge (0.030 m) pour +4 % de longueur.
- **Fichier** : `scenarios/scenario2_obstacle.m`

### Scénario 3 : proximité d'une singularité
- **Question** : que se passe-t-il quand le bras approche d'une configuration singulière (déterminant de la jacobienne proche de 0) ?
- **Méthode** : trajet cartésien vers une cible proche de la singularité, comparé à un trajet en espace articulaire.
- **Indicateurs** : |det J| et conditionnement de la jacobienne le long du trajet, vitesses articulaires maximales, convergence de la MGI.
- **Constat attendu** : en cartésien, les vitesses articulaires augmentent fortement près de la singularité ; l'espace articulaire reste sûr. <à compléter avec les résultats>
- **Fichier** : `scenarios/scenario3_singularite.m` (à venir)

## Méthodes

- **Lois de mouvement** : cubique s(τ) = 3τ² − 2τ³ ; quintique / minimum jerk s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ ; LSPB (accélération constante, croisière, décélération). Avec τ = t/T et q = q0 + (qf − q0)·s(τ).
- **Modèle du UR5** : convention de Denavit-Hartenberg, 6 articulations rotoïdes.
- **MGI** : méthode itérative Damped Least Squares (Levenberg-Marquardt) avec jacobienne numérique 6x6, initialisée par la configuration du pas précédent.
- **Évitement d'obstacle** : chemin de Bézier quadratique, décalage minimal pour respecter une marge de sécurité autour de l'obstacle sphérique, parcouru avec la loi minimum jerk.
- **Indicateurs** : durée, vitesse / accélération / jerk maximaux, intégrale du jerk², longueur du trajet de l'effecteur, écart maximal à la droite A-B, distance minimale à l'obstacle.

## Résultats principaux

**Scénario 1** (déplacement en 2 s, articulation 1 de 90°) :

| Loi | qd_max [rad/s] | qdd_max [rad/s²] | jerk_max [rad/s³] |
|---|---|---|---|
| Cubique | 1.178 | 2.356 | ≈ 236 |
| Quintique / min-jerk | 1.473 | 2.267 | 11.6 |
| LSPB | 0.828 | 8.000 | 800 |

**Scénario 2** :

| Trajet | Longueur [m] | Distance min à l'obstacle [m] | Collision |
|---|---|---|---|
| Ligne droite | 0.6211 | -0.050 | Oui |
| Contournement | 0.6476 | 0.030 | Non |

**Scénario 3** : <à compléter>

## Synthèse générale

| Critère | Scénario 1 (articulaire) | Scénario 2 (cartésien) | Scénario 3 (singularité) |
|---|---|---|---|
| Forme du trajet de l'effecteur | Arc (écart 0.148 m) | Courbe imposée | <à compléter> |
| Vitesse articulaire max | 1.18 à 1.47 rad/s | 2.2 rad/s | <à compléter> |
| Gestion d'obstacle | Non | Oui | Non |
| Risque principal | Trajet non rectiligne | Vitesses articulaires élevées | Vitesses qui explosent |

**Conclusions générales**
1. **Aucune méthode n'est la meilleure** : le choix dépend du compromis entre rapidité, douceur, précision géométrique et sécurité.
2. **L'espace articulaire** est simple et sûr (pas de MGI à chaque pas, pas de singularité sur le chemin), mais l'effecteur ne suit pas une droite.
3. **L'espace cartésien** impose le trajet de l'effecteur, mais demande des vitesses articulaires plus élevées et dépend de la MGI.
4. **La quintique (minimum jerk)** est la loi la plus douce ; le LSPB est préférable quand une limite d'accélération doit être respectée strictement.

## Limites

- Le LSPB multi-axes n'est pas parfaitement synchronisé : chaque articulation calcule sa propre durée d'accélération (longueur 0.7128 m contre 0.7121 m pour les autres lois).
- Seul le point de l'effecteur est testé contre l'obstacle, pas les segments du bras.
- Le contournement est géométrique (obstacle unique, sphérique et connu), pas un planificateur général (RRT, champs de potentiel).
- Les valeurs de jerk des lois à accélération discontinue (cubique, LSPB) dépendent du pas de temps `dt`.
- Les résultats chiffrés valent pour le UR5 et les configurations testées.

## Sources et références

- **Article de référence** : *Trajectory Planning for Robot Manipulators* (référence du projet 3).
- **Paramètres DH du UR5** : documentation technique de Universal Robots.
- **Assistant IA** : Claude (Anthropic) a été utilisé comme aide à la conception et à la rédaction du code (lois de trajectoire, MGD / MGI, scénarios). Le code a été relu, testé et est compris par tous les membres du groupe.
- Aucun autre code externe n'est utilisé. <Ajouter ici toute autre source : exemples MathWorks, dépôts GitHub, etc.>