function [q, qd, qdd] = traj_quintique(q0, qf, T, t, v0, vf, a0, af)
% Polynome de degre 5 avec conditions aux limites (position, vitesse, acceleration)
% Par defaut v0 = vf = a0 = af = 0 (repos-repos)
n = numel(q0); q0 = q0(:); qf = qf(:);
if nargin < 5 || isempty(v0), v0 = zeros(n,1); end
if nargin < 6 || isempty(vf), vf = zeros(n,1); end
if nargin < 7 || isempty(a0), a0 = zeros(n,1); end
if nargin < 8 || isempty(af), af = zeros(n,1); end
v0 = v0(:); vf = vf(:); a0 = a0(:); af = af(:);

M = [1 0 0   0     0      0;
     0 1 0   0     0      0;
     0 0 2   0     0      0;
     1 T T^2 T^3   T^4    T^5;
     0 1 2*T 3*T^2 4*T^3  5*T^4;
     0 0 2   6*T   12*T^2 20*T^3];
B = [q0, v0, a0, qf, vf, af].';        % 6 x n
C = M \ B;                              % 6 x n

tc = min(max(t(:).', 0), T);
z  = zeros(size(tc)); o = ones(size(tc));
q   = C.' * [o; tc; tc.^2; tc.^3; tc.^4; tc.^5];
qd  = C.' * [z; o; 2*tc; 3*tc.^2; 4*tc.^3; 5*tc.^4];
qdd = C.' * [z; z; 2*o; 6*tc; 12*tc.^2; 20*tc.^3];
end