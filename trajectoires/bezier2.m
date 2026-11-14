function p = bezier2(pA, C, pB, s)
% Courbe de Bezier quadratique 3D. pA, C, pB : 3x1 ; s : 1xN dans [0,1]
s = s(:).';
p = pA(:)*((1-s).^2) + C(:)*(2*s.*(1-s)) + pB(:)*(s.^2);
end