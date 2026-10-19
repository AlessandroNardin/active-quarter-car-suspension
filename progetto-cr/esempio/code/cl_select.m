function [K, m] = cl_select(K, sys, p, tag)
% =========================================================================
%  cl_select.m
%  Sceglie il segno del controllore K (u = -K*y oppure u = +K*y) che rende
%  l'anello chiuso STABILE e con il picco di sensibilita' minore. Utile per
%  uniformare le diverse convenzioni di segno di hinfsyn / mixsyn / hinfstruct.
% =========================================================================
if nargin<4, tag=''; end
mA = cl_metrics(K,  sys, p, false, tag);
mB = cl_metrics(-K, sys, p, false, tag);
okA = mA.stable; okB = mB.stable;
if okA && okB
    if mB.Ms < mA.Ms, K=-K; m=mB; else, m=mA; end
elseif okB
    K=-K; m=mB;
else
    m=mA;
end
end
