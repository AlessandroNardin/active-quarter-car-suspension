function [sysu, unc] = model_quartercar_unc(p)
% =========================================================================
%  model_quartercar_unc.m
%  Modello quarter-car INCERTO (uss) con incertezza parametrica su
%  ms, ks0, bs, kt, tau1, tau2  (cfr. selezione delle incertezze).
%  I parametri sono espressi come  p = p0*(1 + r*delta), |delta|<=1.
%
%  Ritorna sysu (uss, ingressi {u1c,u2c,w}, uscite {zsdd,dt,ds,dzu,u1,u2})
%  e la struct `unc` con gli oggetti ureal (per l'analisi a posteriori).
% =========================================================================
unc.ms   = ureal('ms',  p.ms,  'Percentage', 100*p.unc.ms);
unc.ks0  = ureal('ks0', p.ks0, 'Percentage', 100*p.unc.ks0);
unc.bs   = ureal('bs',  p.bs,  'Percentage', 100*p.unc.bs);
unc.kt   = ureal('kt',  p.kt,  'Percentage', 100*p.unc.kt);
unc.tau1 = ureal('tau1',p.tau1,'Percentage', 100*p.unc.tau1);
unc.tau2 = ureal('tau2',p.tau2,'Percentage', 100*p.unc.tau2);

ms=unc.ms; mu=p.mu; ks0=unc.ks0; bs=unc.bs; kt=unc.kt; bt=p.bt;
wn1=p.wn1; z1=p.zeta1; t1=unc.tau1; ta=p.taua; t2=unc.tau2;
nx=9;

A = umat(zeros(nx));   % umat: consente di assegnare coefficienti ureal
A(1,2)=1; A(1,4)=-1;
A(2,1)=-ks0/ms; A(2,2)=-bs/ms; A(2,4)=bs/ms; A(2,5)=1/ms;
A(3,4)=1;
A(4,1)=ks0/mu; A(4,2)=bs/mu; A(4,3)=-kt/mu; A(4,4)=-(bs+bt)/mu; A(4,5)=-1/mu; A(4,8)=1/mu;
A(5,6)=1;
A(6,5)=-wn1^2; A(6,6)=-2*z1*wn1; A(6,7)=2*wn1^2;
A(7,7)=-2/t1;
A(8,8)=-1/ta; A(8,9)=2/ta;
A(9,9)=-2/t2;

Bu=umat(zeros(nx,2)); Bu(6,1)=-wn1^2; Bu(7,1)=2/t1; Bu(8,2)=-1/ta; Bu(9,2)=2/t2;
Bw=umat(zeros(nx,1)); Bw(3)=-1; Bw(4)=bt/mu;

C=umat(zeros(6,nx));
C(1,1)=-ks0/ms; C(1,2)=-bs/ms; C(1,4)=bs/ms; C(1,5)=1/ms;
C(2,3)=1; C(3,1)=1; C(4,4)=1; C(5,5)=1; C(6,8)=1;

sysu = ss(A,[Bu Bw],C,zeros(6,3));
sysu.InputName={'u1c','u2c','w'};
sysu.OutputName={'zsdd','dt','ds','dzu','u1','u2'};
end
