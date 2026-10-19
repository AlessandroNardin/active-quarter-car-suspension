function [sys, A, Bu, Bw, C, D] = model_quartercar(p, opts)
% =========================================================================
%  model_quartercar.m
%  Modello quarter-car attivo linearizzato attorno all'equilibrio
%  (il termine di Duffing alpha*x1^3 si annulla in x=0).
%
%  Stato (9): x = [ds; zsdot; dt; zudot; u1; u1dot; p1; u2; p2]
%     x1 = ds     escursione sospensione          zs - zu
%     x2 = zsdot  velocita' massa sospesa
%     x3 = dt     deformazione pneumatico         zu - zr
%     x4 = zudot  velocita' massa non sospesa
%     x5 = u1     forza attuatore primario
%     x6 = u1dot  derivata della forza u1
%     x7 = p1     stato del Pade' del ritardo tau1
%     x8 = u2     forza attuatore secondario
%     x9 = p2     stato del Pade' del ritardo tau2
%
%  Ingressi:  u = [u1c; u2c]  (comandi attuatori),  w (disturbo = zrdot)
%  Uscite (C, in quest'ordine):
%     'zsdd'  accelerazione massa sospesa (comfort)
%     'dt'    deformazione pneumatico     (road-holding)
%     'ds'    escursione sospensione      (rattle space)
%     'dzu'   velocita' massa non sospesa (misura)
%     'u1'    forza attuatore primario    (sforzo)
%     'u2'    forza attuatore secondario  (sforzo)
%
%  Uso:  sys = model_quartercar(params)
% =========================================================================
if nargin < 2, opts = struct(); end

ms=p.ms; mu=p.mu; ks0=p.ks0; bs=p.bs; kt=p.kt; bt=p.bt;
wn1=p.wn1; z1=p.zeta1; t1=p.tau1; ta=p.taua; t2=p.tau2;

nx = 9; nu = 2; nw = 1;

% ---- Matrice di stato A --------------------------------------------------
A = zeros(nx);
% x1dot = x2 - x4
A(1,2) = 1;  A(1,4) = -1;
% x2dot = (-(ks0*x1 + bs*(x2-x4)) + x5)/ms
A(2,1) = -ks0/ms;  A(2,2) = -bs/ms;  A(2,4) = bs/ms;  A(2,5) = 1/ms;
% x3dot = x4 - w
A(3,4) = 1;
% x4dot = ((ks0*x1 + bs*(x2-x4)) - (kt*x3 + bt*(x4-w)) - x5 + x8)/mu
A(4,1) = ks0/mu;  A(4,2) = bs/mu;  A(4,3) = -kt/mu;
A(4,4) = -(bs+bt)/mu;  A(4,5) = -1/mu;  A(4,8) = 1/mu;
% x5dot = x6
A(5,6) = 1;
% x6dot = -wn1^2*x5 - 2*z1*wn1*x6 + wn1^2*(2*x7 - u1c)
A(6,5) = -wn1^2;  A(6,6) = -2*z1*wn1;  A(6,7) = 2*wn1^2;
% x7dot = -(2/t1)*x7 + (2/t1)*u1c
A(7,7) = -2/t1;
% x8dot = -(1/ta)*x8 + (1/ta)*(2*x9 - u2c)
A(8,8) = -1/ta;  A(8,9) = 2/ta;
% x9dot = -(2/t2)*x9 + (2/t2)*u2c
A(9,9) = -2/t2;

% ---- Ingresso di controllo Bu = [Bu1 Bu2] -------------------------------
Bu = zeros(nx,nu);
Bu(6,1) = -wn1^2;      % u1c
Bu(7,1) =  2/t1;
Bu(8,2) = -1/ta;       % u2c
Bu(9,2) =  2/t2;

% ---- Ingresso di disturbo Bw (w = zrdot) --------------------------------
Bw = zeros(nx,nw);
Bw(3) = -1;
Bw(4) =  bt/mu;

% ---- Matrice di uscita C ------------------------------------------------
% righe: zsdd, dt, ds, dzu, u1, u2
C = zeros(6,nx);
C(1,1) = -ks0/ms; C(1,2) = -bs/ms; C(1,4) = bs/ms; C(1,5) = 1/ms; % zsddot
C(2,3) = 1;                                                          % dt = x3
C(3,1) = 1;                                                          % ds = x1
C(4,4) = 1;                                                          % dzu = x4
C(5,5) = 1;                                                          % u1 = x5
C(6,8) = 1;                                                          % u2 = x8

D = zeros(6, nu+nw);   % nessun feedthrough diretto

% ---- Costruzione oggetto ss ----------------------------------------------
sys = ss(A, [Bu Bw], C, D, ...
    'InputName',  {'u1c','u2c','w'}, ...
    'OutputName', {'zsdd','dt','ds','dzu','u1','u2'}, ...
    'StateName',  {'ds','zsdot','dt','zudot','u1','u1dot','p1','u2','p2'});
end
