function dx = quartercar_nl(x, u, p)
% =========================================================================
%  quartercar_nl.m
%  Dinamica NON LINEARE completa del quarter-car attivo (con Duffing).
%
%  Ingressi:
%     x : stato 9x1 = [ds; zsdot; dt; zudot; u1; u1dot; p1; u2; p2]
%     u : [u1c; u2c; w]   (comandi attuatori + disturbo w = zrdot)
%     p : struct parametri (params.m)
%
%  Serve per: simulazioni tempo (ode45) e determinazione della RAS
%  (regione di stabilita' asintotica) sul modello non lineare.
% =========================================================================
ms=p.ms; mu=p.mu; ks0=p.ks0; al=p.alpha; bs=p.bs; kt=p.kt; bt=p.bt;
wn1=p.wn1; z1=p.zeta1; t1=p.tau1; ta=p.taua; t2=p.tau2;

ds=x(1); zsdot=x(2); dt=x(3); zudot=x(4); u1=x(5); u1dot=x(6); p1=x(7);
u2=x(8); p2=x(9);

u1c=u(1); u2c=u(2); w=u(3);

Fs = ks0*ds + al*ds^3 + bs*(zsdot - zudot);   % forza sospensione (Duffing)
Ft = kt*dt + bt*(zudot - w);                  % forza pneumatico

dx = zeros(9,1);
dx(1) = zsdot - zudot;
dx(2) = (-Fs + u1)/ms;
dx(3) = zudot - w;
dx(4) = (Fs - Ft - u1 + u2)/mu;
dx(5) = u1dot;
dx(6) = -wn1^2*u1 - 2*z1*wn1*u1dot + wn1^2*(2*p1 - u1c);
dx(7) = -(2/t1)*p1 + (2/t1)*u1c;
dx(8) = -(1/ta)*u2 + (1/ta)*(2*p2 - u2c);
dx(9) = -(2/t2)*p2 + (2/t2)*u2c;
end
