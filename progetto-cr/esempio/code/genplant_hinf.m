function [Pgen, ymeas, zphys] = genplant_hinf(sys, Wz)
% =========================================================================
%  genplant_hinf.m
%  Costruzione del PLANT GENERALIZZATO per la sintesi Hinf (formulazione
%  "disturbo esplicito", Capitolo 4 del corso).
%
%  Plant sys: ingressi {u1c,u2c,w}; uscite {zsdd,dt,ds,dzu,u1,u2}.
%
%  Uscite di prestazione fisiche:  z_phys = [zsdd ; dt ; ds ; u1c ; u2c]
%  Pesi:   Wz = append(wc, wrh, wrs, wu1, wu2)   (5x5 diagonale)
%          z = Wz * z_phys
%
%  Ingressi esogeni:  xi = [w ; n1 ; n2 ; n3]   (disturbo stradale + rumori)
%  Controlli:         u  = [u1c ; u2c]
%  Misure:            y  = [zsdd ; ds ; dzu] + n
%
%  Ritorna Pgen (ss) con ingressi [w n1 n2 n3 u1c u2c] e uscite [z1..z5 y1..y3]
%          pronta per  hinfsyn(Pgen, 3, 2).
% =========================================================================
A = sys.A;  B = sys.B;  C = sys.C;
Bu = B(:,1:2);  Bw = B(:,3);
nx = size(A,1);

nz = 5;
% mappe fisiche
Cz   = C(1:3,:);            % zsdd,dt,ds
Mzp  = [Cz; zeros(2,nx)];   % z_phys = Mzp*x + Nzp*u
Nzp  = [0 0; 0 0; 0 0; 1 0; 0 1];

% misure: zsdd, ds, dzu  (=righe 1,3,4 di C)
Cmeas = C([1 3 4],:);  ny = 3;

% sistema di pesi (5x5)
[Aw,Bwq,Cw,Dwq] = ssdata(Wz);
nw = size(Aw,1);

A_aug = blkdiag(A, Aw);
B1 = [Bw, zeros(nx,3); zeros(nw,4)];     % xi = [w; n]
B2 = [Bu; Bwq*Nzp];                        % u
C1 = [Dwq*Mzp, Cw];                        % z = Wz*z_phys
D11 = zeros(nz,4);
D12 = Dwq*Nzp;
% Convenzione di segno: y = -(misura+n), cosi' che u = K*y realizzi il
% feedback negativo fisico u = -K*(misura+n). Il K restituito e' quello fisico.
C2 = [-Cmeas, zeros(ny,nw)];
D21 = [zeros(ny,1), -eye(ny)];
D22 = zeros(ny,2);

Pgen = ss(A_aug, [B1 B2], [C1;C2], [D11 D12; D21 D22]);
Pgen.InputName  = {'w','n1','n2','n3','u1c','u2c'};
Pgen.OutputName = {'z1','z2','z3','z4','z5','y1','y2','y3'};

ymeas = sys({'zsdd','ds','dzu'},{'u1c','u2c'});
zphys = {'zsdd','dt','ds','u1c','u2c'};
end
