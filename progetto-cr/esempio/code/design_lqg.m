%% design_lqg.m - Task 1: sintesi LQG (con e senza integratore)
%  Il progetto e' fatto in COORDINATE NORMALIZZATE (stati, ingressi, uscite
%  scalati alle loro grandezze tipiche) per evitare il mal-condizionamento
%  dovuto all'enorme disparita' di scala tra i modi del sistema.
clear; clc; close all;
p = params(); sys = model_quartercar(p);
A=sys.A; B=sys.B; C=sys.C;
Bu=B(:,1:2); Bw=B(:,3);
Cmeas=C([1 3 4],:);     % [zsdd; ds; dzu]
Cz3  = C(1:3,:);        % [zsdd; dt; ds]  (uscite di prestazione)

% ---- fattori di scala ----------------------------------------------------
sx = [1e-2; 1e-1; 1e-3; 1e-1; 1e3; 1e5; 1e3; 1e3; 1e3];  % stati
Xs = diag(sx);
Us = diag([1e3; 1e3]);      % comandi (N)
Zs = diag([1; 1e-3; 1e-2]); % uscite prestazione (zsdd,dt,ds)
Ms = diag([1; 1e-3; 1e-1]); % misure (zsdd,ds,dzu)

% ---- sistema normalizzato ------------------------------------------------
An  = Xs\A*Xs;
Bn  = Xs\Bu*Us;
Bwn = Xs\Bw;
Cn  = Zs\Cz3*Xs;      % z_n = Cn * x_n
Cmn = Ms\Cmeas*Xs;    % y_n = Cmn * x_n

% ---- pesi LQR (su uscite/ingressi normalizzati) --------------------------
Qn = Cn'*Cn;                 % Qy_n = I  (comfort, road-holding, rattle space)
rho = 1e2;                   % peso dello sforzo (compromesso Ms vs prestazione)
Kxn = lqr(An, Bn, Qn, rho*eye(2));

% ---- covarianze rumori (Kalman) ------------------------------------------
qwn = 1;                     % varianza di w (disturbo bianco unitario)
Rnn = diag([1e-2, 1, 1e-2]); % rumori di misura normalizzati (zsdd,ds,dzu)
Ln  = lqe(An, Bwn, Cmn, qwn, Rnn);

fprintf('LQR |Kxn|=%.3f  ->  |Kx_fis|=%.3e\n', norm(Kxn), norm(Us*Kxn/Xs));
fprintf('Kalman |Ln|=%.3f, poli osservatore max Re=%.3f\n', norm(Ln), max(real(eig(An-Ln*Cmn))));

% ---- LQG senza integratore ----------------------------------------------
Kn = ss(An - Bn*Kxn - Ln*Cmn, Ln, -Kxn, 0);
K_lqg = minreal(-Us*Kn*inv(Ms));   % segno per la convenzione u = -K*y
K_lqg.InputName={'y1','y2','y3'}; K_lqg.OutputName={'u1c','u2c'};

% ---- LQG con integratore su ds ------------------------------------------
Cseln = Cmn(2,:);                      % ds normalizzato
Aaug = [An, zeros(9,1); Cseln, 0];
Baug = [Bn; zeros(1,2)];
Qaug = blkdiag(Qn, 1e2);
Kaug = lqr(Aaug, Baug, Qaug, rho*eye(2));
Kxa = Kaug(:,1:9);  Kia = Kaug(:,10);
Akn_i = [An - Bn*Kxa - Ln*Cmn, -Bn*Kia; zeros(1,9), 0];
Bkn_i = [Ln; 0 1 0];
Ckn_i = [-Kxa, -Kia];
K_lqg_i = minreal(-Us*ss(Akn_i,Bkn_i,Ckn_i,0)*inv(Ms));
K_lqg_i.InputName={'y1','y2','y3'}; K_lqg_i.OutputName={'u1c','u2c'};

% ---- metriche ------------------------------------------------------------
mA = cl_metrics(K_lqg,   sys, p, true,  'LQG');
mB = cl_metrics(K_lqg_i, sys, p, true,  'LQG-integratore');
printmet(mA,'LQG senza integratore');
printmet(mB,'LQG con integratore');

% ---- errore a regime -----------------------------------------------------
t=0:0.02:20;
y1 = squeeze(step(mA.CL(:,1),t));
y2 = squeeze(step(mB.CL(:,1),t));
fprintf('\nA regime (t=20s) su ds: senza int=%.3e, con int=%.3e\n', y1(3,end), y2(3,end));
fprintf('A regime (t=20s) su zsdd: senza int=%.3e, con int=%.3e\n', y1(1,end), y2(1,end));

save(fullfile('..','results','controllers_lqg.mat'), 'K_lqg','K_lqg_i','Kxn','Ln','Xs','Us','Zs','Ms','Rnn','qwn');

figure('Color','w');
subplot(1,2,1); step(mA.CL(:,1),t); title('w -> z_{phys} (LQG)'); grid on;
subplot(1,2,2); step(mB.CL(:,1),t); title('w -> z_{phys} (LQG+int)'); grid on;
saveas(gcf, fullfile('..','results','fig','lqg_step.png'));

function printmet(m,name)
fprintf('  [%s] ||S||inf=%.3f ||T||inf=%.3f Ms=%.3f stab=%d | att zsdd body=%.1f wheel=%.1f | att dt body=%.1f wheel=%.1f\n', ...
        name, m.gammaS, m.gammaT, m.Ms, m.stable, m.att_body, m.att_wheel, m.att_dt_body, m.att_dt_wheel);
end
