%% design_musyn.m - Task 3: mu-synthesis (DK-iteration) + mu-analysis
clear; clc; close all;
p = params(); sys = model_quartercar(p);
[Wz, ~, g0] = weights_hinf(p);
[sysu, unc] = model_quartercar_unc(p);

% =========================== sintesi =====================================
fprintf('=== mu-synthesis (musyn, DK-iteration) ===\n');
rng(0);
Pgen_u = genplant_hinf(sysu, Wz);
[K_mu, CLperf, info] = musyn(Pgen_u, 3, 2);
K_mu = minreal(K_mu,1e-6);
K_mu.InputName={'y1','y2','y3'}; K_mu.OutputName={'u1c','u2c'};
[K_mu, m] = cl_select(K_mu, sys, p, 'musyn');
fprintf('K_mu ordine=%d  stab=%d  Ms=%.3f\n', order(K_mu), m.stable, m.Ms);
fprintf('  att zsdd: body=%.1f wheel=%.1f dB | att dt: body=%.1f wheel=%.1f dB\n', ...
        m.att_body, m.att_wheel, m.att_dt_body, m.att_dt_wheel);
mu_rp = min([info.PeakMu]);   % miglior mu(RP) raggiunto dalla DK
fprintf('  mu(RP) peak (DK, livello nominale) = %.3f\n', mu_rp);

% =========================== mu-analysis =================================
fprintf('\n=== mu-analysis a posteriori ===\n');
Tcl = lft(Pgen_u, K_mu);                 % uss: [w;n] -> z pesato
try
  [wc,~] = wcgain(Tcl(:,1));
  fprintf('wcgain (w -> z pesato): [%.2f, %.2f]\n', wc.LowerBound, wc.UpperBound);
catch e, fprintf('wcgain err: %s\n', e.message); end

% ---- robusta stabilita': margine via campionamento dell'incertezza ------
fprintf('\n=== robusta stabilita'' (verifica) ===\n');
try
  % il soddisfacimento richiederebbe mu_strutturata; qui si verifica che il
  % sistema resti stabile su tutto il set campionato (validazione numerica).
  rng(2); Ns=40; maxRe = -inf;
  for k=1:Ns
    pp = p; d = @() 2*rand-1;   % delta uniforme in [-1,1]
    pp.ms  = p.ms *(1+p.unc.ms *d());
    pp.ks0 = p.ks0*(1+p.unc.ks0*d());
    pp.bs  = p.bs *(1+p.unc.bs *d());
    pp.kt  = p.kt *(1+p.unc.kt *d());
    pp.tau1= p.tau1*(1+p.unc.tau1*d());
    pp.tau2= p.tau2*(1+p.unc.tau2*d());
    sk = model_quartercar(pp);
    mk = cl_metrics(K_mu, sk, pp, false, 's');
    maxRe = max(maxRe, max(real(mk.poles)));
  end
  fprintf('max Re(poli) su %d impianti campionati = %+.2f  (stabile: %d)\n', ...
          Ns, maxRe, maxRe<0);
catch e, fprintf('validazione err: %s\n', e.message); end

save(fullfile('..','results','controllers_musyn.mat'), ...
     'K_mu','m','unc','info','mu_rp');
figure('Color','w');
o=sigmaoptions; o.FreqUnits='Hz'; o.Grid='on';
subplot(1,2,1); sigma(m.S,o,{2*pi*0.1,2*pi*100}); title('musyn: |S|');
subplot(1,2,2); sigma(m.T,o,{2*pi*0.1,2*pi*100}); title('musyn: |T|');
saveas(gcf, fullfile('..','results','fig','musyn.png'));
