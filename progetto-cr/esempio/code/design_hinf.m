%% design_hinf.m - Task 2: sintesi Hinf (hinfsyn + mixsyn + hinfstruct)
clear; clc; close all;
p = params(); sys = model_quartercar(p);
[Wz, Wmix, g] = weights_hinf(p);
Pgen = genplant_hinf(sys, Wz);

% ---------- hinfsyn (disturbo esplicito) ---------------------------------
fprintf('=== hinfsyn ===\n');
[K_hin,~,gam_hin] = hinfsyn(Pgen,3,2,'Display','off');
K_hin = minreal(K_hin,1e-6);
[K_hin,~] = cl_select(K_hin, sys, p, 'hinfsyn');
fprintf('gamma = %.4f, ordine K = %d\n', gam_hin, order(K_hin));
mA = cl_metrics(K_hin, sys, p, true, 'hinfsyn'); printmet(mA,'hinfsyn');

% ---------- mixsyn (S/KS/T classico) -------------------------------------
P = sys({'zsdd','ds','dzu'},{'u1c','u2c'});
fprintf('\n=== mixsyn ===\n');
K_mix=[]; gam_mix=NaN; mB=[];
try
  [K_mix,~,gam_mix] = mixsyn(P, Wmix.W1, Wmix.W2, Wmix.W3);
  K_mix = minreal(K_mix,1e-6);
  [K_mix,~] = cl_select(K_mix, sys, p, 'mixsyn');
  fprintf('gamma = %.4f, ordine K = %d\n', gam_mix, order(K_mix));
  mB = cl_metrics(K_mix, sys, p, true, 'mixsyn'); printmet(mB,'mixsyn');
catch e
  fprintf('mixsyn errore: %s\n', e.message);
end

% ---------- hinfstruct ----------------------------------------------------
fprintf('\n=== hinfstruct (struttura fissata) ===\n');
K_pid=[]; gam_pid=NaN; mC=[];
opt = hinfstructOptions('Display','off','RandomStart',6,'MaxIter',80);
try
  Cg = tunableGain('Cg', zeros(2,3));
  Cg.InputName={'y1','y2','y3'}; Cg.OutputName={'u1c','u2c'};
  [Cgs, ggs] = hinfstruct(Pgen, Cg, opt);
  K_pid = minreal(ss(Cgs),1e-6); gam_pid=ggs;
  [K_pid,~] = cl_select(K_pid, sys, p, 'hinfstruct-static');
  fprintf('[statico 2x3] gamma = %.4f\n', ggs);
  mC = cl_metrics(K_pid, sys, p, true, 'hinfstruct-static'); printmet(mC,'hinfstruct-static');
catch e
  fprintf('statico errore: %s\n', e.message);
end
try
  C1 = tunablePID('C1','pid'); C2 = tunablePID('C2','pid');
  C0 = [C1,0,0;0,C2,0]; C0.InputName={'y1','y2','y3'}; C0.OutputName={'u1c','u2c'};
  [Cpid,gpid] = hinfstruct(Pgen, C0, opt);
  Kp = minreal(ss(Cpid),1e-6);
  [Kp,mP] = cl_select(Kp, sys, p, 'hinfstruct-PID');
  fprintf('[PID decentral.] gamma=%.4f stab=%d Ms=%.2f\n', gpid, mP.stable, mP.Ms);
catch e
  fprintf('[PID] non realizzabile: %s\n', e.message);
end

save(fullfile('..','results','controllers_hinf.mat'), ...
     'K_hin','K_mix','K_pid','gam_hin','gam_mix','gam_pid','g','Wz');

function printmet(m,name)
fprintf('  [%s] ||S||inf=%.3f ||T||inf=%.3f ||KS||inf=%.3g Ms=%.3f stab=%d\n', ...
        name, m.gammaS, m.gammaT, m.gammaKS, m.Ms, m.stable);
fprintf('        att zsdd: body=%.1f wheel=%.1f dB | att dt: body=%.1f wheel=%.1f dB\n', ...
        m.att_body, m.att_wheel, m.att_dt_body, m.att_dt_wheel);
end
