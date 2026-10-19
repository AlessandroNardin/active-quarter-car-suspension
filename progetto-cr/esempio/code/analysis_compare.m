%% analysis_compare.m - Task 4 (confronto) e Task 5 (robustezza "a mano")
clear; clc; close all;
p = params(); sys = model_quartercar(p);

% ---- carica i controllori -------------------------------------------------
L1=load(fullfile('..','results','controllers_lqg.mat'));
L2=load(fullfile('..','results','controllers_hinf.mat'));
L3=load(fullfile('..','results','controllers_musyn.mat'));
C = { 'LQG',          L1.K_lqg;
      'LQG+integrat.', L1.K_lqg_i;
      'Hinf (hinfsyn)', L2.K_hin;
      'Hinf (mixsyn)',  L2.K_mix;
      'Hinf (struct)',  L2.K_pid;
      'mu-synthesis',   L3.K_mu };
names = C(:,1); Ks = C(:,2);

% ============================ Task 4 ====================================
fprintf('================ CONFRONTO (Task 4) ================\n');
fprintf('%-16s %8s %8s %8s %8s %8s %8s %8s\n','controller','Ms','||T||','GM[dB]','PM[deg]','att_body','att_wheel','poli stab');
mets = cell(size(Ks));
for i=1:numel(Ks)
  K = Ks{i};
  m = fast_metrics(K, sys, p);
  mets{i}=m;
  fprintf('%-16s %8.2f %8.2f %8.2f %8.1f %8.1f %8.1f %8d\n', ...
     names{i}, m.Ms, m.Tinf, m.GM, m.PM, m.att_body, m.att_wheel, m.stable);
end
save(fullfile('..','results','compare_metrics.mat'),'names','mets');

% --- figura: |S| di tutti i controllori ----------------------------------
figure('Color','w'); hold on;
w=2*pi*logspace(-1,2,300);
cols=lines(numel(Ks));
for i=1:numel(Ks)
  m=mets{i}; mag=zeros(size(w));
  for k=1:numel(w), mag(k)=norm(squeeze(freqresp(m.S,w(k)))); end
  semilogx(w/(2*pi), 20*log10(mag), 'LineWidth',1.4,'Color',cols(i,:));
end
grid on; xlabel('Hz'); ylabel('|S| [dB]'); title('Sensibilita'' |S|');
legend(names,'Location','southeast'); ylim([-40 20]);
saveas(gcf, fullfile('..','results','fig','compare_S.png'));

figure('Color','w'); hold on;
for i=1:numel(Ks)
  m=mets{i}; mag=zeros(size(w));
  for k=1:numel(w), mag(k)=norm(squeeze(freqresp(m.CL(1,1),w(k)))); end
  semilogx(w/(2*pi),20*log10(mag),'LineWidth',1.4,'Color',cols(i,:));
end
% anello aperto (passivo)
mag=zeros(size(w)); for k=1:numel(w), mag(k)=abs(freqresp(mets{1}.OL(1,1),w(k))); end
semilogx(w/(2*pi),20*log10(mag),'k--','LineWidth',1.2);
grid on; xlabel('Hz'); ylabel('|z_{sdd}/w| [dB]'); title('Trasferimento disturbo->accelerazione');
legend([names;{'passivo'}],'Location','southwest');
saveas(gcf, fullfile('..','results','fig','compare_zdist.png'));

% ============================ Task 5 ====================================
fprintf('\n=========== ROBUSTEZZA Monte-Carlo (Task 5) ===========\n');
Ns=60; rng(7);
% set di incertezza campionato
plants = cell(Ns,1);
for k=1:Ns
  pp=p; d=@() 2*rand-1;
  pp.ms=p.ms*(1+p.unc.ms*d()); pp.ks0=p.ks0*(1+p.unc.ks0*d());
  pp.bs=p.bs*(1+p.unc.bs*d()); pp.kt=p.kt*(1+p.unc.kt*d());
  pp.tau1=p.tau1*(1+p.unc.tau1*d()); pp.tau2=p.tau2*(1+p.unc.tau2*d());
  plants{k}=pp;
end

fprintf('%-16s %10s %10s %10s %10s\n','controller','%stab','g_zsdd','g_dt','g_ds');
for i=1:numel(Ks)
  K=Ks{i}; nstab=0; r1=zeros(Ns,1); r2=r1; r3=r1;
  for k=1:Ns
    pp=plants{k}; sk=model_quartercar(pp);
    [stable,rn]=cl_rms(K, sk, pp);
    nstab=nstab+stable; r1(k)=rn(1); r2(k)=rn(2); r3(k)=rn(3);
  end
  fprintf('%-16s %9.0f%% %10.3f %10.3e %10.3e\n', names{i}, 100*nstab/Ns, ...
          rms(r1), rms(r2), rms(r3));
end

% --- confronto tempo: dosso stradale su impianto perturbato --------------
pp = plants{ find(cellfun(@(q) q.ms>p.ms, plants),1) };   % un impianto "carico"
sk = model_quartercar(pp);
T=0:0.001:3; wb = sinc_bump(T);             % velocita' stradale (dosso)
figure('Color','w'); hold on;
col2=lines(numel(Ks));
for i=1:numel(Ks)
  y = lsim_cl(Ks{i}, sk, pp, T, wb);
  plot(T, y(:,1), 'Color',col2(i,:),'LineWidth',1.2);
end
grid on; xlabel('t [s]'); ylabel('z_s acc [m/s^2]');
title('Risposta al dosso: accelerazione carrozzeria (impianto +20% m_s)');
legend(names,'Location','northeast');
saveas(gcf, fullfile('..','results','fig','compare_bump.png'));

% ============================ helper ====================================
function m = fast_metrics(K, sys, p)
  A=sys.A; B=sys.B; C=sys.C; Bu=B(:,1:2); Bw=B(:,3);
  Cmeas=C([1 3 4],:); Cz=C(1:3,:); nx=9;
  Mzp=[Cz;zeros(2,nx)]; Nzp=[0 0;0 0;0 0;1 0;0 1];
  [Ak,Bk,Ck,Dk]=ssdata(minreal(K,1e-6));
  Acl=[A-Bu*Dk*Cmeas, -Bu*Ck; Bk*Cmeas, Ak];
  Bcl=[Bw,-Bu*Dk; zeros(size(Bk,1),1),Bk];
  Czc=[Mzp-Nzp*Dk*Cmeas, -Nzp*Ck]; Dzc=[zeros(5,1),-Nzp*Dk];
  m.poles=eig(Acl); m.stable=all(real(m.poles)<0);
  m.S=minreal(inv(eye(3)+ss(A,Bu,Cmeas,0)*K));
  m.T=eye(3)-m.S; m.CL=ss(Acl,Bcl,Czc,Dzc); m.OL=ss(A,Bw,Mzp,0);
  m.Ms=norm(m.S,'inf'); m.Tinf=norm(m.T,'inf');
  try
    L11 = (ss(A,Bu,Cmeas,0)*K); L11 = L11(1,1);   % anello comfort (zsdd<->u1c)
    [Gm,Pm] = margin(L11); m.GM=20*log10(Gm); m.PM=Pm;
  catch, m.GM=NaN; m.PM=NaN; end
  fb=p.f_body; fw=p.f_wheel;
  m.att_body =20*log10(abs(freqresp(m.CL(1,1),2*pi*fb)))-20*log10(abs(freqresp(m.OL(1,1),2*pi*fb)));
  m.att_wheel=20*log10(abs(freqresp(m.CL(1,1),2*pi*fw)))-20*log10(abs(freqresp(m.OL(1,1),2*pi*fw)));
end

function [stable,rn] = cl_rms(K, sys, p)
  A=sys.A; B=sys.B; C=sys.C; Bu=B(:,1:2); Bw=B(:,3);
  Cmeas=C([1 3 4],:); Cz=C(1:3,:); nx=9;
  Mzp=[Cz;zeros(2,nx)]; Nzp=[0 0;0 0;0 0;1 0;0 1];
  [Ak,Bk,Ck,Dk]=ssdata(minreal(K,1e-6));
  Acl=[A-Bu*Dk*Cmeas, -Bu*Ck; Bk*Cmeas, Ak];
  Bcl=[Bw; zeros(size(Bk,1),1)];
  Czc=[Mzp-Nzp*Dk*Cmeas, -Nzp*Ck]; Dzc=zeros(5,1);
  CL=minreal(ss(Acl,Bcl,Czc,Dzc),1e-6);
  stable=all(real(eig(Acl))<0);
  if stable
    rn=[norm(CL(1,1),'inf'); norm(CL(2,1),'inf'); norm(CL(3,1),'inf')];
  else
    rn=[NaN;NaN;NaN];
  end
end

function w = sinc_bump(T)
  % profilo di velocita' stradale tipo dosso (half-sine) centrato
  Tb=0.15; w=zeros(size(T));
  idx = T<=Tb; w(idx)=sin(pi*T(idx)/Tb);
end

function Y = lsim_cl(K, sys, p, T, w)
  A=sys.A; B=sys.B; C=sys.C; Bu=B(:,1:2); Bw=B(:,3);
  Cmeas=C([1 3 4],:); Cz=C(1:3,:); nx=9;
  Mzp=[Cz;zeros(2,nx)]; Nzp=[0 0;0 0;0 0;1 0;0 1];
  [Ak,Bk,Ck,Dk]=ssdata(minreal(K,1e-6));
  Acl=[A-Bu*Dk*Cmeas, -Bu*Ck; Bk*Cmeas, Ak];
  Bcl=[Bw; zeros(size(Bk,1),1)];
  Czc=[Mzp-Nzp*Dk*Cmeas, -Nzp*Ck]; Dzc=zeros(5,1);
  CL=ss(Acl,Bcl,Czc,Dzc);
  Y=lsim(CL, w, T);
end
