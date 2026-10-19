function m = cl_metrics(K, sys, p, doPlot, tag)
% =========================================================================
%  cl_metrics.m
%  Valutazione di un controllore K (3 misure y -> 2 comandi u) sul modello
%  linearizzato. Ritorna metriche di performance/robustezza e (opz.) grafici.
%
%  Convenzione:  u = -K*(y + n),  y = Cmeas*x + n.
%
%  Metriche:
%    m.gammaS,gammaT,gammaKS : norme Hinf di S, T, KS
%    m.Ms                   : picco di S (su tutte le direzioni)
%    m.gm, m.pm             : margini di gain/phase (diskmargin se disponibile)
%    m.att_dB(5)            : attenuazione disturbo->[zsdd;dt;ds;u1c;u2c] (dB)
%                             rispetto all'anello aperto, valutata ai modi
%    m.CL                   : ss da [w;n1..n3] a z_phys
% =========================================================================
if nargin<4, doPlot=false; end
if nargin<5, tag=''; end

A=sys.A; B=sys.B; C=sys.C;
Bu=B(:,1:2); Bw=B(:,3); nx=numel(A(:,1));
Cmeas=C([1 3 4],:); Cz=C(1:3,:);
Mzp=[Cz; zeros(2,nx)]; Nzp=[0 0;0 0;0 0;1 0;0 1];

% controller state-space
if isa(K,'ss'), [Ak,Bk,Ck,Dk]=ssdata(K); else [Ak,Bk,Ck,Dk]=ssdata(ss(K)); end

% ---- anello chiuso [w;n] -> z_phys --------------------------------------
Acl = [A - Bu*Dk*Cmeas, -Bu*Ck; Bk*Cmeas, Ak];
Bcl = [Bw, -Bu*Dk; zeros(size(Bk,1),1), Bk];
Cz_cl = [Mzp - Nzp*Dk*Cmeas, -Nzp*Ck];
Dz_cl = [zeros(5,1), -Nzp*Dk];
CL = ss(Acl,Bcl,Cz_cl,Dz_cl);
CL.InputName={'w','n1','n2','n3'}; CL.OutputName={'zsdd','dt','ds','u1c','u2c'};

% ---- anello aperto (K=0) -------------------------------------------------
OL = ss(A, Bw, Mzp, zeros(5,1));

% ---- S, T, KS ------------------------------------------------------------
Puy = ss(A,Bu,Cmeas,zeros(3,2));
L   = Puy*K;
I3  = eye(3);
S   = minreal(inv(I3+L));
T   = minreal(I3 - S);
KS  = minreal(K*S);

m.S=S; m.T=T; m.KS=KS; m.CL=CL; m.OL=OL;
m.gammaS  = norm(S,'inf');
m.gammaT  = norm(T,'inf');
m.gammaKS = norm(KS,'inf');
m.Ms      = norm(S,'inf');
m.poles   = pole(minreal(ss(Acl,Bcl,Cz_cl,Dz_cl)));
m.stable  = all(real(m.poles)<0);

% margini classici (diskmargin, robust control toolbox)
try
    DM = diskmargin(Puy*K);
    m.gm = DM.GainMargin(1); m.pm = DM.PhaseMargin(1);
catch
    m.gm = NaN; m.pm = NaN;
end

% ---- attenuazione disturbo -> uscite ai modi -----------------------------
fw = p.f_wheel; fb = p.f_body;
m.att_body  = 20*log10(abs(freqresp(CL(1,1),2*pi*fb))) - 20*log10(abs(freqresp(OL(1,1),2*pi*fb)));
m.att_wheel = 20*log10(abs(freqresp(CL(1,1),2*pi*fw))) - 20*log10(abs(freqresp(OL(1,1),2*pi*fw)));
m.att_dt_body  = 20*log10(abs(freqresp(CL(2,1),2*pi*fb))) - 20*log10(abs(freqresp(OL(2,1),2*pi*fb)));
m.att_dt_wheel = 20*log10(abs(freqresp(CL(2,1),2*pi*fw))) - 20*log10(abs(freqresp(OL(2,1),2*pi*fw)));

% ---- norma H2 (RMS sotto disturbo bianco unitario) -----------------------
if m.stable
    try
        m.rms_z   = norm(minreal(CL(:,1),1e-6),2);   % da w a z_phys (H2)
        m.rms_OL  = norm(minreal(OL,1e-6),2);
    catch
        m.rms_z=NaN; m.rms_OL=NaN;
    end
else
    m.rms_z=NaN; m.rms_OL=NaN;
end

if doPlot
    o = sigmaoptions; o.FreqUnits='Hz'; o.Grid='on';
    figure('Color','w','Name',['CL ',tag]);
    subplot(1,2,1); sigma(S, o, {2*pi*0.1,2*pi*100}); title(['|S| (',tag,')']);
    subplot(1,2,2); sigma(T, o, {2*pi*0.1,2*pi*100}); title(['|T| (',tag,')']);
    if ~isempty(tag)
        saveas(gcf, fullfile('..','results','fig',['cl_' tag '.png']));
    end
end
end
