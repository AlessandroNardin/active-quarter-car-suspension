%% ============================================================
%  BODE DELL'ATTUATORE SECONDARIO u2  (obiettivo: road-holding)
%
%  Modello (traccia):
%     Ga2(s) = 1 / (1 + tau_a*s) * exp(-tau2*s)
%     sistema del 1° ordine con ritardo di trasporto.
%
%  Parametri nominali dalla Tabella 1 della traccia.
%  Lo script mostra:
%    - Bode con ritardo ESATTO
%    - Bode con ritardo approssimato con Pade' del 1° ordine
%    - frequenze di riferimento:
%         modo carrozzeria (bounce)  ~1.2 Hz   [linea verde]
%         modo ruota (wheel-hop)     ~10 Hz    [linea rossa]
%         banda attuatore u2         ~6.4 Hz   [linea blu]
%  ============================================================
clear; clc; close all;

%% ---- Parametri dell'attuatore (Tabella 1) ------------------
tau_a = 25e-3;     % costante di tempo     [s]   (banda ~6.4 Hz)
tau2  = 10e-3;     % ritardo di trasporto  [s]

%% ---- Dinamica del 1° ordine (senza ritardo) ----------------
Ga2_0 = tf(1, [tau_a 1]);

%% ---- Ritardo esatto ----------------------------------------
Ga2 = Ga2_0;
Ga2.InputDelay = tau2;                     % exp(-tau2*s)

%% ---- Ritardo approssimato con Pade' del 1° ordine ----------
[numP, denP] = pade(tau2, 1);
Ga2_pade = Ga2_0 * tf(numP, denP);

%% ---- Diagramma di Bode -------------------------------------
% NOTA: il range di frequenze di `bode` e' sempre in rad/s,
%       anche quando l'asse e' visualizzato in Hz.
fmin = 0.1;  fmax = 100;                   % [Hz]
opts = bodeoptions;
opts.FreqUnits     = 'Hz';
opts.Grid          = 'on';
opts.Title.String  = 'Bode attuatore secondario u_2 (road-holding)';

figure('Name', 'Bode attuatore u2 (road-holding)', 'Color', 'w');
bode(Ga2, Ga2_pade, opts, {2*pi*fmin, 2*pi*fmax});

%% ---- Frequenze di riferimento ------------------------------
ms = 350; mu = 50; ks0 = 20000; kt = 180000;
f_body  = sqrt(ks0/ms)      / (2*pi);      % modo carrozzeria  ~1.2 Hz
f_wheel = sqrt((ks0+kt)/mu) / (2*pi);      % modo ruota        ~10 Hz
f_att2  = 1/(tau_a*2*pi);                  % banda attuatore   ~6.4 Hz

ax    = findobj(gcf, 'Type', 'axes');
axMag = ax(1);                             % asse magnitudine
axPh  = ax(2);                             % asse fase

xlB = xline(axMag, f_body,  '--', 'Color', [0 0.5 0], 'LineWidth', 1);
xlW = xline(axMag, f_wheel, '--', 'Color', [0.8 0 0], 'LineWidth', 1);
xlA = xline(axMag, f_att2,  ':',  'Color', [0 0 0.8], 'LineWidth', 1);
xline(axPh, f_body,  '--', 'Color', [0 0.5 0], 'LineWidth', 1);
xline(axPh, f_wheel, '--', 'Color', [0.8 0 0], 'LineWidth', 1);
xline(axPh, f_att2,  ':',  'Color', [0 0 0.8], 'LineWidth', 1);

hC = flipud(findobj(axMag, 'Type', 'line'));   % curve di Bode
legend(axMag, [hC; xlB; xlW; xlA], ...
    {'ritardo esatto', 'Pade'' 1° ordine', ...
     sprintf('modo carrozzeria (%.1f Hz)', f_body), ...
     sprintf('modo ruota (%.1f Hz)', f_wheel), ...
     sprintf('banda attuatore u2 (%.1f Hz)', f_att2)}, ...
    'Location', 'southwest');

%% ---- Sintesi numerica --------------------------------------
fprintf('--- Attuatore u2 ---\n');
fprintf('Banda del 1° ordine (1/tau_a): %.1f rad/s = %.2f Hz\n', 1/tau_a, f_att2);
fprintf('Ritardo equivalente (1° ord.) : %.1f ms\n', tau_a*1000);
fprintf('Sfasamento del ritardo a 1/tau_a: %.1f deg\n', -(1/tau_a)*tau2*180/pi);

%% ---- Salvataggio figura ------------------------------------
outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~exist(outDir, 'dir'); mkdir(outDir); end
exportgraphics(gcf, fullfile(outDir, 'bode_u2.png'), 'Resolution', 150);
fprintf('Figura salvata in: %s\n', fullfile(outDir, 'bode_u2.png'));
