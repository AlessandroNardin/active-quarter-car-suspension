%% ============================================================
%  BODE DELL'ATTUATORE PRIMARIO u1  (obiettivo: comfort)
%
%  Modello (traccia):
%     Ga1(s) = wn1^2 / (s^2 + 2*zeta1*wn1*s + wn1^2) * exp(-tau1*s)
%     sistema del 2° ordine con ritardo di trasporto.
%
%  Parametri nominali dalla Tabella 1 della traccia.
%  Lo script mostra:
%    - Bode con ritardo ESATTO
%    - Bode con ritardo approssimato con Pade' del 1° ordine
%      (l'approssimazione usata nel modello di progetto)
%    - i modi del quarter-car come frequenze di riferimento:
%         modo carrozzeria (bounce)  ~1.2 Hz   [linea verde]
%         modo ruota (wheel-hop)     ~10 Hz    [linea rossa]
%  ============================================================
clear; clc; close all;

%% ---- Parametri dell'attuatore (Tabella 1) ------------------
wn1   = 80;        % pulsazione naturale   [rad/s]  (~12.7 Hz)
zeta1 = 0.7;       % smorzamento           [-]
tau1  = 5e-3;      % ritardo di trasporto  [s]

%% ---- Dinamica del 2° ordine (senza ritardo) ----------------
Ga1_0 = tf(wn1^2, [1, 2*zeta1*wn1, wn1^2]);

%% ---- Ritardo esatto ----------------------------------------
Ga1 = Ga1_0;
Ga1.InputDelay = tau1;                     % exp(-tau1*s)

%% ---- Ritardo approssimato con Pade' del 1° ordine ----------
[numP, denP] = pade(tau1, 1);
Ga1_pade = Ga1_0 * tf(numP, denP);

%% ---- Diagramma di Bode -------------------------------------
% NOTA: il range di frequenze di `bode` e' sempre in rad/s,
%       anche quando l'asse e' visualizzato in Hz.
fmin = 0.1;  fmax = 100;                   % [Hz]
opts = bodeoptions;
opts.FreqUnits     = 'Hz';
opts.Grid          = 'on';
opts.Title.String  = 'Bode attuatore primario u_1 (comfort)';

figure('Name', 'Bode attuatore u1 (comfort)', 'Color', 'w');
bode(Ga1, Ga1_pade, opts, {2*pi*fmin, 2*pi*fmax});

%% ---- Frequenze di riferimento (modi del quarter-car) -------
ms = 350; mu = 50; ks0 = 20000; kt = 180000;
f_body  = sqrt(ks0/ms)      / (2*pi);      % modo carrozzeria  ~1.2 Hz
f_wheel = sqrt((ks0+kt)/mu) / (2*pi);      % modo ruota        ~10 Hz

ax    = findobj(gcf, 'Type', 'axes');
axMag = ax(1);                             % asse magnitudine
axPh  = ax(2);                             % asse fase

xlB = xline(axMag, f_body,  '--', 'Color', [0 0.5 0], 'LineWidth', 1);
xlW = xline(axMag, f_wheel, '--', 'Color', [0.8 0 0], 'LineWidth', 1);
xline(axPh, f_body,  '--', 'Color', [0 0.5 0], 'LineWidth', 1);
xline(axPh, f_wheel, '--', 'Color', [0.8 0 0], 'LineWidth', 1);

hC = flipud(findobj(axMag, 'Type', 'line'));   % curve di Bode
legend(axMag, [hC; xlB; xlW], ...
    {'ritardo esatto', 'Pade'' 1° ordine', ...
     sprintf('modo carrozzeria (%.1f Hz)', f_body), ...
     sprintf('modo ruota (%.1f Hz)', f_wheel)}, ...
    'Location', 'southwest');

%% ---- Sintesi numerica --------------------------------------
fprintf('--- Attuatore u1 ---\n');
fprintf('Banda naturale (senza ritardo) : %.1f rad/s = %.2f Hz\n', wn1, wn1/(2*pi));
fprintf('Ritardo equivalente T_eq=2z/wn: %.1f ms\n', 2*zeta1/wn1*1000);
fprintf('Sfasamento del ritardo a wn1  : %.1f deg\n', -wn1*tau1*180/pi);

%% ---- Salvataggio figura ------------------------------------
outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~exist(outDir, 'dir'); mkdir(outDir); end
exportgraphics(gcf, fullfile(outDir, 'bode_u1.png'), 'Resolution', 150);
fprintf('Figura salvata in: %s\n', fullfile(outDir, 'bode_u1.png'));
