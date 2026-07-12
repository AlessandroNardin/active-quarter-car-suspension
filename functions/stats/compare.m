% =========================================================================
% SCRIPT: ekf_vs_pf_realtime_comparison.m
% =========================================================================
% Purpose: 
%   Esegue la simulazione parallela di EKF e PF sullo stesso modello,
%   estrae i dati dai rispettivi outport e genera metriche e grafici
%   comparativi per la relazione finale.
% =========================================================================
clear variables; close all; clc;

fprintf('=== INIZIO CONFRONTO PRESTAZIONALE: EKF vs PARTICLE FILTER ===\n');
fprintf('Caricamento parametri e sintonizzazioni ottime...\n');
run('params.m');

% Scegli lo scenario (true = modello perturbato, false = modello nominale)
FLAG_PERTURBED_SYSTEM = true; 

if FLAG_PERTURBED_SYSTEM
    scenario_str = 'PERTURBATO (+5% Errore Parametrico)';
else
    perturbed_plant_param = plant_param; % Riallinea al nominale
    scenario_str = 'NOMINALE';
end

% Assegnazione workspace
assignin('base', 'perturbed_plant_param', perturbed_plant_param);
assignin('base', 'plant_param', plant_param);
assignin('base', 'ekf_param', ekf_param);
assignin('base', 'pf_param', pf_param);

model_name = 'eval_all';
load_system(model_name);
set_param(model_name, 'SimulationMode', 'normal');

%% 1. RUN DELLA SIMULAZIONE UNICA
fprintf('\nEsecuzione della simulazione su orizzonte di 500 secondi...\n');
tic;
simOut = sim(model_name, 'StopTime', '500', 'SrcWorkspace', 'base');
runtime_totale = toc;
fprintf('Simulazione completata in %.2f secondi.\n', runtime_totale);

%% 2. ESTRAZIONE DATI DAI RISPETTIVI BLOCCHI OUTPORT
try
    time_raw = simOut.tout;
    
    % Estrazione Errore EKF
    if isprop(simOut, 'error_stima_ekf') || isfield(simOut, 'error_stima_ekf')
        ekf_err_packet = simOut.error_stima_ekf;
    else
        ekf_err_packet = evalin('base', 'error_stima_ekf');
    end
    if isprop(ekf_err_packet, 'Data') || isfield(ekf_err_packet, 'Data'), ekf_err = ekf_err_packet.Data; else, ekf_err = ekf_err_packet; end
    
    % Estrazione Errore PF
    if isprop(simOut, 'error_stima_pf') || isfield(simOut, 'error_stima_pf')
        pf_err_packet = simOut.error_stima_pf;
    else
        pf_err_packet = evalin('base', 'error_stima_pf');
    end
    if isprop(pf_err_packet, 'Data') || isfield(pf_err_packet, 'Data'), pf_err = pf_err_packet.Data; else, pf_err = pf_err_packet; end

catch ME
    error('Errore nell''estrazione dei blocchi. Verifica che nel modello i ToWorkspace si chiamino "error_stima_ekf" e "error_stima_pf".');
end

%% 3. POST-PROCESSING E ALLINEAMENTO GEOMETRICO
ekf_err = squeeze(ekf_err(:));
pf_err  = squeeze(pf_err(:));

% Pareggiamento lunghezze
N_samples = min([length(ekf_err), length(pf_err)]);
ekf_err = ekf_err(1:N_samples);
pf_err  = pf_err(1:N_samples);
time    = linspace(time_raw(1), time_raw(end), N_samples)';


%% 4. CALCOLO METRICHE DI PERFORMANCE (RMSE)
% Escludiamo i primi 20 secondi (ipotizzando una frequenza di 2000Hz o simile, tagliamo i campioni iniziali)
idx_regime = time > 0; 

rmse_ekf_regime = sqrt(mean(ekf_err(idx_regime).^2));
rmse_pf_regime  = sqrt(mean(pf_err(idx_regime).^2));

fprintf('RMSE a Regime (Dopo 200s) - EKF: %.6f cm | PF: %.6f cm\n', rmse_ekf_regime, rmse_pf_regime);

% Calcolo guadagno relativo del PF rispetto all'EKF
delta_perf = ((rmse_ekf_regime - rmse_pf_regime) / rmse_ekf_regime) * 100;

%% 5. STAMPA DEL TABELLONE COMPARATIVO IN COMMAND WINDOW
fprintf('\n============================================================================\n');
fprintf('                     TABELLA COMPARATIVA FINALE (500s)                      \n');
fprintf('============================================================================\n');
fprintf(' Scenario Attivo nel Plant           : %s\n', scenario_str);
fprintf(' Numero totale di campioni analizzati: %d\n', N_samples);
fprintf('----------------------------------------------------------------------------\n');
fprintf(' -> RMSE Global Error - EKF          : %.6f cm\n', rmse_ekf_regime);
fprintf(' -> RMSE Global Error - Particle Filter: %.6f cm\n', rmse_pf_regime);
fprintf('----------------------------------------------------------------------------\n');

if delta_perf > 0
    fprintf(' VERDETTO: Il Particle Filter supera l''EKF riducendo l''errore del %.2f%%\n', delta_perf);
else
    fprintf(' VERDETTO: L''EKF supera il Particle Filter riducendo l''errore del %.2f%%\n', abs(delta_perf));
end
fprintf('============================================================================\n');

%% 6. GENERAZIONE GRAFICO COMPARATIVO (ALLEGGERITO PER LA RELAZIONE)
figure('Name', 'Confronto Prestazioni: EKF vs Particle Filter', 'Color', 'w');

% Plot decimato a 1 punto ogni 5 per fluidità grafica
plot(time(1:5:end), ekf_err(1:5:end), 'Color', [0.2 0.4 0.8], 'LineWidth', 1, 'DisplayName', 'Errore di Stima EKF');
hold on;
plot(time(1:5:end), pf_err(1:5:end), 'Color', [0.2 0.7 0.2], 'LineWidth', 1, 'DisplayName', 'Errore di Stima PF');

grid on;
title(sprintf('Evoluzione Temporale dell''Errore di Stima - Scenario %s', scenario_str));
xlabel('Tempo [s]'); ylabel('Errore [cm]');
legend('Location', 'best');

% Zoom tattico sui primi 40 secondi per mostrare il comportamento nei transitori
xlim([0, min(40, time(end))]);