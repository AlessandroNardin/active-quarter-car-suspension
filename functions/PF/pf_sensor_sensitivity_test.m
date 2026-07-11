% =========================================================================
% SCRIPT: pf_robustness_comparison_test.m (UPDATED FOR error_stima_pf)
% =========================================================================
% Purpose: 
%   Confronta il Particle Filter con parametri OTTIMIZZATI vs STANDARD.
%   Permette di scegliere tramite un FLAG se testare il comportamento sul
%   SISTEMA NOMINALE oppure sul SISTEMA PERTURBATO.
%   Aggiornato per estrarre l'errore dal nuovo blocco Outport: error_stima_pf
% =========================================================================
clear variables; close all; clc;

%% 1. CONFIGURAZIONE SCENARIO (NOMINALE o PERTURBATO)
% Imposta su TRUE per testare il sistema perturbato, FALSE per quello nominale
run('params.m'); 
FLAG_PERTURBED_SYSTEM = true; 

fprintf('=== PARTICLE FILTER STRESS & COMPARISON TEST ===\n');
fprintf('Caricamento parametri da params.m...\n');

if FLAG_PERTURBED_SYSTEM
    % Scenario Perturbato: applichiamo l'errore parametrico del 5% al plant
    assignin('base', 'perturbed_plant_param', perturbed_plant_param);
    fprintf('SCENARIO SELEZIONATO: SISTEMA PERTURBATO (+5%% Errore Parametrico)\n');
else
    % Scenario Nominale: il plant coincide con il modello nominale
    perturbed_plant_param = plant_param;
    assignin('base', 'perturbed_plant_param', perturbed_plant_param);
    fprintf('SCENARIO SELEZIONATO: SISTEMA NOMINALE\n');
end

model_name = 'eval_all'; 
load_system(model_name);
set_param(model_name, 'SimulationMode', 'normal'); 

% Salviamo in memoria i parametri OTTIMI estratti da params.m
q_gain_opt  = pf_param.q_gain;   % 0.001022
epsilon_opt = pf_param.epsilon;  % 0.000053

%% 2. RUN 1: CONFIGURAZIONE CON PARAMETRI OTTIMIZZATI (Q ed epsilon di fino)
fprintf('\n[RUN 1/2] Simulazione con parametri OTTIMIZZATI (Q e eps ottimi)...\n');
% pf_param nel workspace base ha già i valori ottimi letti da params.m
assignin('base', 'pf_param', pf_param);
simOut_opt = sim(model_name, 'StopTime', '500', 'SrcWorkspace', 'base');

% Estrazione dal NUOVO blocco error_stima_pf (Ottimizzato)
if isprop(simOut_opt, 'error_stima_pf') || isfield(simOut_opt, 'error_stima_pf')
    err_struct_opt = simOut_opt.error_stima_pf;
else
    err_struct_opt = evalin('base', 'error_stima_pf');
end

if isstruct(err_struct_opt) && isfield(err_struct_opt, 'signals')
    err_data_opt = err_struct_opt.signals.values;
elseif isprop(err_struct_opt, 'Data') || isfield(err_struct_opt, 'Data')
    err_data_opt = err_struct_opt.Data;
else
    err_data_opt = err_struct_opt; 
end
rmse_optimized = sqrt(mean(err_data_opt.^2, 'all'));
fprintf(' -> Completato! RMSE Ottimizzato: %.6f\n', rmse_optimized);

%% 3. RUN 2: CONFIGURAZIONE CON PARAMETRI STANDARD (q_gain = 1, epsilon = 0.01)
fprintf('\n[RUN 2/2] Simulazione con parametri STANDARD (q_gain = 1.0, eps = 0.01)...\n');

% Ripristiniamo forzatamente i valori standard di fabbrica prima dell'ottimizzazione
pf_param_std = pf_param;
pf_param_std.q_gain  = 1.0;
pf_param_std.epsilon = 0.010000;

% Ricalcoliamo la matrice Q e la sua fattorizzazione di Cholesky con il guadagno standard
pf_param_std.Q   = pf_param_std.q_gain * diag([u_noise_param.u1_var, u_noise_param.u2_var, r_param.rz_var, r_param.rzdot_var]);
pf_param_std.L_Q = chol(pf_param_std.Q, 'lower');

assignin('base', 'pf_param', pf_std = pf_param_std); % Compatibilità per workspace
assignin('base', 'pf_param', pf_param_std);

simOut_std = sim(model_name, 'StopTime', '500', 'SrcWorkspace', 'base');

% Estrazione dal NUOVO blocco error_stima_pf (Standard)
if isprop(simOut_std, 'error_stima_pf') || isfield(simOut_std, 'error_stima_pf')
    err_struct_std = simOut_std.error_stima_pf;
else
    err_struct_std = evalin('base', 'error_stima_pf');
end

if isstruct(err_struct_std) && isfield(err_struct_std, 'signals')
    err_data_std = err_struct_std.signals.values;
elseif isprop(err_struct_std, 'Data') || isfield(err_struct_std, 'Data')
    err_data_std = err_struct_std.Data;
else
    err_data_std = err_struct_std; 
end
rmse_standard = sqrt(mean(err_data_std.^2, 'all'));
fprintf(' -> Completato! RMSE Standard: %.6f\n', rmse_standard);

%% 4. STAMPA DEL TABELLONE COMPARATIVO DEL FILTRO
fprintf('\n==================== PF COMPARISON & STRESS TEST VERDICT ====================\n');
fprintf(' Orizzonte Temporale di Simulazione   : 500 secondi\n');
if FLAG_PERTURBED_SYSTEM
    fprintf(' Scenario del Modello Fisico (Plant)  : PERTURBATO (+5%% Errore Parametrico)\n');
else
    fprintf(' Scenario del Modello Fisico (Plant)  : NOMINALE\n');
end
fprintf('----------------------------------------------------------------------------\n');
fprintf(' 1. RMSE con Assetto STANDARD (q=1.0, eps=0.01)    : %.6f\n', rmse_standard);
fprintf(' 2. RMSE con Assetto OTTIMIZZATO (q=%.6f, eps=%e) : %.6f\n', q_gain_opt, epsilon_opt, rmse_optimized);
fprintf('----------------------------------------------------------------------------\n');

% Calcolo del guadagno percentuale di accuratezza
improvement = ((rmse_standard - rmse_optimized) / rmse_standard) * 100;
if improvement > 0
    fprintf(' --> L''ottimizzazione riduce l''errore del %.2f%% in questo scenario!\n', improvement);
else
    fprintf(' --> ATTENZIONE: L''assetto standard si comporta in modo analogo o migliore (Scostamento: %.2f%%).\n', improvement);
end
fprintf('============================================================================\n');