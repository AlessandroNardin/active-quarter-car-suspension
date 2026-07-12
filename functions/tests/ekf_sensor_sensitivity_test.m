% =========================================================================
% SCRIPT: ekf_robustness_comparison_test.m (UPDATED FOR error_stima_ekf)
% =========================================================================
clear variables; close all; clc;

%% 1. CONFIGURAZIONE SCENARIO (NOMINALE o PERTURBATO)
% Imposta su TRUE per testare il sistema perturbato, FALSE per quello nominale
run('params.m'); 
FLAG_PERTURBED_SYSTEM = true; 

fprintf('=== EKF COMPARISON & ROBUSTNESS STRESS TEST ===\n');
fprintf('Caricamento parametri da params.m...\n');

% Gestione del Plant in base al Flag
if FLAG_PERTURBED_SYSTEM
    % Scenario Perturbato: il veicolo reale subisce l'errore parametrico
    assignin('base', 'perturbed_plant_param', perturbed_plant_param);
    scenario_str = 'PERTURBATO (+5% Errore Parametrico)';
else
    % Scenario Nominale: il veicolo reale coincide con il modello ideale
    perturbed_plant_param = plant_param;
    assignin('base', 'perturbed_plant_param', perturbed_plant_param);
    scenario_str = 'NOMINALE';
end

% I filtri (sia EKF che PF) usano i parametri nominali idealizzati
assignin('base', 'plant_param', plant_param);

model_name = 'eval_all'; 
load_system(model_name);
set_param(model_name, 'SimulationMode', 'normal'); 

%% 2. STAMPA A SCHERMO DEI VALORI VERI UTILIZZATI (ISPEZIONE DATI)
fprintf('\n==================== VALORI VERI IN INGRESSO AL SISTEMA ====================\n');
fprintf('Scenario Active: %s\n', scenario_str);
fprintf('----------------------------------------------------------------------------\n');
fprintf('PARAMETRI FISICI VEICOLO:\n');
fprintf(' -> Massa Sospesa (ms)     - Nominale: %6.2f kg  |  Usata nel Plant: %6.2f kg\n', plant_param.ms, perturbed_plant_param.ms);
fprintf(' -> Massa Non Sospesa (mu) - Nominale: %6.2f kg  |  Usata nel Plant: %6.2f kg\n', plant_param.mu, perturbed_plant_param.mu);
fprintf(' -> Rigidezza Sosp. (ks0)  - Nominale: %6.1f N/m |  Usata nel Plant: %6.1f N/m\n', plant_param.ks0, perturbed_plant_param.ks0);
fprintf(' -> Smorzamento Sosp. (bs) - Nominale: %6.1f Ns/m|  Usata nel Plant: %6.1f Ns/m\n', plant_param.bs, perturbed_plant_param.bs);

% Prepariamo la matrice R standard per la stampa
R_std_diag = [lpot_param.noise_var, acc_param.noise_var, acc_param.noise_var];

fprintf('\nMATRICI DI COVARIANZA R (RUMORE DI MISURA) DELL''EKF:\n');
fprintf(' -> Diagonale R STANDARD    [1, 1, 1]  : [ %e,  %e,  %e ]\n', R_std_diag(1), R_std_diag(2), R_std_diag(3));
fprintf(' -> Diagonale R OTTIMIZZATA Nominali   : [ %e,  %e,  %e ]\n', ekf_param.R(1,1), ekf_param.R(2,2), ekf_param.R(3,3));
fprintf('============================================================================\n');

%% 3. RUN 1: SISTEMA CON COEFFICIENTI OTTIMIZZATI NOMINALI
fprintf('\n[RUN 1/2] Simulazione con coefficienti EKF OTTIMIZZATI...\n');
assignin('base', 'ekf_param', ekf_param);
simOut_opt = sim(model_name, 'StopTime', '500', 'SrcWorkspace', 'base');

% Estrazione dal nuovo blocco error_stima_ekf (Ottimizzato)
if isprop(simOut_opt, 'error_stima_ekf') || isfield(simOut_opt, 'error_stima_ekf')
    err_struct_opt = simOut_opt.error_stima_ekf;
else
    err_struct_opt = evalin('base', 'error_stima_ekf');
end

if isstruct(err_struct_opt) && isfield(err_struct_opt, 'signals')
    err_data_opt = err_struct_opt.signals.values;
elseif isprop(err_struct_opt, 'Data') || isfield(err_struct_opt, 'Data')
    err_data_opt = err_struct_opt.Data;
else
    err_data_opt = err_struct_opt; 
end
rmse_optimized = sqrt(mean(err_data_opt.^2, 'all'));

%% 4. RUN 2: SISTEMA CON COEFFICIENTI DI BASE NON OTTIMIZZATI [1, 1, 1]
fprintf('\n[RUN 2/2] Simulazione con coefficienti EKF NON OTTIMIZZATI [1, 1, 1]...\n');
ekf_param_unopt = ekf_param;
ekf_param_unopt.R = diag(R_std_diag);
assignin('base', 'ekf_param', ekf_param_unopt);
simOut_unopt = sim(model_name, 'StopTime', '500', 'SrcWorkspace', 'base');

% Estrazione dal nuovo blocco error_stima_ekf (Non Ottimizzato)
if isprop(simOut_unopt, 'error_stima_ekf') || isfield(simOut_unopt, 'error_stima_ekf')
    err_struct_unopt = simOut_unopt.error_stima_ekf;
else
    err_struct_unopt = evalin('base', 'error_stima_ekf');
end

if isstruct(err_struct_unopt) && isfield(err_struct_unopt, 'signals')
    err_data_unopt = err_struct_unopt.signals.values;
elseif isprop(err_struct_unopt, 'Data') || isfield(err_struct_unopt, 'Data')
    err_data_unopt = err_struct_unopt.Data;
else
    err_data_unopt = err_struct_unopt; 
end
rmse_unoptimized = sqrt(mean(err_data_unopt.^2, 'all'));

%% 5. STAMPA DEL VERDETTO FINALE COMPARATIVO
fprintf('\n==================== EKF ROBUSTNESS STRESS TEST VERDICT ====================\n');
fprintf(' Orizzonte Temporale di Simulazione   : 500 secondi\n');
fprintf(' Scenario del Modello Fisico (Plant)  : %s\n', scenario_str);
fprintf('----------------------------------------------------------------------------\n');
fprintf(' 1. RMSE con Coefficienti di Base [1, 1, 1]      : %.6f \n', rmse_unoptimized);
fprintf(' 2. RMSE con Coefficienti Ottimizzati Nominali   : %.6f \n', rmse_optimized);
fprintf('----------------------------------------------------------------------------\n');

improvement = ((rmse_unoptimized - rmse_optimized) / rmse_unoptimized) * 100;
if improvement > 0
    fprintf(' --> L''ottimizzazione riduce l''errore del %.2f%% in questo scenario!\n', improvement);
else
    fprintf(' --> ATTENZIONE: Il filtro di base si comporta in modo analogo o migliore (Scostamento: %.2f%%).\n', improvement);
end
fprintf('============================================================================\n');