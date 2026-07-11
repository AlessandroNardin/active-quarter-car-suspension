% =========================================================================
% SCRIPT: ekf_covariance_optimization_fast.m (WITH 500s VALIDATION)
% =========================================================================
% Purpose: 
%   Ottimizzazione globale dei coefficienti della matrice R dell'EKF.
%   Aggiornato con il nuovo Outport/ToWorkspace: error_stima_ekf
% =========================================================================
clear variables; close all; clc;

%% 0. CONFIGURAZIONE STRATEGIA DI OTTIMIZZAZIONE
% 1. Carica PRIMA i parametri di base
fprintf('Initializing baseline parameters from params.m...\n');
run('params.m');

% 2. Definisci il flag SUBITO DOPO il run (Senza apici!)
FLAG_NOMINAL_TUNING = true;

% GESTIONE INTELLIGENTE DEL PLANT
if FLAG_NOMINAL_TUNING  
    fprintf('--> STRATEGIA: Ottimizzazione sul SISTEMA NOMINALE.\n');
    perturbed_plant_param = plant_param; 
else
    fprintf('--> STRATEGIA: Ottimizzazione sul SISTEMA PERTURBATO (+5%%).\n');
end
assignin('base', 'perturbed_plant_param', perturbed_plant_param);

pot_base_noise = lpot_param.noise_var;
acc_base_noise = acc_param.noise_var;

model_name = 'eval_all'; 
fprintf('Configuring model "%s" to ACCELERATOR mode...\n', model_name);
load_system(model_name);
set_param(model_name, 'SimulationMode', 'accelerator');

%% 2. GLOBAL OPTIMIZATION SETUP
start_coeffs = [1.0, 1.0, 1.0];
lb = [0.001, 0.001, 0.001]; 
ub = [500, 500, 500];       

if isempty(gcp('nocreate'))
    fprintf('Opening Parallel Pool for multi-core acceleration...\n');
    parpool; 
end

options = optimoptions('fmincon', ...
                       'Algorithm', 'interior-point', ...
                       'Display', 'iter', ...
                       'MaxIterations', 100, ...    
                       'UseParallel', true);       

problem = createOptimProblem('fmincon', ...
    'objective', @(coeffs) ekf_cost_function(coeffs, pot_base_noise, acc_base_noise, ekf_param, model_name, perturbed_plant_param), ...
    'x0', start_coeffs, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options);

gs = GlobalSearch('Display', 'iter', ...
                  'NumTrialPoints', 30, ...
                  'NumStageOnePoints', 10);

fprintf('\nLaunching Parallel GlobalSearch (Optimization window: 50 seconds)...\n');

%% 3. OPTIMIZATION RUN
tic;
[best_coeffs, min_error] = run(gs, problem);
elapsed_time = toc;

%% 4. SIMULINK RESTORATION
fprintf('\nRestoring Simulink model back to Normal mode...\n');
set_param(model_name, 'SimulationMode', 'normal');
save_system(model_name);

%% 5. OPTIMIZATION VERDICT PRINTING
fprintf('\n======================= FAST GLOBAL OPTIMIZATION RESULTS =======================\n');
fprintf('Optimization completed in %.1f seconds on a 50s window.\n', elapsed_time);
fprintf('OPTIMAL COEFFICIENTS FOUND:\n');
fprintf(' -> Optimal Multiplier - Potentiometer (Sensor 1) : %.4f\n', best_coeffs(1));
fprintf(' -> Optimal Multiplier - Body Acc      (Sensor 2) : %.4f\n', best_coeffs(2));
fprintf(' -> Optimal Multiplier - Wheel Acc     (Sensor 3) : %.4f\n', best_coeffs(3));
fprintf('====================================================================\n');

%% 6. AUTOMATIC FINAL VALIDATION (FULL HORIZON: 500 SECONDS)
fprintf('\nLaunching final validation run over the FULL 500-second horizon...\n');

% Applichiamo i coefficienti ottimi trovati alla struttura finale
ekf_param_opt = ekf_param;
ekf_param_opt.R = diag([pot_base_noise * best_coeffs(1), ...
                        acc_base_noise * best_coeffs(2), ...
                        acc_base_noise * best_coeffs(3)]);

% Carichiamo l'assetto ottimo nel Workspace Base
assignin('base', 'ekf_param', ekf_param_opt);
assignin('base', 'perturbed_plant_param', perturbed_plant_param);

% Facciamo girare la simulazione completa a 500s in Normal Mode
simOutFinal = sim(model_name, 'StopTime', '500', 'SrcWorkspace', 'base');

% Estrazione dell'errore sui 500s dal nuovo blocco error_stima_ekf
if isprop(simOutFinal, 'error_stima_ekf') || isfield(simOutFinal, 'error_stima_ekf')
    error_struct_final = simOutFinal.error_stima_ekf;
else
    error_struct_final = evalin('base', 'error_stima_ekf');
end

if isstruct(error_struct_final) && isfield(error_struct_final, 'signals')
    err_data_final = error_struct_final.signals.values;
elseif isprop(error_struct_final, 'Data') || isfield(error_struct_final, 'Data')
    err_data_final = error_struct_final.Data;
else
    err_data_final = error_struct_final; 
end

final_mse_500s = mean(err_data_final.^2, 'all');
final_rmse_500s = sqrt(final_mse_500s);

fprintf('\n======================= FINAL 500s VALIDATION VERDICT =======================\n');
if FLAG_NOMINAL_TUNING
    fprintf(' VALIDATION SCENARIO            : NOMINAL SYSTEM\n');
else
    fprintf(' VALIDATION SCENARIO            : PERTURBED SYSTEM\n');
end
fprintf(' Total Mean Squared Error (MSE)  : %.6f\n', final_mse_500s);
fprintf(' Total Root Mean Squared Error (RMSE): %.6f\n', final_rmse_500s);
fprintf('=============================================================================\n');

%% =========================================================================
% LOCAL FUNCTION: COVARIANCE TUNING COST FUNCTION (Restricted to 50s)
% =========================================================================
function cost = ekf_cost_function(coeffs, pot_noise, acc_noise, ekf_param_base, model_name, plant_param_scen)
    assignin('base', 'perturbed_plant_param', plant_param_scen);
    ekf_param = ekf_param_base;
    ekf_param.R = diag([pot_noise * coeffs(1), ...
                        acc_noise * coeffs(2), ...
                        acc_noise * coeffs(3)]);
    assignin('base', 'ekf_param', ekf_param);
    try
        simOut = sim(model_name, 'StopTime', '50', 'SrcWorkspace', 'base');
        
        % Estrazione dal nuovo blocco error_stima_ekf per la funzione di costo
        if isprop(simOut, 'error_stima_ekf') || isfield(simOut, 'error_stima_ekf')
            error_struct = simOut.error_stima_ekf;
        else
            error_struct = evalin('base', 'error_stima_ekf');
        end
        
        if isstruct(error_struct) && isfield(error_struct, 'signals')
            err_data = error_struct.signals.values;
        elseif isprop(error_struct, 'Data') || isfield(error_struct, 'Data')
            err_data = error_struct.Data;
        else
            err_data = error_struct; 
        end
        cost = mean(err_data.^2, 'all');
    catch ME
        cost = 999999; 
    end
end