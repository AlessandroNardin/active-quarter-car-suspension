% =========================================================================
% SCRIPT: pf_covariance_optimization_2.m (AGGIORNATO CON error_stima_pf)
% =========================================================================
% Purpose: 
%   Ottimizzazione globale dei parametri del Particle Filter (q_scale, epsilon)
%   su una finestra di 50s utilizzando GlobalSearch e fmincon in parallelo.
%   Aggiornato per estrarre l'errore dal nuovo blocco Outport: error_stima_pf
% =========================================================================
clear variables; close all; clc;

fprintf('=== INIZIALIZZAZIONE OTTIMIZZAZIONE GLOBALE PARTICLE FILTER ===\n');
run('params.m');

FLAG_NOMINAL_TUNING = true;
if FLAG_NOMINAL_TUNING  
    perturbed_plant_param = plant_param; 
end
assignin('base', 'perturbed_plant_param', perturbed_plant_param);

model_name = 'eval_all'; 
load_system(model_name);

% FORZIAMO LA MODALITÀ NORMAL (Massima stabilità sui core paralleli)
set_param(model_name, 'SimulationMode', 'normal');
save_system(model_name);
fprintf('Modello "%s" blindato in modalità NORMAL.\n', model_name);

%% 1. CONFIGURAZIONE SPAZIO DI RICERCA & APERTURA PARALLELO
start_params = [1.0, 0.01]; % [q_scale iniziale, epsilon iniziale]
lb = [0.001, 1e-5];        % Bound inferiori
ub = [100.0, 0.5];         % Bound superiori

if isempty(gcp('nocreate'))
    fprintf('Apertura del pool parallelo per calcolo gradienti...\n');
    parpool; 
end

% Opzioni fmincon: passo di perturbazione macroscopico per battere il rumore stocastico
options = optimoptions('fmincon', ...
                       'Algorithm', 'interior-point', ...
                       'Display', 'iter', ...
                       'MaxIterations', 30, ...    
                       'UseParallel', true, ...
                       'FiniteDifferenceStepSize', 0.05); 

% CONFIGURAZIONE GLOBAL SEARCH COMPLETA
gs = GlobalSearch('Display', 'iter', ...
                  'NumTrialPoints', 30, ...      
                  'NumStageOnePoints', 15);      

problem = createOptimProblem('fmincon', ...
    'objective', @(params) pf_cost_function_full(params, pf_param, model_name, perturbed_plant_param), ...
    'x0', start_params, 'lb', lb, 'ub', ub, 'options', options);

%% 2. ESECUZIONE OTTIMIZZAZIONE GLOBALE
fprintf('\nLancio ottimizzazione su finestra da 50 secondi. Macinazione in corso...\n');
tic;
[best_params, min_error] = run(gs, problem);
opt_time = toc;

opt_q_scale = best_params(1);
opt_epsilon = best_params(2);

fprintf('\n======================= FAST GLOBAL OPTIMIZATION RESULTS =======================\n');
fprintf('Optimization completed in %.1f seconds on a 50s window.\n', opt_time);
fprintf('OPTIMAL PARAMETERS FOUND:\n');
fprintf(' -> Optimal q_scale (Q Multiplier)     : %f\n', opt_q_scale);
fprintf(' -> Optimal epsilon (Resampling Jitter): %f\n', opt_epsilon);
fprintf('====================================================================\n');

%% 3. VALIDAZIONE FINALE ESTESA SUI 500 SECONDI
fprintf('\nLaunching final validation run over the FULL 500-second horizon...\n');

% Applica i parametri ottimi trovati alla struttura finale
pf_param.Q = pf_param.Q * opt_q_scale;
pf_param.L_Q = chol(pf_param.Q, 'lower');
pf_param.epsilon = opt_epsilon;

assignin('base', 'pf_param', pf_param);
assignin('base', 'perturbed_plant_param', perturbed_plant_param);

% Esegui la simulazione finale a 500s
simOut_final = sim(model_name, 'StopTime', '500', 'SrcWorkspace', 'base');

% Estrazione dal NUOVO blocco error_stima_pf
if isprop(simOut_final, 'error_stima_pf') || isfield(simOut_final, 'error_stima_pf')
    err_packet = simOut_final.error_stima_pf;
else
    err_packet = evalin('base', 'error_stima_pf');
end

if isprop(err_packet, 'Data') || isfield(err_packet, 'Data')
    err_data = err_packet.Data;
else
    err_data = err_packet;
end

mse_validation  = mean(err_data.^2, 'all');
rmse_validation = sqrt(mse_validation);

fprintf('\n======================= FINAL 500s VALIDATION VERDICT =======================\n');
fprintf(' VALIDATION SCENARIO            : NOMINAL SYSTEM\n');
fprintf(' Total Mean Squared Error (MSE)  : %f\n', mse_validation);
fprintf(' Total Root Mean Squared Error (RMSE): %f\n', rmse_validation);
fprintf('=============================================================================\n');

%% =========================================================================
% FUNZIONE DI COSTO LOCALE (FINESTRA A 50 SECONDI)
% =========================================================================
function cost = pf_cost_function_full(params, pf_param_base, model_name, plant_param_scen)
    assignin('base', 'perturbed_plant_param', plant_param_scen);
    
    pf_param = pf_param_base;
    pf_param.Q = pf_param.Q * params(1);
    pf_param.L_Q = chol(pf_param.Q, 'lower');
    pf_param.epsilon = params(2);
    
    assignin('base', 'pf_param', pf_param);
    
    try
        % Finestra di saggio standard a 50 secondi
        simOut = sim(model_name, 'StopTime', '50', 'SrcWorkspace', 'base');
        
        % Estrazione dal NUOVO blocco error_stima_pf per la funzione di costo
        if isprop(simOut, 'error_stima_pf') || isfield(simOut, 'error_stima_pf')
            error_struct = simOut.error_stima_pf;
        else
            error_struct = evalin('base', 'error_stima_pf');
        end
        
        if isstruct(error_struct) && isfield(error_struct, 'signals')
            err_data = error_struct.signals.values;
        elseif isprop(error_struct, 'Data') || isfield(error_struct, 'Data')
            err_data = error_struct.Data;
        else
            err_data = error_struct; 
        end
        
        cost = mean(err_data.^2, 'all');
        
    catch
        cost = 999999; % Penalità in caso di divergenza esplosiva
    end
end