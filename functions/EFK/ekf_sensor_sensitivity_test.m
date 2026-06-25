% =========================================================================
% SCRIPT: ekf_sensor_sensitivity_test.m
% =========================================================================
% Purpose: 
%   This script initializes the workspace with the default filter settings
%   defined in 'params.m' and runs a single baseline simulation. It then 
%   computes the Root Mean Squared Error (RMSE) to quantify the tracking 
%   accuracy of the nominal state estimation.
% =========================================================================

%% 1. ENVIRONMENT INITIALIZATION & PARAMETERS LOADING
% This call safely executes the clear commands inside params.m
run('params.m'); 

%% 2. SIMULATION EXECUTION
% Select the desired model file to simulate
model_name = 'eval_ekf.slx'; 

fprintf('Starting single simulation for nominal model: %s...\n', model_name);
simOut_nominal = sim(model_name);

%% 3. PERFORMANCE METRICS CALCULATION
% Extract state estimation error and compute Mean Squared Error (MSE)
mse_nominal  = mean(simOut_nominal.error_stima.Data.^2, 'all');
rmse_nominal = sqrt(mse_nominal);

%% 4. BENCHMARK VERDICT PRINTING
fprintf('\n==================== NOMINAL BENCHMARK VERDICT ====================\n');
fprintf('Simulated Model                        : %s\n', model_name);
fprintf('Root Mean Squared Error (RMSE) Stima   : %f\n', rmse_nominal);
fprintf('==================================================================\n');