%%
% =========================================================================
% SCRIPT: pf_measurement_covariance_optimization.m
% =========================================================================
% Purpose:
%   This script performs an automated multi-variable optimization of the
%   Particle Filter (PF) measurement noise covariance matrix (R)
%   using the derivative-free 'fminsearch' algorithm.
%
%   It tunes the individual noise scaling coefficients for each measurement
%   channel, exactly in the same spirit as the EKF covariance optimization,
%   but acting on pf_param.R instead.
%
% Notes:
%   - The PF structure is modified ONLY through pf_param.R
%   - All other PF parameters remain unchanged
%   - The Simulink model used is 'eval_all'
%   - The optimization cost is the total Mean Squared Error (MSE)
%     computed from simOut.error_stima.Data
%   - The simulation horizon is restricted to 50 seconds
% =========================================================================
clear variables; close all; clc;

%% 1. LOAD BASELINE PARAMETERS
fprintf('Initializing baseline parameters from params.m...\n');
run('params.m');

% Store baseline PF measurement covariance
R_base = pf_param.R;

%% 2. OPTIMIZATION SETUP
% Initial guess for the scaling coefficients of each measurement variance
% [Sensor_1_scale, Sensor_2_scale, Sensor_3_scale]
start_coeffs = [157.5201, 12.4961, 461.2471];

% Solver options configuration
options = optimset('Display', 'iter', ...
                   'MaxIter', 50, ...
                   'TolX', 1e-4, ...
                   'TolFun', 1e-4);

fprintf('\nLaunching fminsearch for optimal PF measurement covariance scaling...\n');

%% 3. OPTIMIZATION RUN
tic;
[best_coeffs, min_error] = fminsearch(@(coeffs) pf_R_cost_function(coeffs, R_base, pf_param), ...
                                      start_coeffs, options);
elapsed_time = toc;
min_rmse = sqrt(min_error);

%% 4. OPTIMIZATION VERDICT PRINTING
fprintf('\n======================= OPTIMIZATION RESULTS =======================\n');
fprintf('Optimization successfully completed in %.1f seconds!\n\n', elapsed_time);
fprintf('OPTIMAL COEFFICIENTS FOUND (Ready to be copied into params.m):\n');
fprintf(' -> Optimal Multiplier - Sensor_1 : %.4f\n', best_coeffs(1));
fprintf(' -> Optimal Multiplier - Sensor_2 : %.4f\n', best_coeffs(2));
fprintf(' -> Optimal Multiplier - Sensor_3 : %.4f\n', best_coeffs(3));

fprintf('\nOPTIMAL PF MEASUREMENT COVARIANCE MATRIX R:\n');
R_opt = diag(diag(R_base) .* best_coeffs(:));
disp(R_opt);

fprintf('\nTEST WINDOW PERFORMANCE (50 seconds validation horizon):\n');
fprintf(' -> Minimum Total Mean Squared Error : %.6f\n', min_error);
fprintf(' -> Minimum Total Estimation RMSE    : %.6f\n', min_rmse);
fprintf('====================================================================\n');

%% =========================================================================
% LOCAL FUNCTION: PF MEASUREMENT COVARIANCE TUNING COST FUNCTION
% =========================================================================
function cost = pf_R_cost_function(coeffs, R_base, pf_param_base)

    % SAFETY BOUNDARY WALL:
    % Coefficients must remain strictly positive and bounded
    if any(coeffs <= 0) || any(coeffs > 1000)
        cost = 999999;
        return;
    end

    % Copy baseline PF structure to avoid permanent corruption
    pf_param = pf_param_base;

    % Update ONLY the measurement covariance matrix R
    % Each diagonal element is independently scaled
    pf_param.R = diag(diag(R_base) .* coeffs(:));

    % Inject updated structure into base workspace for Simulink access
    assignin('base', 'pf_param', pf_param);

    try
        % Run the simulation over a restricted 50-second horizon
        simOut = sim('eval_all', 'StopTime', '50', 'SrcWorkspace', 'base');

        % Extract estimation error data and compute total MSE
        err_data = simOut.error_stima_pf.Data;
        cost = mean(err_data.^2, 'all');

    catch ME
        cost = 999999; % Penalize failures, divergence, or numerical crashes
    end
end