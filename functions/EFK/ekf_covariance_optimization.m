%% 
% =========================================================================
% SCRIPT: ekf_covariance_optimization.m
% =========================================================================
% Purpose: 
%   This script performs an automated multi-variable optimization of the 
%   Extended Kalman Filter (EKF) measurement noise covariance matrix (R) 
%   using the derivative-free 'fminsearch' algorithm. 
%   It tunes the individual noise scaling coefficients for each sensor
%   to minimize the overall state estimation error.
% =========================================================================

clear variables; close all; clc;

% 1. Load nominal sensor parameters from parameters file
fprintf('Initializing baseline parameters from params.m...\n');
run('params.m'); 

pot_base_noise = lpot_param.noise_var;
acc_base_noise = acc_param.noise_var;

%% 2. OPTIMIZATION SETUP
% Initial guess for the scaling coefficients [pot_scale, body_acc_scale, wheel_acc_scale]
start_coeffs = [1.0, 1.0, 1.0];

% Solver options configuration (maximum 150 iterations)
options = optimset('Display', 'iter', ...
                   'MaxIter', 100, ...
                   'TolX', 1e-4);

fprintf('\nLaunching fminsearch for optimal EKF sensor noise scale tracking...\n');

%% 3. OPTIMIZATION RUN
tic;
[best_coeffs, min_error] = fminsearch(@(coeffs) ekf_cost_function(coeffs, pot_base_noise, acc_base_noise, ekf_param), ...
                                      start_coeffs, options);
elapsed_time = toc;
min_rmse = sqrt(min_error);

%% 4. OPTIMIZATION VERDICT PRINTING
fprintf('\n======================= OPTIMIZATION RESULTS =======================\n');
fprintf('Optimization successfully completed in %.1f seconds!\n\n', elapsed_time);
fprintf('OPTIMAL COEFFICIENTS FOUND (Ready to be copied into params.m):\n');
fprintf(' -> Optimal Multiplier - Sensor_1 : %.4f\n', best_coeffs(1));
fprintf(' -> Optimal Multiplier - Sensor_2     : %.4f\n', best_coeffs(2));
fprintf(' -> Optimal Multiplier - Sensor_3   : %.4f\n', best_coeffs(3));
fprintf('\nTEST WINDOW PERFORMANCE:\n');
fprintf(' -> Minimum Total Mean Squared Error   : %.6f\n', min_error);
fprintf(' -> Minimum Total Estimation RMSE      : %.6f\n', min_rmse);
fprintf('====================================================================\n');

%% =========================================================================
% LOCAL FUNCTION: COVARIANCE TUNING COST FUNCTION
% =========================================================================
function cost = ekf_cost_function(coeffs, pot_noise, acc_noise, ekf_param_base)
    % SAFETY BOUNDARY WALL: Prevent negative, null, or unstable exploration steps
    if any(coeffs <= 0.001)
        cost = 999999; 
        return;
    end
    
    % Copy baseline structure to prevent permanent data corruption
    ekf_param = ekf_param_base;
    
    % Update the R matrix of the filter with the new scaled parameters
    ekf_param.R = diag([pot_noise * coeffs(1), ...
                        acc_noise * coeffs(2), ...
                        acc_noise * coeffs(3)]);
    
    % Inject updated structures directly into the base workspace for Simulink access
    assignin('base', 'ekf_param', ekf_param);
    
    try
        % Find the Simulink model path and run the simulation
        model_path = which('eval_ekf.slx');
        simOut = sim(model_path);
        
        % Extract estimation error data from simulation results
        err_data = simOut.error_stima.Data; 
        
        % Calculate the total Mean Squared Error (MSE) across all states
        cost = mean(err_data.^2, 'all');
        
    catch ME
        cost = 999999; % High penalization barrier in case of numerical crash/divergence
    end
end