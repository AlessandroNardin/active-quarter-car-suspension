% =========================================================================
% SCRIPT: pf_covariance_optimization.m
% =========================================================================
% Purpose: 
%   This script performs an automated multi-variable optimization of the 
%   Particle Filter (PF) hyperparameters using the derivative-free 'fminsearch' 
%   algorithm. It simultaneously tunes the process noise 
%   scaling factor (q_scale) and the particle jittering factor (epsilon).
%
% Acceleration Strategy:
%   To drastically cut optimization time, the Simulink model is executed 
%   in 'Accelerator' mode. The cost function restricts evaluation to the 
%   first 20 seconds of the horizon, ensuring immediate evaluations.
% =========================================================================

clear variables; close all; clc;

% 1. Load baseline parameters (ensures workspace consistency)
fprintf('Initializing baseline parameters from params.m...\n');
run('params.m');

model_name = 'eval_pf';
fprintf('Configuring model "%s" to ACCELERATOR mode...\n', model_name);
load_system(model_name);
set_param(model_name, 'SimulationMode', 'accelerator');

% Retrieve simulation properties from params.m
N_particles = pf_param.N;
fprintf('Dynamic particle population detected for tuning: %d\n', N_particles);

% Save base EKF process covariance for the cost function scaling
Q_base = filter_param.Q; 

%% 2. OPTIMIZATION SETUP
% Initial guess [q_scale, epsilon] centered around manual tuning results
start_coeffs = [5.0, 0.0010]; 

% Solver options configuration
options = optimset('Display', 'iter', ...
                   'MaxIter', 50, ...   
                   'TolX', 1e-3, ...
                   'TolFun', 1e-4);

fprintf('\nLaunching fminsearch for optimal q_scale and epsilon tracking...\n');
fprintf('NOTE: The first iteration will require C-code compilation time.\n');
fprintf('Subsequent iterations will evaluate instantaneously.\n\n');

%% 3. OPTIMIZATION RUN
tic;
[best_coeffs, min_mse] = fminsearch(@(coeffs) pf_cost_function(coeffs, Q_base, pf_param, model_name), ...
                                    start_coeffs, options);
elapsed_time = toc;
min_rmse = sqrt(min_mse);

%% 4. SIMULINK RESTORATION
fprintf('\nRestoring Simulink model back to Normal mode...\n');
set_param(model_name, 'SimulationMode', 'normal');
save_system(model_name);

%% 5. OPTIMIZATION VERDICT PRINTING
fprintf('\n======================= OPTIMIZATION RESULTS =======================\n');
fprintf('Optimization successfully completed in %.1f seconds!\n\n', elapsed_time);
fprintf('OPTIMAL HYPERPARAMETERS FOUND (Ready to be copied into params.m):\n');
fprintf(' -> Optimal Q-matrix scale factor (q_scale) : %.4f\n', best_coeffs(1));
fprintf(' -> Optimal Jitter factor (epsilon)          : %.6f\n', best_coeffs(2));
fprintf('\nTEST WINDOW PERFORMANCE (20 seconds validation horizon):\n');
fprintf(' -> Absolute Minimum Estimation RMSE (PF)    : %.6f\n', min_rmse);
fprintf('====================================================================\n');

%% =========================================================================
% LOCAL FUNCTION: COVARIANCE TUNING COST FUNCTION (Evaluation Horizon: 20s)
% =========================================================================
function cost = pf_cost_function(coeffs, Q_base, pf_param_base, model_name)
    current_q_scale = coeffs(1);
    current_eps     = coeffs(2);
    
    % SAFETY BOUNDARY WALL: Prevent negative, null, or unstable exploration steps
    if current_q_scale < 0.1 || current_eps < 1e-6 || current_eps > 0.05
        cost = 999999; 
        return;
    end
    
    % Copy baseline structure (keeps the optimal sensor matrix R inherited from EKF)
    pf_param = pf_param_base;
    
    % Map dynamic optimization coefficients into the parameter structure
    pf_param.q_scale = current_q_scale;
    pf_param.Q       = current_q_scale * Q_base;
    pf_param.L_Q     = chol(pf_param.Q, 'lower'); 
    pf_param.epsilon = current_eps;
    
    % Inject updated structures directly into the base workspace for Simulink access
    assignin('base', 'pf_param', pf_param);
    
    % Seed the random number generator to suppress stochastic simulation noise
    rng(100); 
    
    try
        % Simulate a restricted 20-second window to sense state-tracking quality
        simOut = sim(model_name, 'StopTime', '20', 'SrcWorkspace', 'base');
        
        % Compute the Mean Squared Error (MSE) of the state estimation
        err_data = simOut.error_stima.Data; 
        cost     = mean(err_data.^2, 'all');
        
    catch ME
        cost = 999999; % High penalization barrier in case of numerical crash/divergence
    end
end