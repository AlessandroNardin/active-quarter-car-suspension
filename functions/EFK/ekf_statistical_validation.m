% =========================================================================
% SCRIPT: ekf_statistical_validation.m
% =========================================================================
% Purpose: 
%   This script evaluates the analytical convergence and statistical 
%   consistency of the Extended Kalman Filter (EKF). It processes the 3D 
%   state covariance matrix P (extracting state variances over time) and 
%   performs a whiteness test on the measurement innovations (residuals) 
%   against the empirical 2-sigma confidence boundaries.
% =========================================================================

% Safety check: Verify that simulation data 'out' exists in the workspace
if ~exist('out', 'var')
    error('Simulation data missing. Please run the EKF model and save the results in "out" first.');
end

fprintf('Extracting and realigning Extended Kalman Filter diagnostic signals...\n');

% Extract raw timeseries data from the Simulink output structure
time_raw = out.tout;
P_raw    = out.P_history.Data; % Expected dimension layout: [4 x 4 x Time]
res_raw  = out.residui.Data;   % Measurement residuals (innovations)

%% 1. COVARIANCE MATRIX P PROCESSING (Time along the third dimension)
N_steps_P = size(P_raw, 3);    
P_data    = zeros(N_steps_P, 4);  % Allocation for the 4 state variances

for t = 1:N_steps_P
    % Extract the diagonal elements (variances) of the 4x4 matrix at time step t
    P_data(t, :) = diag(P_raw(:, :, t))';
end

%% 2. MEASUREMENT RESIDUALS PROCESSING
res_data = squeeze(res_raw);

% Enforce time-along-rows matrix layout for residuals
if size(res_data, 1) == 3 && size(res_data, 2) ~= 3
    res_data = res_data';
elseif ndims(res_raw) == 3 && size(res_raw, 1) == 3
    % If time happens to be on the third dimension for residuals as well
    res_data = permute(res_raw, [3, 1, 2]);
    res_data = squeeze(res_data);
end

%% 3. TIME RE-SAMPLING AND ARRAY ALIGNMENT
N_samples = size(P_data, 1);

if size(res_data, 1) ~= N_samples
    N_samples = min([N_samples, size(res_data, 1)]);
    P_data    = P_data(1:N_samples, :);
    res_data  = res_data(1:N_samples, :);
end

% Regenerate synchronized time vector
time = linspace(time_raw(1), time_raw(end), N_samples)';

fprintf('EKF data successfully loaded! Total samples analyzed: %d\n', N_samples);
fprintf('-> Final Time dimension    : [%d x 1]\n', length(time));
fprintf('-> Final P_data dimension  : [%d x 4]\n', size(P_data, 1));
fprintf('-> Final res_data dimension: [%d x 3]\n\n', size(res_data, 1));

%% =========================================================================
% DIAGNOSTIC 1: COVARIANCE MATRIX P CONVERGENCE (State Estimation Uncertainty)
% =========================================================================
figure('Name', 'EKF Diagnostics - Covariance P Convergence', 'Color', 'w');

subplot(2,1,1)
plot(time, P_data(:,1), 'LineWidth', 1.5, 'DisplayName', 'Var(x_1) - Suspended Pos.');
hold on;
plot(time, P_data(:,3), 'LineWidth', 1.5, 'DisplayName', 'Var(x_3) - Wheel Pos.');
grid on; 
xlim([0, min(5, time(end))]); % Zoom on first 5s to inspect initial transient behavior
title('Convergence of Position State Variances (Matrix P)');
xlabel('Time [s]'); ylabel('Uncertainty [(cm)^2]');
legend('Location', 'best');

subplot(2,1,2)
plot(time, P_data(:,2), 'LineWidth', 1.5, 'DisplayName', 'Var(x_2) - Suspended Vel.');
hold on;
plot(time, P_data(:,4), 'LineWidth', 1.5, 'DisplayName', 'Var(x_4) - Wheel Vel.');
grid on; 
xlim([0, min(5, time(end))]);
title('Convergence of Velocity State Variances (Matrix P)');
xlabel('Time [s]'); ylabel('Uncertainty [(cm/s)^2]');
legend('Location', 'best');

%% =========================================================================
% DIAGNOSTIC 2: ADVANCED MEASUREMENT RESIDUALS STATISTICAL ANALYSIS
% =========================================================================
figure('Name', 'EKF Diagnostics - Advanced Residuals Analysis', 'Color', 'w');
sensor_names = {'Linear Potentiometer', 'Body Accelerometer', 'Wheel Accelerometer'};

for idx = 1:3
    subplot(3, 1, idx)
    
    % Plot with alpha transparency to mitigate overplotting across 500k samples
    plot(time, res_data(:, idx), 'Color', [0.4 0.4 0.4 0.15], 'LineWidth', 0.5);
    hold on;
    
    % Compute residual tracking statistics
    res_mean = mean(res_data(:, idx));
    res_std  = std(res_data(:, idx));
    
    % Compute the exact percentage of samples falling within the 2-sigma band
    samples_inside = sum(abs(res_data(:, idx) - res_mean) <= 2*res_std);
    percentage_inside = (samples_inside / length(time)) * 100;
    
    % Plot upper and lower empirical 2-sigma confidence boundaries
    plot(time, res_mean + 2*res_std * ones(size(time)), 'r--', 'LineWidth', 1.5);
    plot(time, res_mean - 2*res_std * ones(size(time)), 'r--', 'LineWidth', 1.5);
    
    grid on;
    title(sprintf('Residual: %s (Inside 2\\sigma: %.2f%%)', sensor_names{idx}, percentage_inside));
    xlabel('Time [s]'); ylabel('Error [z - z\_hat]');
    
    % Print detailed statistical log in the MATLAB Command Window
    fprintf('--- EKF Statistical Analysis: Sensor %d (%s) ---\n', idx, sensor_names{idx});
    fprintf('Residual Mean (Optimal if ~0)      : %.6f\n', res_mean);
    fprintf('Residual Standard Deviation (\\sigma) : %.6f\n', res_std);
    fprintf('Samples inside 2-\\sigma Band (Target ~95%%): %.2f%%\n\n', percentage_inside);
end