% =========================================================================
% SCRIPT: pf_statistical_validation.m
% =========================================================================
% Purpose: 
%   This script evaluates the health and statistical consistency of the 
%   Particle Filter (PF). It analyzes the effective particle population 
%   (N_eff) to detect sample depletion, processes measurement innovations, 
%   and verifies if the residuals satisfy the empirical 2-sigma boundary criteria.
% =========================================================================

% Safety check: Verify that simulation data 'out' exists in the workspace
if ~exist('out', 'var')
    error('Simulation data missing. Please run the PF model and save the results in "out" first.');
end

fprintf('Extracting and realigning Particle Filter diagnostic signals...\n');

% Extract raw timeseries data from the Simulink output structure
time_raw = out.tout;
Neff_raw = out.Neff_history.Data;   % N_eff history over time
res_raw  = out.residui_pf.Data;     % Measurement residuals (z - z_hat)

% Remove trailing singleton dimensions from matrix structures
Neff_data = squeeze(Neff_raw);
res_data  = squeeze(res_raw);

% Enforce time-along-rows matrix layout for residuals
if size(res_data, 1) == 3 && size(res_data, 2) ~= 3
    res_data = res_data';
end

% Synchronize array lengths based on N_eff samples reference
N_samples = length(Neff_data);
if size(res_data, 1) ~= N_samples
    N_samples = min([N_samples, size(res_data, 1)]);
    Neff_data = Neff_data(1:N_samples);
    res_data  = res_data(1:N_samples, :);
end

% Regenerate synchronized time vector
time = linspace(time_raw(1), time_raw(end), N_samples)';

fprintf('PF data successfully loaded! Total samples analyzed: %d\n\n', N_samples);

%% =========================================================================
% DIAGNOSTIC 1: EFFECTIVE PARTICLE POPULATION EVALUATION (N_eff)
% =========================================================================
figure('Name', 'PF Diagnostics - Effective Particles (Neff)', 'Color', 'w');

% Plot dynamic N_eff profile
plot(time, Neff_data, 'Color', [0.2 0.6 0.2], 'LineWidth', 1);
hold on;

% Compute average steady-state N_eff value
mean_neff = mean(Neff_data);

% Nominal particle population configuration (synchronized with params.m)
N_nominal = 1000; 
resampling_threshold = 0.5 * N_nominal; % Standard critical limit line (500)

% Plot diagnostic references (Resampling threshold and mean profile)
plot(time, resampling_threshold * ones(size(time)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Resampling Threshold (50%)');
plot(time, mean_neff * ones(size(time)), 'b-', 'LineWidth', 1.5, 'DisplayName', sprintf('Mean N_{eff}: %.1f', mean_neff));

grid on;
xlim([0, min(50, time(end))]); % Zoom on first 50s to inspect typical sawtooth profile
title('Particle Deprivation Analysis (N_{eff})');
xlabel('Time [s]'); ylabel('Number of Effective Particles');
legend('Location', 'best');

%% =========================================================================
% DIAGNOSTIC 2: STOCHASTIC MEASUREMENT RESIDUALS ANALYSIS
% =========================================================================
figure('Name', 'PF Diagnostics - Residuals Analysis', 'Color', 'w');
sensor_names = {'Linear Potentiometer', 'Body Accelerometer', 'Wheel Accelerometer'};

for idx = 1:3
    subplot(3, 1, idx)
    
    % Plot noise with transparency to reveal stochastic probability density density
    plot(time, res_data(:, idx), 'Color', [0.2 0.2 0.5 0.15], 'LineWidth', 0.5);
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
    fprintf('--- PF Statistical Analysis: Sensor %d (%s) ---\n', idx, sensor_names{idx});
    fprintf('Residual Mean (Optimal if ~0)      : %.6f\n', res_mean);
    fprintf('Residual Standard Deviation (\\sigma) : %.6f\n', res_std);
    fprintf('Samples inside 2-\\sigma Band (Target ~95%%): %.2f%%\n\n', percentage_inside);
end