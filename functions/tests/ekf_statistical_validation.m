% =========================================================================
% SCRIPT: ekf_statistical_validation.m (UPDATED ARCHITECTURE)
% =========================================================================
% Purpose: 
%   This script evaluates the analytical convergence and statistical 
%   consistency of the Extended Kalman Filter (EKF) based on the new
%   Simulink Outport architecture. Optimized to prevent graphical freezing.
% =========================================================================
clear variables; close all; clc;

% Ponticello di compatibilità per intercettare l'oggetto di output
if exist('simOut_opt', 'var')
    out = simOut_opt;
elseif exist('simOut', 'var')
    out = simOut;
end

% Safety check: Verifica che l'oggetto principale esista
if ~exist('out', 'var')
    error('I dati di simulazione "out" non sono presenti nel workspace. Lancia la simulazione prima di eseguire lo script.');
end

fprintf('Extracting diagnostic signals from new Simulink Outports...\n');

%% 0. ESTRAZIONE DIRETTA DAI NUOVI OUTPORT
try
    % 1. Vettore Tempo (Standard dal solutore)
    time_raw = out.tout;
    
    % 2. Estrazione Matrice Covarianza P (dal blocco out.eval_ekf_P_k)
    if isprop(out, 'eval_ekf_P_k') || isfield(out, 'eval_ekf_P_k')
        P_packet = out.eval_ekf_P_k;
    else
        P_packet = evalin('base', 'eval_ekf_P_k');
    end
    if isprop(P_packet, 'Data') || isfield(P_packet, 'Data'), P_raw = P_packet.Data; else, P_raw = P_packet; end

    % 3. Estrazione Residui/Errori di innovazione (dal blocco out.eval_ekf_e_k)
    if isprop(out, 'eval_ekf_e_k') || isfield(out, 'eval_ekf_e_k')
        res_packet = out.eval_ekf_e_k;
    else
        res_packet = evalin('base', 'eval_ekf_e_k');
    end
    if isprop(res_packet, 'Data') || isfield(res_packet, 'Data'), res_raw = res_packet.Data; else, res_raw = res_packet; end

catch ME
    error('Errore nell''estrazione dei nuovi blocchi Outport. Controlla che i nomi nel blocco (eval_ekf_P_k, eval_ekf_e_k) corrispondano al Workspace.');
end

%% 1. COVARIANCE MATRIX P PROCESSING (Time along the third dimension)
N_steps_P = size(P_raw, 3);    
P_data    = zeros(N_steps_P, 4);  % Allocation for the 4 state variances
for t = 1:N_steps_P
    P_data(t, :) = diag(P_raw(:, :, t))';
end

%% 2. MEASUREMENT RESIDUALS PROCESSING
res_data_mat = squeeze(res_raw);
% Forza il layout dei residui (tempo sulle righe, sensori sulle colonne)
if size(res_data_mat, 1) == 3 && size(res_data_mat, 2) ~= 3
    res_data_mat = res_data_mat';
elseif ndims(res_raw) == 3 && size(res_raw, 1) == 3
    res_data_mat = permute(res_raw, [3, 1, 2]);
    res_data_mat = squeeze(res_data_mat);
end

%% 3. TIME RE-SAMPLING AND ARRAY ALIGNMENT
N_samples = size(P_data, 1);
if size(res_data_mat, 1) ~= N_samples
    N_samples = min([N_samples, size(res_data_mat, 1)]);
    P_data    = P_data(1:N_samples, :);
    res_data_mat  = res_data_mat(1:N_samples, :);
end

% Rigenerazione asse dei tempi perfettamente coerente in colonna
time_col = linspace(time_raw(1), time_raw(end), N_samples)';
fprintf('EKF data successfully loaded! Total samples analyzed: %d\n', N_samples);

%% =========================================================================
% DIAGNOSTIC 1: COVARIANCE MATRIX P CONVERGENCE (State Uncertainty)
% =========================================================================
figure('Name', 'EKF Diagnostics - Covariance P Convergence', 'Color', 'w');
subplot(2,1,1)
plot(time_col(1:10:end), P_data(1:10:end, 1), 'LineWidth', 1.5, 'DisplayName', 'Var(x_1) - Suspended Pos.');
hold on;
plot(time_col(1:10:end), P_data(1:10:end, 3), 'LineWidth', 1.5, 'DisplayName', 'Var(x_3) - Wheel Pos.');
grid on; 
xlim([0, min(5, time_col(end))]); 
title('Convergence of Position State Variances (Matrix P)');
xlabel('Time [s]'); ylabel('Uncertainty [(cm)^2]');
legend('Location', 'best');

subplot(2,1,2)
plot(time_col(1:10:end), P_data(1:10:end, 2), 'LineWidth', 1.5, 'DisplayName', 'Var(x_2) - Suspended Vel.');
hold on;
plot(time_col(1:10:end), P_data(1:10:end, 4), 'LineWidth', 1.5, 'DisplayName', 'Var(x_4) - Wheel Vel.');
grid on; 
xlim([0, min(5, time_col(end))]);
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
    
    current_res = res_data_mat(:, idx);
    
    % Calcolo delle metriche statistiche REALI su tutto l'array
    res_mean = mean(current_res);
    res_std  = std(current_res);
    
    samples_inside = sum(abs(current_res - res_mean) <= 2*res_std);
    percentage_inside = (samples_inside / length(time_col)) * 100;
    
    % PLOT ALLEGGERITO: Disegniamo 1 punto ogni 5 senza trasparenza alpha per fluidità grafico
    plot(time_col(1:5:end), current_res(1:5:end), 'Color', [0.4 0.4 0.4], 'LineWidth', 0.5);
    hold on;
    
    % Bande 2-sigma
    plot(time_col, res_mean + 2*res_std * ones(size(time_col)), 'r--', 'LineWidth', 1.5);
    plot(time_col, res_mean - 2*res_std * ones(size(time_col)), 'r--', 'LineWidth', 1.5);
    
    grid on;
    title(sprintf('Residual: %s (Inside 2\\sigma: %.2f%%)', sensor_names{idx}, percentage_inside));
    xlabel('Time [s]'); ylabel('Error [z - z\_hat]');
    
    % Log di verifica a schermo
    fprintf('--- EKF Statistical Analysis: Sensor %d (%s) ---\n', idx, sensor_names{idx});
    fprintf('Residual Mean (Optimal if ~0)      : %.6f\n', res_mean);
    fprintf('Residual Standard Deviation (\\sigma) : %.6f\n', res_std);
    fprintf('Samples inside 2-\\sigma Band (Target ~95%%): %.2f%%\n\n', percentage_inside);
end