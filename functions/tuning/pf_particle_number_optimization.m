% =========================================================================
% SCRIPT: pf_sweep_N_and_plot.m
% =========================================================================
% Purpose:
%   Sweep pf_param.N from B to C with increment D,
%   simulate for A seconds for each N,
%   compute MSE/RMSE for each of the 4 states,
%   and plot error vs N.
%
% Assumption:
%   error_stima_pf has size [4 x T]
%   -> 4 rows = states
%   -> T columns = time samples
% =========================================================================

clear variables; close all; clc;

fprintf('=== PARTICLE FILTER N SWEEP ===\n');
run('params.m');

%% USER SETTINGS
A = 500;      % simulation time [s]
B = 50;      % minimum N
C = 150;    % maximum N
D = 10;      % increment in N

model_name = 'eval_all';

FLAG_NOMINAL_TUNING = false;
if FLAG_NOMINAL_TUNING
    perturbed_plant_param = plant_param;
end

%% PREPARE SWEEP
N_values = B:D:C;
num_cases = numel(N_values);

rmse_states = nan(num_cases, 4);
mse_states  = nan(num_cases, 4);
total_rmse  = nan(num_cases, 1);
total_mse   = nan(num_cases, 1);

load_system(model_name);
set_param(model_name, 'SimulationMode', 'normal');
save_system(model_name);

assignin('base', 'perturbed_plant_param', perturbed_plant_param);

fprintf('Model "%s" loaded in NORMAL mode.\n', model_name);
fprintf('Running %d simulations from N = %d to N = %d with step %d...\n', ...
    num_cases, B, C, D);

%% MAIN LOOP
for k = 1:num_cases
    N_current = N_values(k);
    fprintf('\n[%d/%d] Testing N = %d ... ', k, num_cases, N_current);

    try
        pf_param_k = rebuild_pf_with_new_N(pf_param, N_current);

        assignin('base', 'pf_param', pf_param_k);
        assignin('base', 'perturbed_plant_param', perturbed_plant_param);

        simOut = sim(model_name, 'StopTime', num2str(A), 'SrcWorkspace', 'base');

        % Extract error_stima_pf
        if isprop(simOut, 'error_stima_pf') || isfield(simOut, 'error_stima_pf')
            err_packet = simOut.error_stima_pf;
        else
            err_packet = evalin('base', 'error_stima_pf');
        end

        % Convert output to numeric array
        if isstruct(err_packet) && isfield(err_packet, 'signals')
            err_data = err_packet.signals.values;
        elseif isprop(err_packet, 'Data') || isfield(err_packet, 'Data')
            err_data = err_packet.Data;
        else
            err_data = err_packet;
        end

        % Ensure numeric 2D matrix
        err_data = squeeze(err_data);

        if isempty(err_data)
            error('error_stima_pf is empty');
        end

        if ndims(err_data) ~= 2
            error('error_stima_pf is not a 2D matrix after squeeze. ndims = %d', ndims(err_data));
        end

        if size(err_data,1) ~= 4
            error('error_stima_pf must be [4 x T], found [%d x %d].', ...
                  size(err_data,1), size(err_data,2));
        end

        % Per-state metrics: one row per state
        mse_k  = mean(err_data.^2, 2);   % [4 x 1]
        rmse_k = sqrt(mse_k);            % [4 x 1]

        mse_states(k, :)  = mse_k.';
        rmse_states(k, :) = rmse_k.';

        % Overall metrics across all states and all time samples
        total_mse(k)  = mean(err_data.^2, 'all');
        total_rmse(k) = sqrt(total_mse(k));

        fprintf('OK | RMSE = [%.5f %.5f %.5f %.5f]', ...
            rmse_k(1), rmse_k(2), rmse_k(3), rmse_k(4));

    catch ME
        fprintf('FAILED\n');
        fprintf(2, '   Reason: %s\n', ME.message);
    end
end

%% BUILD RESULTS TABLE
results_table = table( ...
    N_values(:), ...
    rmse_states(:,1), rmse_states(:,2), rmse_states(:,3), rmse_states(:,4), ...
    total_rmse, ...
    mse_states(:,1), mse_states(:,2), mse_states(:,3), mse_states(:,4), ...
    total_mse, ...
    'VariableNames', { ...
    'N', ...
    'RMSE_x1', 'RMSE_x2', 'RMSE_x3', 'RMSE_x4', 'RMSE_total', ...
    'MSE_x1',  'MSE_x2',  'MSE_x3',  'MSE_x4',  'MSE_total'});

disp(results_table);

%% BEST N VALUES
[best_rmse_x1, idx1] = min(rmse_states(:,1));
[best_rmse_x2, idx2] = min(rmse_states(:,2));
[best_rmse_x3, idx3] = min(rmse_states(:,3));
[best_rmse_x4, idx4] = min(rmse_states(:,4));
[best_rmse_tot, idxT] = min(total_rmse);

fprintf('\n=== BEST RESULTS ===\n');
fprintf('State 1 best: N = %d, RMSE = %.6f\n', N_values(idx1), best_rmse_x1);
fprintf('State 2 best: N = %d, RMSE = %.6f\n', N_values(idx2), best_rmse_x2);
fprintf('State 3 best: N = %d, RMSE = %.6f\n', N_values(idx3), best_rmse_x3);
fprintf('State 4 best: N = %d, RMSE = %.6f\n', N_values(idx4), best_rmse_x4);
fprintf('Total   best: N = %d, RMSE = %.6f\n', N_values(idxT), best_rmse_tot);

%% PLOT 1: PER-STATE RMSE VS N
fig1 = figure('Color', 'w');
plot(N_values, rmse_states(:,1), '-o', 'LineWidth', 1.5, 'DisplayName', 'State 1'); hold on;
plot(N_values, rmse_states(:,2), '-s', 'LineWidth', 1.5, 'DisplayName', 'State 2');
plot(N_values, rmse_states(:,3), '-d', 'LineWidth', 1.5, 'DisplayName', 'State 3');
plot(N_values, rmse_states(:,4), '-^', 'LineWidth', 1.5, 'DisplayName', 'State 4');
grid on;
xlabel('Number of particles N');
ylabel('RMSE');
title(sprintf('Per-state RMSE vs N (simulation time = %g s)', A));
legend('Location', 'best');
hold off;

%% PLOT 2: TOTAL RMSE VS N
fig2 = figure('Color', 'w');
plot(N_values, total_rmse, '-o', 'LineWidth', 1.8, 'DisplayName', 'Total RMSE');
grid on;
xlabel('Number of particles N');
ylabel('Total RMSE');
title(sprintf('Total RMSE vs N (simulation time = %g s)', A));
legend('Location', 'best');

%% PLOT 3: PER-STATE MSE VS N
fig3 = figure('Color', 'w');
plot(N_values, mse_states(:,1), '-o', 'LineWidth', 1.5, 'DisplayName', 'State 1'); hold on;
plot(N_values, mse_states(:,2), '-s', 'LineWidth', 1.5, 'DisplayName', 'State 2');
plot(N_values, mse_states(:,3), '-d', 'LineWidth', 1.5, 'DisplayName', 'State 3');
plot(N_values, mse_states(:,4), '-^', 'LineWidth', 1.5, 'DisplayName', 'State 4');
grid on;
xlabel('Number of particles N');
ylabel('MSE');
title(sprintf('Per-state MSE vs N (simulation time = %g s)', A));
legend('Location', 'best');
hold off;

%% SAVE RESULTS
writetable(results_table, 'pf_sweep_results.csv');
saveas(fig1, 'pf_rmse_per_state_vs_N.png');
saveas(fig2, 'pf_total_rmse_vs_N.png');
saveas(fig3, 'pf_mse_per_state_vs_N.png');

fprintf('\n=== SWEEP COMPLETED ===\n');
fprintf('Saved files:\n');
fprintf(' - pf_sweep_results.csv\n');
fprintf(' - pf_rmse_per_state_vs_N.png\n');
fprintf(' - pf_total_rmse_vs_N.png\n');
fprintf(' - pf_mse_per_state_vs_N.png\n');

%% =========================================================================
% HELPER FUNCTION
% =========================================================================
function pf_param_out = rebuild_pf_with_new_N(pf_param_base, N_new)
    pf_param_out = pf_param_base;
    pf_param_out.N = N_new;

    init_chol = chol(pf_param_out.P_init);
    init_noise = randn(pf_param_out.N, 4);
    pf_param_out.particles_init = repmat(pf_param_out.x_init', pf_param_out.N, 1) + init_noise * init_chol;
    pf_param_out.weights_init   = (1 / pf_param_out.N) * ones(pf_param_out.N, 1);
end