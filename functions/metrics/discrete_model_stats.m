% Script for evaluation of discrete model
xtrue = out.disc_true_state.Data;
xpred = out.disc_pred_state.Data;

% xtrue and xpred are assumed to be size: [4 x 1 x N]
% If needed, squeeze the middle singleton dimension
xtrue = squeeze(xtrue);   % now [4 x N]
xpred = squeeze(xpred);   % now [4 x N]

% Check dimensions
assert(isequal(size(xtrue), size(xpred)), 'xtrue and xpred must have the same size.');

% Number of samples and states
[nx, N] = size(xtrue);

% Optional: discard an initial transient portion
discard_frac = 0.0;   % set e.g. 0.1 to discard first 10%
k0 = floor(discard_frac * N) + 1;

xtrue_use = xtrue(:, k0:end);
xpred_use = xpred(:, k0:end);

% Error definition
e = xtrue_use - xpred_use;      % [nx x Nuse]
abs_e = abs(e);
sq_e = e.^2;
e_norm = sqrt(sum(e.^2, 1));    % Euclidean norm over states at each time

% Per-state statistics
stats = struct();
for i = 1:nx
    ei = e(i, :);
    stats.state(i).mean = mean(ei);
    stats.state(i).variance = var(ei, 0);
    stats.state(i).std = std(ei, 0);
    stats.state(i).min = min(ei);
    stats.state(i).max = max(ei);
    stats.state(i).min_abs = min(abs(ei));
    stats.state(i).max_abs = max(abs(ei));
    stats.state(i).mae = mean(abs(ei));
    stats.state(i).mse = mean(ei.^2);
    stats.state(i).rmse = sqrt(stats.state(i).mse);
    stats.state(i).median = median(ei);
    stats.state(i).iqr = iqr(ei);
end

% Global statistics across all states and all times
e_all = e(:);

stats.global.mean = mean(e_all);
stats.global.variance = var(e_all, 0);
stats.global.std = std(e_all, 0);
stats.global.min = min(e_all);
stats.global.max = max(e_all);
stats.global.min_abs = min(abs(e_all));
stats.global.max_abs = max(abs(e_all));
stats.global.mae = mean(abs(e_all));
stats.global.mse = mean(e_all.^2);
stats.global.rmse = sqrt(stats.global.mse);
stats.global.median = median(e_all);
stats.global.iqr = iqr(e_all);

% Time-wise statistics over the 4 states
stats.time.mean_error = mean(e, 1);
stats.time.rms_error = sqrt(mean(e.^2, 1));
stats.time.max_abs_error = max(abs(e), [], 1);
stats.time.min_error = min(e, [], 1);
stats.time.max_error = max(e, [], 1);
stats.time.error_norm = e_norm;

% Whiteness test on each state error sequence
% Requires Econometrics Toolbox for lbqtest, but the script will still run
stats.whiteness = struct();
for i = 1:nx
    ei = e(i, :)';
    try
        [h, pValue, Qstat] = lbqtest(ei);
        stats.whiteness.state(i).h = h;
        stats.whiteness.state(i).pValue = pValue;
        stats.whiteness.state(i).Qstat = Qstat;
    catch
        stats.whiteness.state(i).h = NaN;
        stats.whiteness.state(i).pValue = NaN;
        stats.whiteness.state(i).Qstat = NaN;
    end
end

% Display summary
fprintf('---- Global error statistics (all states, all retained samples) ----\n');
fprintf('Mean error      : %.6g\n', stats.global.mean);
fprintf('Variance        : %.6g\n', stats.global.variance);
fprintf('Std deviation   : %.6g\n', stats.global.std);
fprintf('Min error       : %.6g\n', stats.global.min);
fprintf('Max error       : %.6g\n', stats.global.max);
fprintf('Mean abs error  : %.6g\n', stats.global.mae);
fprintf('MSE             : %.6g\n', stats.global.mse);
fprintf('RMSE            : %.6g\n', stats.global.rmse);
fprintf('Median error    : %.6g\n', stats.global.median);
fprintf('IQR             : %.6g\n', stats.global.iqr);

fprintf('\n---- Per-state RMSE ----\n');
for i = 1:nx
    fprintf('State %d: RMSE = %.6g, MAE = %.6g, mean = %.6g, var = %.6g\n', ...
        i, stats.state(i).rmse, stats.state(i).mae, stats.state(i).mean, stats.state(i).variance);
end

fprintf('\n---- Whiteness test (Ljung-Box) ----\n');
for i = 1:nx
    fprintf('State %d: h = %d, p = %.6g\n', i, stats.whiteness.state(i).h, stats.whiteness.state(i).pValue);
end

% Optional plots
t = 1:size(e,2);

figure;
subplot(4,1,1);
plot(t, e(1,:), 'LineWidth', 1); grid on;
ylabel('e_1');

subplot(4,1,2);
plot(t, e(2,:), 'LineWidth', 1); grid on;
ylabel('e_2');

subplot(4,1,3);
plot(t, e(3,:), 'LineWidth', 1); grid on;
ylabel('e_3');

subplot(4,1,4);
plot(t, e(4,:), 'LineWidth', 1); grid on;
ylabel('e_4');
xlabel('Time index');

figure;
plot(t, e_norm, 'LineWidth', 1.2); grid on;
xlabel('Time index');
ylabel('||e(t)||_2');
title('State error norm over time');