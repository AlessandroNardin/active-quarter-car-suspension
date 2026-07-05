% User settings
exclude_pct = 60;   % percent of initial samples to exclude (0-100)

% Extract data from workspace (as in your snippet)
x_k      = out.eval_ekf_x_k.Data;       % expected size [n x 1 x N] or [n x N]
P_k      = out.eval_ekf_P_k.Data;       % not used for these stats but kept
x_true_k = out.eval_ekf_true_x_k.Data;  % expected size [n x 1 x N] or [n x N]

% Ensure shapes are [n x N]
x_k      = squeeze(x_k);        % now [n x N]
x_true_k = squeeze(x_true_k);   % now [n x N]

% Basic checks
[n, N] = size(x_true_k);
if size(x_k,1) ~= n || size(x_k,2) ~= N
    error('x_k and x_true_k must have same size [n x N].');
end

% Compute index to start after excluding first exclude_pct percent
if exclude_pct < 0 || exclude_pct >= 100
    error('exclude_pct must be in range [0, 100).');
end

start_idx = floor(N * (exclude_pct/100)) + 1;
if start_idx > N
    error('exclude_pct too large: no samples left after exclusion.');
end

% Select data after exclusion
x_k_f      = x_k(:, start_idx:end);        % [n x M]
x_true_k_f = x_true_k(:, start_idx:end);   % [n x M]
M = size(x_k_f,2);

% Error (signed): true - estimate
e = x_true_k_f - x_k_f;    % [n x M]
abs_e = abs(e);

% Per-state (per-row) statistics over time (after exclusion)
rmse_state = sqrt(mean(e.^2, 2));     % [n x 1] RMSE per state
mean_state = mean(e, 2);              % [n x 1] signed mean error per state
max_state  = max(e, [], 2);           % [n x 1] max signed error per state
min_state  = min(e, [], 2);           % [n x 1] min signed error per state

% Per-state absolute-error statistics
mean_abs_state = mean(abs_e, 2);      % [n x 1] mean absolute error per state
max_abs_state  = max(abs_e, [], 2);   % [n x 1] max absolute error per state
min_abs_state  = min(abs_e, [], 2);   % [n x 1] min absolute error per state

% Overall statistics across all states and time (after exclusion)
rmse_all      = sqrt(mean(e(:).^2));  % scalar
mean_all      = mean(e(:));           % signed mean
max_all       = max(e(:));            % signed max
min_all       = min(e(:));            % signed min

% Overall absolute-error stats
mean_abs_all  = mean(abs_e(:));
max_abs_all   = max(abs_e(:));
min_abs_all   = min(abs_e(:));

% Pack results into a struct for easy use
stats.exclude_pct = exclude_pct;
stats.N_total     = N;
stats.N_used      = M;
stats.start_idx   = start_idx;
stats.per_state.rmse         = rmse_state;
stats.per_state.mean         = mean_state;
stats.per_state.max          = max_state;
stats.per_state.min          = min_state;
stats.per_state.mean_abs     = mean_abs_state;
stats.per_state.max_abs      = max_abs_state;
stats.per_state.min_abs      = min_abs_state;
stats.overall.rmse       = rmse_all;
stats.overall.mean       = mean_all;
stats.overall.max        = max_all;
stats.overall.min        = min_all;
stats.overall.mean_abs   = mean_abs_all;
stats.overall.max_abs    = max_abs_all;
stats.overall.min_abs    = min_abs_all;

% Display concise summary
fprintf('Excluded first %g%% -> using %d of %d samples (start idx %d)\n', ...
    exclude_pct, M, N, start_idx);
fprintf('Overall RMSE: %g, Mean: %g, Max: %g, Min: %g\n', ...
    rmse_all, mean_all, max_all, min_all);
fprintf('Overall Mean abs: %g, Max abs: %g, Min abs: %g\n', ...
    mean_abs_all, max_abs_all, min_abs_all);

% (Optional) show per-state table
for i = 1:n
    fprintf('State %d: RMSE=%g, Mean=%g, Max=%g, Min=%g, MeanAbs=%g\n', ...
        i, rmse_state(i), mean_state(i), max_state(i), min_state(i), mean_abs_state(i));
end

% Plot error in 4 vertical subplots (one per state)
if n ~= 4
    error('This plotting snippet expects exactly 4 states.');
end

sample_idx = start_idx:N;

figure;
for i = 1:4
    subplot(4,1,i);
    plot(sample_idx, e(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(sprintf('e_{%d}', i));
    title(sprintf('State %d Error', i));
    
    if i == 1
        sgtitle(sprintf('EKF Error: true - estimate (after excluding first %g%%)', exclude_pct));
    end
    
    if i == 4
        xlabel('Sample index');
    end
end