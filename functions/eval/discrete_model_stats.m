% Compact evaluation script - returns stats struct and prints summary + per-state lines
xtrue = squeeze(out.disc_eval_true_state.Data);   % [n x N]
xpred = squeeze(out.disc_eval_disc_state.Data);   % [n x N]

% Basic check
if ~isequal(size(xtrue), size(xpred))
    error('xtrue and xpred must have the same size [n x N].');
end

% Error (true - pred)
e = xtrue - xpred;        % [n x N]
abs_e = abs(e);
[n, N] = size(e);

% Per-state stats
rmse_state     = sqrt(mean(e.^2, 2));    % [n x 1]
mean_state     = mean(e, 2);             % [n x 1]
max_state      = max(e, [], 2);          % [n x 1]
min_state      = min(e, [], 2);          % [n x 1]
meanabs_state  = mean(abs_e, 2);         % [n x 1]
maxabs_state   = max(abs_e, [], 2);      % [n x 1]
minabs_state   = min(abs_e, [], 2);      % [n x 1]

% Overall stats
rmse_all    = sqrt(mean(e(:).^2));
mean_all    = mean(e(:));
max_all     = max(e(:));
min_all     = min(e(:));
meanabs_all = mean(abs_e(:));
maxabs_all  = max(abs_e(:));
minabs_all  = min(abs_e(:));

% Pack into struct
stats.N_total = N;
stats.per_state.rmse    = rmse_state;
stats.per_state.mean    = mean_state;
stats.per_state.max     = max_state;
stats.per_state.min     = min_state;
stats.per_state.meanabs = meanabs_state;
stats.per_state.maxabs  = maxabs_state;
stats.per_state.minabs  = minabs_state;
stats.overall.rmse    = rmse_all;
stats.overall.mean    = mean_all;
stats.overall.max     = max_all;
stats.overall.min     = min_all;
stats.overall.meanabs = meanabs_all;
stats.overall.maxabs  = maxabs_all;
stats.overall.minabs  = minabs_all;

% Display concise summary (similar style to EKF script)
fprintf('Using %d samples (no exclusion), n = %d states\n', N, n);
fprintf('Overall RMSE: %g, Mean: %g, Max: %g, Min: %g\n', ...
    rmse_all, mean_all, max_all, min_all);
fprintf('Overall Mean abs: %g, Max abs: %g, Min abs: %g\n', ...
    meanabs_all, maxabs_all, minabs_all);

% Per-state lines (similar style to EKF script)
for i = 1:n
    fprintf('State %d: RMSE=%g, Mean=%g, Max=%g, Min=%g, MeanAbs=%g\n', ...
        i, rmse_state(i), mean_state(i), max_state(i), min_state(i), meanabs_state(i));
end

% Plot error in 4 vertical subplots (one per state)
if n ~= 4
    error('This plotting snippet expects exactly 4 states.');
end

sample_idx = 1:N;

figure;
for i = 1:4
    subplot(4,1,i);
    plot(sample_idx, e(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(sprintf('e_{%d}', i));
    title(sprintf('State %d Error', i));
    
    if i == 1
        sgtitle('Discrete model error: true - pred');
    end
    
    if i == 4
        xlabel('Sample index');
    end
end