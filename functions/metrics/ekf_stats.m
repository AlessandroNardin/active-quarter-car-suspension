%% User setting: exclude first X percent of dataset
exclude_pct = 10;   % example: exclude first 10% of samples

%% Extract filtered estimate, covariance, and true trajectory
x = out.reg_xprev.Data;      % filtered state trajectory
P = out.reg_Pprev.Data;      % filtered covariance trajectory
xtrue = out.reg_xtrue.Data;  % true state trajectory

Nfull = size(x,3);
state_names = {'x_1','x_2','x_3','x_4'};
nStates = size(x,1);

%% Compute starting index after excluding first X percent
exclude_pct = max(0, min(exclude_pct, 100));
start_idx = floor(exclude_pct/100 * Nfull) + 1;

if start_idx > Nfull
    error('exclude_pct is too large: no samples remain after exclusion.');
end

%% Keep only the retained portion
x = x(:,:,start_idx:end);
P = P(:,:,start_idx:end);
xtrue = xtrue(:,:,start_idx:end);

N = size(x,3);
t = start_idx:Nfull;   % original sample numbering

fprintf('Excluded first %.2f%% of dataset (%d samples).\n', exclude_pct, start_idx-1);
fprintf('Using samples %d to %d (%d samples).\n', start_idx, Nfull, N);

%% Filtered error
efilt = x - xtrue;

%% Metrics on filtered trajectory
mean_err = zeros(nStates,1);
max_err  = zeros(nStates,1);
rmse_err = zeros(nStates,1);

for k = 1:nStates
    e = squeeze(efilt(k,1,:));
    mean_err(k) = mean(e);
    max_err(k)  = max(abs(e));
    rmse_err(k) = sqrt(mean(e.^2));
end

metrics_table = table( ...
    state_names(:), mean_err, max_err, rmse_err, ...
    'VariableNames', {'State','MeanError','MaxAbsError','RMSE'});

disp('Filtered trajectory metrics (after initial exclusion):')
disp(metrics_table)

%% Filtered NEES
nees = zeros(N,1);
for i = 1:N
    e = squeeze(efilt(:,1,i));
    Pi = squeeze(P(:,:,i));
    nees(i) = e' / Pi * e;
end

mean_nees = mean(nees);
fprintf('Mean filtered NEES = %.6f\n', mean_nees);

%% Plot 1: true vs filtered
figure('Name','True vs filtered states','Color','w');
for k = 1:nStates
    subplot(nStates,1,k);
    plot(t, squeeze(xtrue(k,1,:)), 'k', 'LineWidth', 1.5); hold on;
    plot(t, squeeze(x(k,1,:)), 'b', 'LineWidth', 1.2);
    grid on;
    ylabel(state_names{k});
    if k == 1
        title(sprintf('True and filtered states (excluding first %.1f%%)', exclude_pct));
        legend('True', 'Filtered x_{k|k}', 'Location', 'best');
    end
end
xlabel('Sample');

%% Plot 2: filtered signed error
figure('Name','Filtered state error','Color','w');
for k = 1:nStates
    subplot(nStates,1,k);
    plot(t, squeeze(efilt(k,1,:)), 'b', 'LineWidth', 1.3); hold on;
    yline(0, 'k--', 'LineWidth', 1.0);
    grid on;
    ylabel(['e_' num2str(k)]);
    if k == 1
        title(sprintf('Filtered state error: x_{k|k} - x_{true} (excluding first %.1f%%)', exclude_pct));
        legend('Filtered error', 'Zero line', 'Location', 'best');
    end
end
xlabel('Sample');

%% Plot 3: absolute filtered error
figure('Name','Absolute filtered error','Color','w');
for k = 1:nStates
    subplot(nStates,1,k);
    plot(t, abs(squeeze(efilt(k,1,:))), 'b', 'LineWidth', 1.3);
    grid on;
    ylabel(['|e_' num2str(k) '|']);
    if k == 1
        title(sprintf('Absolute filtered state error (excluding first %.1f%%)', exclude_pct));
    end
end
xlabel('Sample');

%% Plot 4: filtered covariance diagonals
figure('Name','Filtered covariance diagonals','Color','w');
for k = 1:nStates
    subplot(nStates,1,k);
    plot(t, squeeze(P(k,k,:)), 'b', 'LineWidth', 1.4);
    grid on;
    ylabel(['P(' num2str(k) ',' num2str(k) ')']);
    if k == 1
        title(sprintf('Filtered covariance diagonals (excluding first %.1f%%)', exclude_pct));
    end
end
xlabel('Sample');

%% Plot 5: filtered NEES
figure('Name','Filtered NEES','Color','w');
plot(t, nees, 'm', 'LineWidth', 1.4); hold on;
yline(mean_nees, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Sample');
ylabel('NEES');
title(sprintf('Filtered-state NEES (excluding first %.1f%%)', exclude_pct));
legend('NEES', 'Mean NEES', 'Location', 'best');