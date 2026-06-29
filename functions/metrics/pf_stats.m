N = size(x,3);
t = 1:N;
nStates = size(x,1);
state_names = compose('x_%d', 1:nStates).';

% Filtered error as [nStates x N]
efilt = reshape(out.error_stima.Data, nStates, N);

% Metrics on filtered trajectory
mean_err = mean(efilt, 2);
max_err  = max(abs(efilt), [], 2);
rmse_err = sqrt(mean(efilt.^2, 2));

metrics_table = table( ...
    state_names, mean_err, max_err, rmse_err, ...
    'VariableNames', {'State','MeanError','MaxAbsError','RMSE'});

disp('Filtered trajectory metrics:')
disp(metrics_table)

% Filtered NEES
nees = zeros(N,1);
for i = 1:N
    e = efilt(:,i);
    nees(i) = e' / P(:,:,i) * e;
end

mean_nees = mean(nees);
fprintf('Mean filtered NEES = %.6f\n', mean_nees);

% Plot 1: true vs filtered
figure('Name','True vs filtered states','Color','w');
for k = 1:nStates
    subplot(nStates,1,k)
    plot(t, reshape(xtrue(k,1,:),1,[]), 'k', 'LineWidth', 1.5); hold on
    plot(t, reshape(x(k,1,:),1,[]), 'b', 'LineWidth', 1.2)
    grid on
    ylabel(state_names{k})
    if k == 1
        title('True and filtered states')
        legend('True', 'Filtered x_{k|k}', 'Location', 'best')
    end
end
xlabel('Sample')

% Plot 2: filtered signed error
figure('Name','Filtered state error','Color','w');
for k = 1:nStates
    subplot(nStates,1,k)
    plot(t, efilt(k,:), 'b', 'LineWidth', 1.3); hold on
    yline(0, 'k--', 'LineWidth', 1.0)
    grid on
    ylabel(sprintf('e_%d', k))
    if k == 1
        title('Filtered state error: x_{k|k} - x_{true}')
        legend('Filtered error', 'Zero line', 'Location', 'best')
    end
end
xlabel('Sample')

% Plot 3: absolute filtered error
figure('Name','Absolute filtered error','Color','w');
for k = 1:nStates
    subplot(nStates,1,k)
    plot(t, abs(efilt(k,:)), 'b', 'LineWidth', 1.3)
    grid on
    ylabel(sprintf('|e_%d|', k))
    if k == 1
        title('Absolute filtered state error')
    end
end
xlabel('Sample')

% Plot 4: filtered covariance diagonals
figure('Name','Filtered covariance diagonals','Color','w');
for k = 1:nStates
    subplot(nStates,1,k)
    plot(t, reshape(P(k,k,:),1,[]), 'b', 'LineWidth', 1.4)
    grid on
    ylabel(sprintf('P(%d,%d)', k, k))
    if k == 1
        title('Filtered covariance diagonals')
    end
end
xlabel('Sample')

% Plot 5: filtered NEES
figure('Name','Filtered NEES','Color','w');
plot(t, nees, 'm', 'LineWidth', 1.4); hold on
yline(mean_nees, 'k--', 'LineWidth', 1.0)
grid on
xlabel('Sample')
ylabel('NEES')
title('Filtered-state NEES')
legend('NEES', 'Mean NEES', 'Location', 'best')