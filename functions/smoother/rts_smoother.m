% Extract data from sim output
x = out.eval_ekf_x_k.Data;
xpred = out.eval_ekf_x_kp1.Data;
P = out.eval_ekf_P_k.Data;
Ppred = out.eval_ekf_P_kp1.Data;

N = size(x,3);

% Compute F for each step
F = zeros(4,4,N);
for i = 1:N
    F(:,:,i) = suspension_F(x(:,:,i), plant_param, ekf_param);
end

% Preallocation
C = zeros(4,4,N);
xsmooth = zeros(4,1,N);
Psmooth = zeros(4,4,N);

% Init of last smooth values
xsmooth(:,:,N) = x(:,:,N);
Psmooth(:,:,N) = P(:,:,N);

for i = N-1:-1:1
    C(:,:,i) = P(:,:,i) * F(:,:,i)' / Ppred(:,:,i);
    xsmooth(:,:,i) = x(:,:,i) + C(:,:,i) * (xsmooth(:,:,i+1) - xpred(:,:,i));
    Psmooth(:,:,i) = P(:,:,i) + C(:,:,i) * (Psmooth(:,:,i+1) - Ppred(:,:,i)) * C(:,:,i)';
end

% Retrieve true trajectory
xtrue = out.disc_eval_true_state.Data;

%% Metrics
t = 1:N;
state_names = {'x_1','x_2','x_3','x_4'};
nStates = size(x,1);

esmooth = xsmooth - xtrue;

mean_err = zeros(nStates,1);
max_err  = zeros(nStates,1);
rmse_err = zeros(nStates,1);

for k = 1:nStates
    e = squeeze(esmooth(k,1,:));
    mean_err(k) = mean(e);
    max_err(k)  = max(abs(e));
    rmse_err(k) = sqrt(mean(e.^2));
end

metrics_table = table( ...
    state_names(:), mean_err, max_err, rmse_err, ...
    'VariableNames', {'State','MeanError','MaxAbsError','RMSE'});

disp('RTS smoother metrics:')
disp(metrics_table)

%% NEES
nees = zeros(N,1);
for i = 1:N
    e = squeeze(esmooth(:,1,i));
    Pi = squeeze(Psmooth(:,:,i));
    nees(i) = e' / Pi * e;
end
mean_nees = mean(nees);

fprintf('Mean NEES = %.6f\n', mean_nees);

%% Plot 1: filtered vs smoothed
figure('Name','Overlay filtered vs smoothed states','Color','w');
for k = 1:nStates
    subplot(nStates,1,k);
    plot(t, squeeze(x(k,1,:)), 'b', 'LineWidth', 1.2); hold on;
    plot(t, squeeze(xsmooth(k,1,:)), 'r--', 'LineWidth', 1.4);
    grid on;
    ylabel(state_names{k});
    if k == 1
        title('Filtered and smoothed states');
        legend('Filtered x_{k|k}', 'Smoothed x_{k|N}', 'Location', 'best');
    end
end
xlabel('Sample');

%% Plot 2: true vs filtered vs smoothed
figure('Name','True vs filtered vs smoothed states','Color','w');
for k = 1:nStates
    subplot(nStates,1,k);
    plot(t, squeeze(xtrue(k,1,:)), 'k', 'LineWidth', 1.5); hold on;
    plot(t, squeeze(x(k,1,:)), 'b', 'LineWidth', 1.2);
    plot(t, squeeze(xsmooth(k,1,:)), 'r--', 'LineWidth', 1.4);
    grid on;
    ylabel(state_names{k});
    if k == 1
        title('True, filtered, and smoothed states');
        legend('True', 'Filtered x_{k|k}', 'Smoothed x_{k|N}', 'Location', 'best');
    end
end
xlabel('Sample');

%% Plot 3: smoother signed error
figure('Name','Smoothed state error','Color','w');
for k = 1:nStates
    subplot(nStates,1,k);
    plot(t, squeeze(esmooth(k,1,:)), 'r', 'LineWidth', 1.3); hold on;
    yline(0, 'k--', 'LineWidth', 1.0);
    grid on;
    ylabel(['e_' num2str(k)]);
    if k == 1
        title('Smoothed state error: x_{smooth} - x_{true}');
        legend('Smoothed error', 'Zero line', 'Location', 'best');
    end
end
xlabel('Sample');

%% Plot 4: absolute smoother error
figure('Name','Absolute smoother error','Color','w');
for k = 1:nStates
    subplot(nStates,1,k);
    plot(t, abs(squeeze(esmooth(k,1,:))), 'r', 'LineWidth', 1.3);
    grid on;
    ylabel(['|e_' num2str(k) '|']);
    if k == 1
        title('Absolute smoothed state error');
    end
end
xlabel('Sample');

%% Plot 5: smoothed covariance diagonals
figure('Name','Smoothed covariance diagonals','Color','w');
for k = 1:nStates
    subplot(nStates,1,k);
    plot(t, squeeze(Psmooth(k,k,:)), 'r--', 'LineWidth', 1.4);
    grid on;
    ylabel(['P(' num2str(k) ',' num2str(k) ')']);
    if k == 1
        title('Smoothed covariance diagonals');
    end
end
xlabel('Sample');

%% Plot 6: NEES
figure('Name','Smoother NEES','Color','w');
plot(t, nees, 'm', 'LineWidth', 1.4); hold on;
yline(mean_nees, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Sample');
ylabel('NEES');
title('Smoothed-state NEES');
legend('NEES','Mean NEES','Location','best');