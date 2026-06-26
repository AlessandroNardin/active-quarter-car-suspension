% Extract data from sim output
x = out.reg_xprev.Data;
xpred = out.reg_xpred.Data;
P = out.reg_Pprev.Data;
Ppred = out.reg_Ppred.Data;

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

% Time axis
t = 1:N;

% State names, edit if needed
state_names = {'x_1','x_2','x_3','x_4'};

figure('Name','RTS smoother states','Color','w');
for k = 1:4
    subplot(4,1,k);
    plot(t, squeeze(x(k,1,:)), 'b', 'LineWidth', 1.2); hold on;
    plot(t, squeeze(xsmooth(k,1,:)), 'r--', 'LineWidth', 1.4);
    if N > 1
        plot(t(1:end-1), squeeze(xpred(k,1,1:end-1)), 'k:', 'LineWidth', 1.0);
    end
    grid on;
    ylabel(state_names{k});
    if k == 1
        title('Filtered, predicted, and smoothed states');
        legend('Filtered x_{k|k}', 'Smoothed x_{k|N}', 'Predicted x_{k+1|k}', ...
            'Location', 'best');
    end
end
xlabel('Sample');

figure('Name','RTS smoother covariance','Color','w');
for k = 1:4
    subplot(4,1,k);
    plot(t, squeeze(P(k,k,:)), 'b', 'LineWidth', 1.2); hold on;
    plot(t, squeeze(Psmooth(k,k,:)), 'r--', 'LineWidth', 1.4);
    if N > 1
        plot(t(1:end-1), squeeze(Ppred(k,k,1:end-1)), 'k:', 'LineWidth', 1.0);
    end
    grid on;
    ylabel(['P(' num2str(k) ',' num2str(k) ')']);
    if k == 1
        title('Filtered, predicted, and smoothed covariance diagonals');
        legend('Filtered P_{k|k}', 'Smoothed P_{k|N}', 'Predicted P_{k+1|k}', ...
            'Location', 'best');
    end
end
xlabel('Sample');