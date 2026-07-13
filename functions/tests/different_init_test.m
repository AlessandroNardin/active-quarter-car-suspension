clear variables; close all; clc;

run('params.m');

model_name = 'eval_all';
load_system(model_name);

% ============================ ADAPT HERE ============================
ekf_traj_name  = 'eval_ekf_x_k';
ekf_error_name = 'error_stima_ekf';
tStop = '50';

zs0_vals    =  -5  : 1 : 5;   % cm
zu0_vals    =  -5  : 1 : 5;   % cm
zsdot0_vals =  -2  : 1 :  2;   % cm/s
zudot0_vals =  -2  : 1 :  2;   % cm/s
% ==================================================================
% ====== HUGE NOTE: CHANGE THE TWO NAMES ABOVE IF THEY DIFFER ======
% ekf_traj_name  -> name of EKF state trajectory signal from Simulink
% ekf_error_name -> name of EKF error signal from Simulink
% ================================================================

[ZS0, ZU0, ZSDOT0, ZUDOT0] = ndgrid(zs0_vals, zu0_vals, zsdot0_vals, zudot0_vals);
all_ic = [ZS0(:), ZU0(:), ZSDOT0(:), ZUDOT0(:)];
nRuns = size(all_ic, 1);

traj = cell(nRuns, 1);
err  = cell(nRuns, 1);
tt   = cell(nRuns, 1);

for k = 1:nRuns
    plant_param_k = plant_param;
    plant_param_k.zs0    = all_ic(k,1);
    plant_param_k.zu0    = all_ic(k,2);
    plant_param_k.zsdot0 = all_ic(k,3);
    plant_param_k.zudot0 = all_ic(k,4);

    assignin('base', 'plant_param', plant_param_k);
    assignin('base', 'ekf_param', ekf_param);

    simOut = sim(model_name, 'StopTime', tStop, 'SrcWorkspace', 'base');

    % ============================ ADAPT HERE ============================
    % If these signals are not inside simOut, replace with:
    % traj_sig = evalin('base', ekf_traj_name);
    % err_sig  = evalin('base', ekf_error_name);
    traj_sig = simOut.(ekf_traj_name);
    err_sig  = simOut.(ekf_error_name);
    % ==================================================================

    [t1, y1] = get_data(traj_sig);
    [t2, y2] = get_data(err_sig);

    tt{k}   = t1;
    traj{k} = y1;
    err{k}  = y2;
end

idx_traj = find(~cellfun(@isempty, traj), 1);
idx_err  = find(~cellfun(@isempty, err), 1);

nTrajStates = size(traj{idx_traj}, 2);
nErrStates  = size(err{idx_err}, 2);

figure('Color','w');
for i = 1:nTrajStates
    subplot(nTrajStates,1,i); hold on; grid on;
    for k = 1:nRuns
        plot(tt{k}, traj{k}(:,i), 'LineWidth', 0.8);
    end
    ylabel(sprintf('\\hat{x}_{%d}', i));
    if i == 1
        title('EKF trajectories overlay');
    end
end
xlabel('Time [s]');

figure('Color','w');
for i = 1:nErrStates
    subplot(nErrStates,1,i); hold on; grid on;
    for k = 1:nRuns
        plot(tt{k}, err{k}(:,i), 'LineWidth', 0.8);
    end
    ylabel(sprintf('e_{%d}', i));
    if i == 1
        title('EKF error overlay');
    end
end
xlabel('Time [s]');

function [t, y] = get_data(sig)
    if isstruct(sig) && isfield(sig,'signals') && isfield(sig,'time')
        t = sig.time;
        y = sig.signals.values;
    elseif isprop(sig,'Time') && isprop(sig,'Data')
        t = sig.Time;
        y = sig.Data;
    elseif isfield(sig,'Time') && isfield(sig,'Data')
        t = sig.Time;
        y = sig.Data;
    else
        error('Unsupported signal format.')
    end

    t = t(:);
    if size(y,1) ~= numel(t) && size(y,2) == numel(t)
        y = y.';
    end
end