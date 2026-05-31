clear all;
close all;
clc;

%% plant parameters (suspension)
plant_param = struct();
plant_param.ms    = 350;        % kg
plant_param.mu    = 50;         % kg
plant_param.ks0   = 20000;      % N/m
plant_param.alpha = 1e5;        % N/(m^3)
plant_param.bs    = 1500;       % (N*s)/m
plant_param.kt    = 180000;     % N/m
plant_param.bt    = 150;        % (N*s)/m

%% plant initial state
zs0 = 0;                        % m
zu0 = 0;                        % m
zsdot0 = 0;                     % m/s
zudot0 = 0;                     % m/s

%% accelerometer parameters (ST AIS25BA)
g = 9.81;                       % [m/s^2] gravity constant
fs = 100;                       % [Hz] sampling frequency
FS_g = 7.7;                     % [g] full-scale range
N_bit = 16;                     % [bit] TDM data slot width
ND_ug = 30;                     % [ug/sqrt(Hz)] noise density

Ts_ZOH = 1 / fs;                % [s] ZOH sample time
acc_max = FS_g * g;             % [m/s^2] upper saturation limit
acc_min = -FS_g * g;            % [m/s^2] lower saturation limit
q_acc = (acc_max - acc_min) / (2^N_bit); % [m/s^2] quantization step

ND_SI = (ND_ug * 10^-6) * g;    % [m/s^2/sqrt(Hz)] noise density in SI
PSD = ND_SI^2;                  % [(m/s^2)^2/Hz] power spectral density
B_Nyq = fs / 2;                 % [Hz] Nyquist bandwidth
var_acc = PSD * B_Nyq;          % [(m/s^2)^2] white noise variance

%% linear potentiometer paramters
lpot_param = struct();
lpot_param.low_b = -0.05;           % m
lpot_param.high_b = 0.05;           % m
lpot_param.noise_std_var = 1*10^-4; % m
lpot_param.sample_freq = 100;       % Hz
lpot_param.n_bit = 32;
