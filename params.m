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
T_delay = 266e-6;               % 266 microsecondi
g = 9.81;                       % [m/s^2] gravity constant
fs = 1000;                       % [Hz] sampling frequency
FS_g = 3.85;                     % [g] full-scale range
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
lpot_param.sample_freq = 1000;       % Hz
lpot_param.n_bit = 32;

%% road parameters
road_param = struct();
road_param.n0 = 0.1;              % reference spatial frequency [cycles/m]
road_param.Gqn0 = 16e-7;             % NEED TO FIND VALUE FOR THIS
road_param.w = 2;                 % ISO waviness exponent
road_param.low_freq = 1e-2;       % min spatial frequency [cycles/m]
road_param.high_freq = 1e1;       % max spatial frequency [cycles/m]
road_param.L = 2000;              % road length [m]
road_param.k = 3;                 % class-dependent coefficient
road_param.vehicle_speed = 2;     % vehicle speed [m/s]
road_param.seed = 213412;         % random seed

% Paper-style bin width
road_param.delta_n = (2^road_param.k) / road_param.L;

% Number of full bins inside the frequency range
road_param.num_bins = floor((road_param.high_freq - road_param.low_freq) / road_param.delta_n);

% Frequency vector: use bin centers
%freq = road_param.low_freq + ((0:num_bins-1) + 0.5) * road_param.delta_n;

% Optional: if you prefer left edges instead of centers, use this instead
road_param.freq = road_param.low_freq + (0:road_param.num_bins-1) * road_param.delta_n;

% ISO 8608 PSD evaluated at the chosen frequencies
road_param.Gq = road_param.Gqn0 * (road_param.freq / road_param.n0) .^ (-road_param.w);

% Amplitude vector from PSD times bin width
road_param.A = sqrt(road_param.Gq * road_param.delta_n);

% Random phases uniformly distributed in [0, 2*pi)
rng(road_param.seed);
road_param.phi = 2*pi*rand(size(road_param.freq));

%% Partially Hardcoded, they depend on more parameters than just those listed as they were simplified in this specific case.
road_param.zr_variance = road_param.Gqn0 * 0.999
road_param.zrdot_variance = road_param.Gqn0 * (2*pi*2*0.1)^2 * 9.9

%% filter parameters
filter_param = struct();
filter_param.freq = 500; % hz
filter_param.sample_t = 1 / filter_param.freq;
filter_param.Q = [ road_param.zr_variance 0; 0 road_param.zrdot_variance]; % based on stuff that is to be verified
filter_param.R = [ lpot_param.noise_std_var^2 0 0; 0 var_acc 0; 0 0 var_acc]; % capire come costruirla [cov disturbo di misura]



