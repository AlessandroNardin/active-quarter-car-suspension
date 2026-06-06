clear all;
close all;
clc;

%% PLANT PARAMETERS (SUSPENSION)
plant_param = struct();

plant_param.ms    = 350;    % kg
plant_param.mu    = 50;     % kg

plant_param.ks0   = 200;    % N/cm
plant_param.alpha = 0.1;    % N/(cm^3)
plant_param.bs    = 15;     % (N*s)/cm

plant_param.kt    = 1800;   % N/cm
plant_param.bt    = 1.5;    % (N*s)/cm


%% PLANT INITIAL STATE (PERTURBATION OF THE EQ)
zs0    = 0;   % cm
zu0    = 0;   % cm
zsdot0 = 0;   % cm/s
zudot0 = 0;   % cm/s


%% ACCELEROMETER PARAMETERS (ST AIS25BA)
g_cms2          = 981;     % [cm/s^2]
fs_hz           = 1000;    % [Hz]
full_scale_g    = 3.85;    % [g]
num_bits        = 16;      % [bit]
noise_density_ug = 30;     % [ug/sqrt(Hz)]

acc_param = struct();

acc_param.delay       = 0;                                 % [s]
acc_param.sample_time = 1 / fs_hz;                         % [s]

acc_param.max_acc = full_scale_g * g_cms2;                 % [cm/s^2]
acc_param.min_acc = -acc_param.max_acc;                    % [cm/s^2]

acc_param.quant_step = (acc_param.max_acc - acc_param.min_acc) / (2^num_bits);

acc_param.noise_var = ((((noise_density_ug * 1e-6) * g_cms2)^2) * (fs_hz / 2)); % [(cm/s^2)^2]

%% ADDED FOR TEST
%acc_param.noise_var = (0.05)^2;

%% LINEAR POTENTIOMETER PARAMETERS
lpot_param = struct();

lpot_param.low_b      = -10;     % cm
lpot_param.high_b     = 10;      % cm
lpot_param.noise_var  = (0.1)^2;    % cm^2
lpot_param.sample_freq = 1000;   % Hz
lpot_param.n_bit      = 16;


%% ROAD
r_param = struct();

r_param.rz_var    = (0.2)^2;   % cm^2
r_param.rzdot_var = (2)^2;     % cm^2/s^2


%% FILTER PARAMETERS
filter_param = struct();

filter_param.freq     = 1000;                          % [Hz]
filter_param.sample_t = 1 / filter_param.freq;        % [s]

filter_param.Q = diag([r_param.rz_var r_param.rzdot_var]);
filter_param.R = diag([lpot_param.noise_var acc_param.noise_var acc_param.noise_var]);

filter_param.P_init = eye(4)*10;
filter_param.x_init = [0 0 0 0]';


%{
%% SIMPLE ROAD PARAMETERS [UNUSED]
low_freq_m  = 1e-2; %[cycles/meter]
high_freq_m = 1e1;  %[cycles/meter]
Gdn0 = 128;         %[m^3]
car_speed = 100;    %[cm/s]

simple_road_param = struct();

simple_road_param.low_freq  = low_freq_m * 1e-2 * car_speed;   %[Hz]
simple_road_param.high_freq = high_freq_m * 1e-2 * car_speed;  %[Hz]

simple_road_param.road_variance = Gdn0 * 9.99e-9 * 1e4;        %[cm^2]

simple_road_param.road_dot_variance = ( ...
    (4 * pi^2 * simple_road_param.road_variance) * ...
    (simple_road_param.low_freq^2 + simple_road_param.high_freq^2 + ...
     simple_road_param.low_freq * simple_road_param.high_freq) ...
) / 3;

simple_road_param.sample_t = 1 / 10000;

simple_road_param.road_noise_power     = simple_road_param.road_variance * simple_road_param.sample_t;
simple_road_param.road_dot_noise_power = simple_road_param.road_dot_variance * simple_road_param.sample_t;


%% ROAD PARAMETERS [UNUSED]
road_param = struct();

road_param.n0         = 0.1;     % reference spatial frequency [cycles/m]
road_param.Gqn0       = 16e-7;   % NEED TO FIND VALUE FOR THIS
road_param.w          = 2;       % ISO waviness exponent
road_param.low_freq   = 1e-2;    % min spatial frequency [cycles/m]
road_param.high_freq  = 1e1;     % max spatial frequency [cycles/m]
road_param.L          = 2000;    % road length [m]
road_param.k          = 3;       % class-dependent coefficient
road_param.vehicle_speed = 2;    % vehicle speed [m/s]
road_param.seed       = 213412;  % random seed

road_param.delta_n = (2^road_param.k) / road_param.L;

road_param.num_bins = floor((road_param.high_freq - road_param.low_freq) / road_param.delta_n);

road_param.freq = road_param.low_freq + (0:road_param.num_bins-1) * road_param.delta_n;

road_param.Gq = road_param.Gqn0 * (road_param.freq / road_param.n0) .^ (-road_param.w);

road_param.A = sqrt(road_param.Gq * road_param.delta_n);

rng(road_param.seed);
road_param.phi = 2*pi*rand(size(road_param.freq));

road_param.zr_variance    = road_param.Gqn0 * 0.999;
road_param.zrdot_variance = road_param.Gqn0 * (2*pi*2*0.1)^2 * 9.9;
%}