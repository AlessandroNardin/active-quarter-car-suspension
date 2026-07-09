clear all;
close all;
clc;

random_seed = 42;
%% NOMINAL PLANT PARAMETERS (SUSPENSION)
plant_param = struct();
plant_param.ms    = 350;    % kg
plant_param.mu    = 50;     % kg
plant_param.ks0   = 200;    % N/cm
plant_param.alpha = 0.1;    % N/(cm^3)
plant_param.bs    = 15;     % (N*s)/cm
plant_param.kt    = 1800;   % N/cm
plant_param.bt    = 1.5;    % (N*s)/cm

%% NOMINAL PLANT INITIAL STATE
plant_param.zs0    = 0;   % cm
plant_param.zu0    = 0;   % cm
plant_param.zsdot0 = 0;   % cm/s
plant_param.zudot0 = 0;   % cm/s

%% PERTURBATED PLANT PARAMETER (SUSPENSION)
error_p = 5; % percentage
rng(random_seed + 21); 
perturbed_plant_param = struct();
perturbed_plant_param.ms    = (error_p*randn/100 + 1) * plant_param.ms;
perturbed_plant_param.mu    = (error_p*randn/100 + 1) * plant_param.mu;
perturbed_plant_param.ks0   = (error_p*randn/100 + 1) * plant_param.ks0;
perturbed_plant_param.alpha = (error_p*randn/100 + 1) * plant_param.alpha;
perturbed_plant_param.bs    = (error_p*randn/100 + 1) * plant_param.bs;
perturbed_plant_param.kt    = (error_p*randn/100 + 1) * plant_param.kt;
perturbed_plant_param.bt    = (error_p*randn/100 + 1) * plant_param.bt;
clear error_p;

%% INPUT NOISE PARAMETERS
u_noise_param = struct();
u_noise_param.u1_var = 5;
u_noise_param.u2_var = 5;

%% ACCELEROMETER PARAMETERS (ST AIS25BA)
acc_param = struct();
acc_param.freq      = 500;                                              % Hz
acc_param.sample_t  = 1 / acc_param.freq;                               % s
acc_param.max_acc   = 3776.85;                                          % cm/s^2
acc_param.min_acc   = -3776.85;                                         % cm/s^2
acc_param.q_step    = ( acc_param.max_acc - acc_param.min_acc ) / 2^16; % cm/s^2
acc_param.noise_var = 0.216531225;                                      % (cm/s^2)^2

%% LINEAR POTENTIOMETER PARAMETERS
lpot_param = struct();
lpot_param.freq         = 500;                                           % Hz
lpot_param.sample_t     = 1 / acc_param.freq;                           % s
lpot_param.max_d        = 100;                                           % cm 
lpot_param.min_d        = -100;                                          % cm
lpot_param.q_step       = (lpot_param.max_d - lpot_param.min_d) / 2^16; % cm
lpot_param.noise_var    = (0.1)^2;                                      % cm^2

%% ROAD PARAMETERS
r_param = struct();
r_param.rz_var    = (0.2)^2;   % cm^2
r_param.rzdot_var = (2)^2;     % cm^2/s^2

%% EXTENDED KALMAN FILTER PARAMETERS
ekf_param = struct();
ekf_param.freq      = 500;                          % Hz
ekf_param.sample_t  = 1 / ekf_param.freq;           % s

% filter init
ekf_param.x_init    = [10; 0; 0; 0];               
ekf_param.P_init    = eye(4);                      

% process noise matrix
ekf_param.Q         = diag([u_noise_param.u1_var, u_noise_param.u2_var, r_param.rz_var, r_param.rzdot_var]);
% measurement noise matrix
ekf_param.R         = diag([lpot_param.noise_var 10* acc_param.noise_var 1000 * acc_param.noise_var]);

%% PARTICLE FILTER (PF) PARAMETERS
pf_param = struct();
pf_param.N               = 1000;                      % Number of particles (Synchronized with pf_step.m)
pf_param.freq            = 500;                       % [Hz] Filter execution frequency
pf_param.sample_t        = 1 / pf_param.freq;         % [s] Sample time
pf_param.threshold_n_eff = 0.5;                        % Resampling threshold: N_eff / N (standard at 0.5)
pf_param.epsilon         = 0.001038;                  % Jitter noise factor (Optimized via pf_covariance_optimization.m)

% Process noise matrix
pf_param.Q               = diag([u_noise_param.u1_var, u_noise_param.u2_var, r_param.rz_var, r_param.rzdot_var]);

% Measurement noise matrix
pf_param.R               = diag([lpot_param.noise_var acc_param.noise_var acc_param.noise_var]);

% Outlier gate
pf_param.outlier_gate = [3.84; 3.84; 3.84];

% filter init
pf_param.x_init    = [10; 0; 0; 0];               
pf_param.P_init    = eye(4);

% Particle cloud initialization
init_chol = chol(pf_param.P_init);
init_noise = randn(pf_param.N, 4);
pf_param.particles_init  = repmat(pf_param.x_init', pf_param.N, 1) + init_noise*init_chol;
pf_param.weights_init    = (1 / pf_param.N) * ones(pf_param.N, 1);
clear init_chol;
clear init_noise;

% Lower triangular cholesky for noise in prediction
pf_param.L_Q             = chol(pf_param.Q, 'lower'); 