function [ xcurr, Pcurr, L ] = ekf_step ( xprev, Pprev, uprev, ucurr, z, plant_param, filter_param)
%% Prediction
% compute new x
xpred = suspension_f_dics_euler( xprev, uprev, [ 0; 0 ], plant_param, filter_param.sample_t);

F = suspension_F(xprev, plant_param, filter_param);
D = suspension_D(plant_param, filter_param);

% compute new P matrix
Ppred = F * Pprev * F' + D * filter_param.Q * D';

%% Correction
H = suspension_H(xpred, plant_param);
M = suspension_M();

% innovation
e = z - suspension_h(xpred, ucurr, [0; 0], plant_param);

% HERE I WILL ADD CONTROLS ON INNOVATION TO FILTER OUT OUTLIERS
% MIGLT ALSTO ADD A "VALID" VECOTR OF 1 AND 0 SO I CAN DO e = z - sus.... *
% VALID SO ONLY VALID MEASUREMTS ARE TAKEN INTO ACCOUT


S = H * Ppred * H' + M * filter_param.R * M';
L = Ppred * H' / S;

% Update the state estimate and covariance
xcurr = xpred + L * e;

I = eye(size(Ppred));
Pcurr = (I - L * H) * Ppred * (I - L * H)' + L * (M * filter_param.R * M') * L';





