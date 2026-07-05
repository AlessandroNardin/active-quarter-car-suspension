% Jacobian of discretized (euler) f with respect to w
function Dk = suspension_D(plant_param, ekf_param)

% extract params
ms = plant_param.ms;
mu = plant_param.mu;
kt = plant_param.kt;
bt = plant_param.bt;

% continuous jacobian with respect to w
Dc = zeros(4,4);

% first two rows are 0

% third row
Dc(3,1) = 1 / ms;

%fourth row
Dc(4,1) = -1 / mu;
Dc(4,1) = 1 / mu;
Dc(4,1) = kt / mu;
Dc(4,2) = bt / mu;

% discrete jacobian
Dk = ekf_param.sample_t * Dc;
