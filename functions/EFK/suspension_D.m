% Jacobian of discretized (euler) f with respect to w
function Dk = suspension_D(plant_param, filter_param)

% extract params
mu = plant_param.mu;
kt = plant_param.kt;
bt = plant_param.bt;

% continuous jacobian with respect to w
Dc = zeros(4,2);

% first three rows are 0

%fourth row
Dc(4,1) = kt / mu;
Dc(4,2) = bt / mu;

% discrete jacobian
Dk = filter_param.sample_t * Dc;