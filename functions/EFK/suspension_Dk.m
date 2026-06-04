% computation of discrete jacobian Dw
% state is x = [ x1 x2 x3 x4 ] = [ zs zu zsdot zudot ]
% disturbance is w = [ w1 w2 ] = [ zr zrdot ]
function Dk = suspension_Dk(plant_param, filter_param)
%#codegen

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