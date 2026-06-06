% Jacobian of discretized (euler) f with respect to x
function Fk = suspension_F(x, plant_param, filter_param)

%extract params
ms    = plant_param.ms;
mu    = plant_param.mu;
ks0   = plant_param.ks0;
alpha = plant_param.alpha;
bs    = plant_param.bs;
kt    = plant_param.kt;
bt    = plant_param.bt;

%extract state
zs = x(1);
zu = x(2);

delta_s = zs - zu;

% continue jacobian
Ac = zeros(4,4);

% first row
Ac(1,3) = 1;

% second row
Ac(2,4) = 1;

% third row
Ac(3,1) = -(1/ms) * (ks0 + 3*alpha*delta_s^2);
Ac(3,2) =  (1/ms) * (ks0 + 3*alpha*delta_s^2);
Ac(3,3) = -(1/ms) * bs;
Ac(3,4) =  (1/ms) * bs;

% fourth row
Ac(4,1) =  (1/mu) * (ks0 + 3*alpha*delta_s^2);
Ac(4,2) = -(1/mu) * (ks0 + 3*alpha*delta_s^2 + kt);
Ac(4,3) =  (1/mu) * bs;
Ac(4,4) = -(1/mu) * (bs + bt);

% discrete jacobian
Fk = eye(4) + filter_param.sample_t * Ac;