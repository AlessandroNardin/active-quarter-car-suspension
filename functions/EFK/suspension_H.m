% Jacobian of h with respect to x
function Hk = suspension_H(x, plant_param)

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
Hk = zeros(3,4);

% first row
Hk(1,1) = 1;
Hk(1,2) = -1;

% second row
Hk(2,1) = -(1/ms) * (ks0 + 3*alpha*delta_s^2);
Hk(2,2) =  (1/ms) * (ks0 + 3*alpha*delta_s^2);
Hk(2,3) = -(1/ms) * bs;
Hk(2,4) =  (1/ms) * bs;

% third row
Hk(3,1) =  (1/mu) * (ks0 + 3*alpha*delta_s^2);
Hk(3,2) = -(1/mu) * (ks0 + 3*alpha*delta_s^2 + kt);
Hk(3,3) =  (1/mu) * bs;
Hk(3,4) = -(1/mu) * (bs + bt);
end