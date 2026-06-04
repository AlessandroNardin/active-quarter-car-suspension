function qddot = suspension_dynamics(u, w, q, qdot, plant_param)

ms    = plant_param.ms;
mu    = plant_param.mu;
ks0   = plant_param.ks0;
alpha = plant_param.alpha;
bs    = plant_param.bs;
kt    = plant_param.kt;
bt    = plant_param.bt;

u1 = u(1);
u2 = u(2);

zr    = w(1);
zrdot = w(2);

zs    = q(1);
zu    = q(2);

zsdot = qdot(1);
zudot = qdot(2);

delta_s     = zs - zu;
delta_s_dot = zsdot - zudot;
delta_t     = zu - zr;
delta_t_dot = zudot - zrdot;

Fs = ks0 * delta_s + alpha * delta_s^3 + bs * delta_s_dot;
Ft = kt * delta_t + bt * delta_t_dot;

M = [ms 0;
    0  mu];

n = [-Fs + u1;
    Fs - Ft - u1 + u2];

qddot = M \ n;