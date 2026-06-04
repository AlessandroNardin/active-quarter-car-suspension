function xdot = suspension_f_cont(x, u, w, plant_param)
% state is x = [zs; zu; zsdot; zudot]

% ---------- basic input shape checks ----------
if numel(x) ~= 4
    error('suspension_f_cont:InvalidState', ...
        'x must have 4 elements: [zs zu zsdot zudot]. Got %d.', numel(x));
end

if numel(u) < 2
    error('suspension_f_cont:InvalidInput', ...
        'u must have at least 2 elements: [u1 u2]. Got %d.', numel(u));
end

if numel(w) < 2
    error('suspension_f_cont:InvalidDisturbance', ...
        'w must have at least 2 elements: [zr zrdot]. Got %d.', numel(w));
end

required_fields = {'ms','mu','ks0','alpha','bs','kt','bt'};
for k = 1:numel(required_fields)
    if ~isfield(plant_param, required_fields{k})
        error('suspension_f_cont:MissingParam', ...
            'Missing plant_param field: %s', required_fields{k});
    end
end

% ---------- extract parameters ----------
ms    = plant_param.ms;
mu    = plant_param.mu;
ks0   = plant_param.ks0;
alpha = plant_param.alpha;
bs    = plant_param.bs;
kt    = plant_param.kt;
bt    = plant_param.bt;

% ---------- parameter validation ----------
param_vec = [ms mu ks0 alpha bs kt bt];
param_names = {'ms','mu','ks0','alpha','bs','kt','bt'};

if any(~isfinite(param_vec))
    fprintf('\n[ERROR] Non-finite parameter detected.\n');
    for i = 1:numel(param_vec)
        fprintf('  %s = %g\n', param_names{i}, param_vec(i));
    end
    error('suspension_f_cont:NonFiniteParam', ...
        'plant_param contains NaN or Inf.');
end

if ms == 0 || mu == 0
    fprintf('\n[ERROR] Division by zero risk.\n');
    fprintf('  ms = %g\n', ms);
    fprintf('  mu = %g\n', mu);
    error('suspension_f_cont:ZeroMass', ...
        'ms and mu must be nonzero.');
end

if ms < 0 || mu < 0
    fprintf('\n[WARNING] Negative mass detected.\n');
    fprintf('  ms = %g\n', ms);
    fprintf('  mu = %g\n', mu);
end

% ---------- force column vectors ----------
x = x(:);
u = u(:);
w = w(:);

% ---------- input validation ----------
if any(~isfinite(x))
    fprintf('\n[ERROR] Non-finite state x detected.\n');
    fprintf('  x = [%g %g %g %g]\n', x(1), x(2), x(3), x(4));
    error('suspension_f_cont:NonFiniteState', ...
        'State x contains NaN or Inf.');
end

if any(~isfinite(u(1:2)))
    fprintf('\n[ERROR] Non-finite control input u detected.\n');
    fprintf('  u1 = %g, u2 = %g\n', u(1), u(2));
    error('suspension_f_cont:NonFiniteInput', ...
        'Input u contains NaN or Inf.');
end

if any(~isfinite(w(1:2)))
    fprintf('\n[ERROR] Non-finite disturbance w detected.\n');
    fprintf('  zr = %g, zrdot = %g\n', w(1), w(2));
    error('suspension_f_cont:NonFiniteDisturbance', ...
        'Disturbance w contains NaN or Inf.');
end

% ---------- state extraction ----------
zs    = x(1);
zu    = x(2);
zsdot = x(3);
zudot = x(4);

% ---------- disturbance extraction ----------
zr    = w(1);
zrdot = w(2);

% ---------- input extraction ----------
u1 = u(1);
u2 = u(2);

% ---------- relative deformations ----------
delta_s     = zs - zu;
delta_s_dot = zsdot - zudot;
delta_t     = zu - zr;
delta_t_dot = zudot - zrdot;

% ---------- check intermediate states ----------
intermediate_vec = [delta_s delta_s_dot delta_t delta_t_dot];
intermediate_names = {'delta_s','delta_s_dot','delta_t','delta_t_dot'};

if any(~isfinite(intermediate_vec))
    fprintf('\n[ERROR] Non-finite intermediate deformation term detected.\n');
    for i = 1:numel(intermediate_vec)
        fprintf('  %s = %g\n', intermediate_names{i}, intermediate_vec(i));
    end
    fprintf('  zs=%g, zu=%g, zsdot=%g, zudot=%g, zr=%g, zrdot=%g\n', ...
        zs, zu, zsdot, zudot, zr, zrdot);
    error('suspension_f_cont:NonFiniteIntermediate', ...
        'Intermediate deformation term became NaN or Inf.');
end

% ---------- forces ----------
Fs = ks0 * delta_s + alpha * delta_s^3 + bs * delta_s_dot;
Ft = kt  * delta_t + bt  * delta_t_dot;

if any(~isfinite([Fs Ft]))
    fprintf('\n[ERROR] Non-finite force detected.\n');
    fprintf('  Fs = %g\n', Fs);
    fprintf('  Ft = %g\n', Ft);
    fprintf('  delta_s = %g\n', delta_s);
    fprintf('  delta_s_dot = %g\n', delta_s_dot);
    fprintf('  delta_t = %g\n', delta_t);
    fprintf('  delta_t_dot = %g\n', delta_t_dot);
    fprintf('  alpha * delta_s^3 term = %g\n', alpha * delta_s^3);
    error('suspension_f_cont:NonFiniteForce', ...
        'Fs or Ft became NaN or Inf.');
end

% ---------- optional magnitude warnings ----------
if abs(delta_s) > 1e3 || abs(delta_t) > 1e3
    fprintf('\n[WARNING] Very large displacement detected.\n');
    fprintf('  delta_s = %g\n', delta_s);
    fprintf('  delta_t = %g\n', delta_t);
end

if abs(Fs) > 1e12 || abs(Ft) > 1e12
    fprintf('\n[WARNING] Very large force detected.\n');
    fprintf('  Fs = %g\n', Fs);
    fprintf('  Ft = %g\n', Ft);
end

% ---------- accelerations ----------
zsddot = (-Fs + u1) / ms;
zuddot = ( Fs - Ft - u1 + u2) / mu;

if any(~isfinite([zsddot zuddot]))
    fprintf('\n[ERROR] Non-finite acceleration detected.\n');
    fprintf('  zsddot = %g\n', zsddot);
    fprintf('  zuddot = %g\n', zuddot);
    fprintf('  Fs = %g, Ft = %g, u1 = %g, u2 = %g\n', Fs, Ft, u1, u2);
    fprintf('  ms = %g, mu = %g\n', ms, mu);
    error('suspension_f_cont:NonFiniteAcceleration', ...
        'Acceleration became NaN or Inf.');
end

% ---------- state derivative ----------
xdot = [zsdot;
        zudot;
        zsddot;
        zuddot];

if any(~isfinite(xdot))
    fprintf('\n[ERROR] Non-finite state derivative detected.\n');
    fprintf('  xdot = [%g %g %g %g]\n', xdot(1), xdot(2), xdot(3), xdot(4));
    error('suspension_f_cont:NonFiniteXdot', ...
        'xdot contains NaN or Inf.');
end
end