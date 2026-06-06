function xdot = suspension_f_cont(x, u, w, plant_param)
    % extract param
    ms    = plant_param.ms;
    mu    = plant_param.mu;
    ks0   = plant_param.ks0;
    alpha = plant_param.alpha;
    bs    = plant_param.bs;
    kt    = plant_param.kt;
    bt    = plant_param.bt;
    
    % State extraction
    zs    = x(1);
    zu    = x(2);
    zsdot = x(3);
    zudot = x(4);
    
    % Disturbance extraction
    zr    = w(1);
    zrdot = w(2);
    
    % Input extraction
    u1 = u(1);
    u2 = u(2);
    
    % Relative deformations
    delta_s     = zs - zu;
    delta_s_dot = zsdot - zudot;
    delta_t     = zu - zr;
    delta_t_dot = zudot - zrdot;
    
    % Forces
    Fs = ks0 * delta_s + alpha * delta_s^3 + bs * delta_s_dot;
    Ft = kt  * delta_t + bt  * delta_t_dot;
    
    % Accelerations from Newton's second law - eqs (3) and (4)
    zsddot = (-Fs + u1)          / ms;
    zuddot = ( Fs - Ft - u1 + u2) / mu;
    
    % State derivative
    xdot = [zsdot; zudot; zsddot; zuddot];
end