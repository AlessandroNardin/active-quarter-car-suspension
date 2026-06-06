function y = suspension_h(x, u, w, plant_param)
    % extract parameters
    ms    = plant_param.ms;
    mu    = plant_param.mu;
    ks0   = plant_param.ks0;
    alpha = plant_param.alpha;
    bs    = plant_param.bs;
    kt    = plant_param.kt;
    bt    = plant_param.bt;
    
    % extract state
    zs    = x(1);
    zu    = x(2);
    zsdot = x(3);
    zudot = x(4);
    
    % extract input
    u1 = u(1);
    u2 = u(2);
    
    % extract disturbance
    zr    = w(1);
    zrdot = w(2);
    
    % suspension deformation
    delta_s    = zs - zu;
    delta_sdot = zsdot - zudot;
    
    % tire deformation
    delta_t    = zu - zr;
    delta_tdot = zudot - zrdot;
    
    % forces
    Fs = ks0 * delta_s + alpha * delta_s^3 + bs * delta_sdot;
    Ft = kt  * delta_t + bt * delta_tdot;
    
    % output
    y = zeros(3,1);
    y(1) = zs - zu;
    y(2) = (-Fs + u1) / ms;
    y(3) = (Fs - Ft - u1 + u2) / mu;
end