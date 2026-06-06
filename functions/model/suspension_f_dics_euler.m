function x_next = suspension_f_dics_euler( x, u, w, plant_param, sample_t )
    xdot = suspension_f_cont(x, u, w, plant_param);
    x_next = x + sample_t * xdot;
end