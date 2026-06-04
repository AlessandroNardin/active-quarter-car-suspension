% state is x = [ x1 x2 x3 x4 ] = [ zs zu zsdot zudot ]
function x_next = suspension_f_dics_euler( x, u, w, plant_param, filter_param)
xdot = suspension_f_cont(x, u, w, plant_param);
x_next = x + filter_param.sample_t * xdot;