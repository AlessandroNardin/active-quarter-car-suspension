function zr = road_gen(t, road_params)
% If we are beyond the road length, return zero
if road_params.vehicle_speed * t > road_params.L
    zr = 0;
    return;
end

% Phase argument for the cosine terms (scalar t → scalar phase_arg per harmonic)
phase_arg = 2 * pi * road_params.vehicle_speed * t * road_params.freq + road_params.phi;

% Multisine: sum of A_i * cos(phase_arg_i)
zr = sum(road_params.A .* cos(phase_arg));
end