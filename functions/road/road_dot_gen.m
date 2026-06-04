function zr = road_dot_gen(t, road_params)
    % If we are beyond the road length, return zero
    if road_params.vehicle_speed * t > road_params.L
        zr = 0;
        return;
    end

    % Derivative of the phase with respect to time
    rate = 2 * pi * road_params.vehicle_speed * road_params.freq;

    % Phase of each harmonic
    phase_arg = rate * t + road_params.phi;

    % Time derivative of the multisine road profile
    zr = sum(-road_params.A .* rate .* sin(phase_arg));
end