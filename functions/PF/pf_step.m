function [xcurr, Pcurr, particles_curr, weights_curr, Neff, residui_out] = pf_step(particles_prev, weights_prev, uprev, ucurr, z, plant_param, pf_param, valid_meas)

    N = size(particles_prev, 1);
    nx = size(particles_prev, 2);

    particles_curr = zeros(N, nx);
    weights_curr   = zeros(N, 1);
    xcurr          = zeros(nx, 1);
    Pcurr          = zeros(nx, nx);

    R   = pf_param.R;
    L_Q = pf_param.L_Q;

    %% Prediction
    for i = 1:N
        noise = L_Q * randn(4, 1);
        input_noise = noise(1:2);
        road_noise = noise(3:4);
        particles_curr(i, :) = suspension_f_dics_euler( ...
            particles_prev(i, :)', uprev + input_noise, road_noise, plant_param, pf_param.sample_t)';
    end

    %% Correction
    likelihood = ones(N, 1);

    for i = 1:N
        z_hat = suspension_h(particles_curr(i, :)', ucurr, [0; 0], plant_param);
        temp_l = 1.0;

        for j = 1:3
            if valid_meas(j) == 1
                err = z(j) - z_hat(j);
                nu  = (err^2) / (R(j, j) + eps);
                
                if nu >= pf_param.outlier_gate(j)
                    continue
                end
                
                temp_l = temp_l * exp(-0.5 * nu);
            end
        end

        likelihood(i) = temp_l;
    end

    %% Weight update
    weights_curr = weights_prev .* likelihood;
    sum_w = sum(weights_curr) + eps;
    weights_curr = weights_curr / sum_w;

    Neff = 1 / (sum(weights_curr.^2) + eps);

    %% State estimate
    xcurr = particles_curr' * weights_curr;

    %% State covariance
    for i = 1:N
        dx = particles_curr(i, :)' - xcurr;
        Pcurr = Pcurr + weights_curr(i) * (dx * dx');
    end

    Pcurr   = 0.5 * (Pcurr + Pcurr');

    %% Residuals from estimated state
    z_est       = suspension_h(xcurr, ucurr, [0; 0], plant_param);
    residui_out = z - z_est;

    %% Resampling
    if Neff < (N * pf_param.threshold_n_eff)
        s = cumsum(weights_curr);
        new_p = zeros(N, nx);

        for j = 1:N
            u = rand(1);
            idx = 1;

            while (idx < N) && (s(idx) < u)
                idx = idx + 1;
            end

            new_p(j, :) = particles_curr(idx, :) + pf_param.epsilon * randn(1, nx);
        end

        particles_curr = new_p;
        weights_curr   = ones(N, 1) / N;
    end

end