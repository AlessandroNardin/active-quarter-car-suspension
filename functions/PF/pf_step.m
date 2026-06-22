function [xcurr, particles_curr, weights_curr, Neff, residui_out] = pf_step(particles_prev, weights_prev, uprev, ucurr, z, plant_param, pf_param, valid_meas)
    % Dynamically retrieve the number of particles from the input matrix size
    N = size(particles_prev, 1); 
    
    % Static allocation based on signal dimensions
    particles_curr = zeros(N, 4);
    weights_curr   = zeros(N, 1);
    xcurr          = zeros(4, 1);
    
    Q   = pf_param.Q; 
    R   = pf_param.R;
    L_Q = pf_param.L_Q; 
    
    %% Prediction
    for i = 1:N
        % Inject process noise using the lower triangular Cholesky factor
        noise = L_Q * randn(2, 1);
        particles_curr(i, :) = suspension_f_dics_euler(particles_prev(i, :)', uprev, noise, plant_param, pf_param.sample_t)';
    end
    
    %% Correction (Likelihood)
    likelyhood = ones(N, 1);
    for i = 1:N
        z_hat = suspension_h(particles_curr(i, :)', ucurr, [0; 0], plant_param);
        temp_l = 1.0;
        
        % Evaluate sensor measurements considering multirate control
        for j = 1:3
            if valid_meas(j) == 1
                err = (z(j) - z_hat(j)) / sqrt(R(j, j));
                temp_l = temp_l * exp(-0.5 * err^2);
            end
        end
        likelyhood(i) = temp_l;
    end
    
    % Update and normalize weights
    sum_w        = sum(weights_prev .* likelyhood) + eps;
    weights_curr = (weights_prev .* likelyhood) / sum_w;
    
    % DIAGNOSTICA: Compute the number of effective particles (N_eff) before resampling
    Neff = 1 / (sum(weights_curr.^2) + eps);
    
    %% Resampling
    if Neff < (N * pf_param.threshold_n_eff)
        s     = cumsum(weights_curr);
        new_p = zeros(N, 4);
        
        for j = 1:N
            u   = rand(1);
            idx = 1;
            while (idx < N) && (s(idx) < u)
                idx = idx + 1; 
            end
            % Apply jittering (epsilon) to avoid particle deprivation/sample impoverishment
            new_p(j, :) = particles_curr(idx, :) + pf_param.epsilon * randn(1, 4);
        end
        particles_curr = new_p;
        weights_curr   = (1 / N) * ones(N, 1);
    end
    
    %% MMSE State Estimation
    xcurr = particles_curr' * weights_curr;
    
    % DIAGNOSTICA: Compute measurement residuals based on the MMSE state estimate
    z_est       = suspension_h(xcurr, ucurr, [0; 0], plant_param);
    residui_out = z - z_est;
end