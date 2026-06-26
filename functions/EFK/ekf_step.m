function [xcurr, Pcurr, xpred, Ppred, L, e] = ekf_step(xprev, Pprev, uprev, ucurr, z, plant_param, ekf_param, valid_meas)
    %% Prediction
    % compute new x
    xpred = suspension_f_dics_euler(xprev, uprev, [0; 0], plant_param, ekf_param.sample_t);
    F = suspension_F(xprev, plant_param, ekf_param);
    D = suspension_D(plant_param, ekf_param);
    
    % compute new P matrix
    Ppred = F * Pprev * F' + D * ekf_param.Q * D';
    
    %% Correction
    H = suspension_H(xpred, plant_param);
    M = suspension_M();
    R_eff = M * ekf_param.R * M'; % Matrice di covarianza del rumore effettiva
    
    % innovation
    e = z - suspension_h(xpred, ucurr, [0; 0], plant_param);
    
    % Outlier detection and Multirate handling
    
    % Mahalanobis threshold (6 for 6-sigma)
    mhlb_th = 6.0; 
    
    for i = 1:size(z, 1)
        % Control on new measurements to detect outliers
        if valid_meas(i) == 1

            % uncertainty
            S_ii = H(i, :) * Ppred * H(i, :)' + R_eff(i, i);
            
            % Mahalanobis distance
            D_k = sqrt((e(i)^2) / S_ii);
            
            
            if D_k > mhlb_th
                valid_meas(i) = 0; % If outlier set to invalid
            end
        end
        
        % Remove effect of discarded measurements (outliers or old)
      
        if valid_meas(i) == 0
            % Set rows to zero to delete measurement effect
            H(i, :) = 0;
            e(i) = 0;
            % Set corresponding value in R high value to delete effect
            R_eff(i,i) = 1e12;
        end
    end
    
    S = H * Ppred * H' + R_eff;
    L = Ppred * H' / S;
    
    % Update the state estimate and covariance
    xcurr = xpred + L * e;
    I = eye(size(Ppred));
    Pcurr = (I - L * H) * Ppred * (I - L * H)' + L * R_eff * L';
end