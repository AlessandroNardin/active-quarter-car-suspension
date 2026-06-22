function [xcurr, Pcurr, L, e] = ekf_step(xprev, Pprev, uprev, ucurr, z, plant_param, ekf_param, valid_meas)
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
    
    % DETECTION DI OUTLIER (MAHALANOBIS) E MULTIRATE CONTROL
    % Soglia per la distanza di Mahalanobis (impostata a 6 per i 6-sigma)
    soglia = 6.0; 
    
    for i = 1:size(z, 1)
        % 1. OUTLIER DETECTION: valuto solo se il sensore ha inviato un dato (valid_meas == 1)
        if valid_meas(i) == 1
            % Calcolo l'incertezza S per la singola i-esima misura
            S_ii = H(i, :) * Ppred * H(i, :)' + R_eff(i, i);
            
            % Distanza di Mahalanobis: D_k = sqrt(e_k^T * S_k^-1 * e_k)
            D_k = sqrt((e(i)^2) / S_ii);
            
            % Se la distanza supera la soglia, è un outlier
            if D_k > soglia
                valid_meas(i) = 0; % Misura spuria: forzo valid_meas a 0 per scartarla
            end
        end
        % 2. MULTIRATE CONTROL: annulla H se il sensore è lento o scartato per outlier
        if valid_meas(i) == 0
            H(i, :) = 0; % Annulla la riga dello Jacobiano per il sensore non aggiornato
        end
    end
    
    S = H * Ppred * H' + R_eff;
    L = Ppred * H' / S;
    
    % Update the state estimate and covariance
    xcurr = xpred + L * e;
    I = eye(size(Ppred));
    Pcurr = (I - L * H) * Ppred * (I - L * H)' + L * R_eff * L';
end