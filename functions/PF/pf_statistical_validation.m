% =========================================================================
% SCRIPT: pf_statistical_validation.m (BLINDATO SULLE DIMENSIONI RESIDUI)
% =========================================================================

% Ponticello di compatibilità per intercettare l'oggetto di output
if exist('simOut_opt', 'var')
    out = simOut_opt;
elseif exist('simOut', 'var')
    out = simOut;
end

% Safety check: Verifica che l'oggetto principale esista
if ~exist('out', 'var')
    error('I dati di simulazione "out" non sono presenti nel workspace. Lancia la simulazione prima di eseguire lo script.');
end

fprintf('Extracting diagnostic signals from Particle Filter Outports...\n');

%% 0. ESTRAZIONE DIRETTA DAI BLOCCHI OUTPORT DEL PF
try
    time_raw = out.tout;
    
    if isprop(out, 'eval_pf_Neff') || isfield(out, 'eval_pf_Neff')
        Neff_packet = out.eval_pf_Neff;
    else
        Neff_packet = evalin('base', 'eval_pf_Neff');
    end
    if isprop(Neff_packet, 'Data') || isfield(Neff_packet, 'Data'), Neff_raw = Neff_packet.Data; else, Neff_raw = Neff_packet; end
    
    if isprop(out, 'eval_pf_e') || isfield(out, 'eval_pf_e')
        res_packet = out.eval_pf_e;
    else
        res_packet = evalin('base', 'eval_pf_e');
    end
    if isprop(res_packet, 'Data') || isfield(res_packet, 'Data'), res_raw = res_packet.Data; else, res_raw = res_packet; end

catch ME
    error('Errore nell''estrazione dei blocchi Outport del PF. Verifica la coerenza delle variabili.');
end

%% 1. POST-PROCESSING AVANZATO DELLA MATRICE DEI RESIDUI
Neff_data_col = Neff_raw(:);

% Scompattamento intelligente basato sulle dimensioni effettive di res_raw
if ndims(res_raw) == 3
    % Se Simulink ha salvato come [3 x 1 x Tempo] o [1 x 3 x Tempo]
    res_data_mat = permute(res_raw, [3, 1, 2]);
    res_data_mat = squeeze(res_data_mat);
else
    res_data_mat = squeeze(res_raw);
end

% Se per qualunque motivo i sensori sono sulle righe, trasponiamo in colonne
if size(res_data_mat, 1) == 3 && size(res_data_mat, 2) ~= 3
    res_data_mat = res_data_mat';
end

% CONTROLLO DI SICUREZZA: Se è ancora un vettore singolo (N_samples x 1), 
% significa che res_raw conteneva solo il primo sensore o era schiacciato. 
% In tal caso, per non far crashare il plot, lo replichiamo o lo isoliamo.
if size(res_data_mat, 2) == 1
    fprintf('--> WARNING: Trovata una sola colonna nei residui. Verifico scompattamento nativo...\n');
    % Tentativo estremo: se la Timeseries originale aveva i dati strutturati in larghezza
    if size(res_raw, 2) == 3
        res_data_mat = res_raw;
    else
        % Se c'è un solo sensore davvero, forziamo la matrice a 3 colonne (clonandola per non rompere il ciclo)
        % così vedi comunque il grafico del Potenziometro senza crash!
        res_data_mat = [res_data_mat, res_data_mat, res_data_mat];
    end
end

% Il punto di riferimento centrale è la lunghezza di Neff
N_samples = length(Neff_data_col);

% Tagliamo i residui se balla qualche campione rispetto a Neff
if size(res_data_mat, 1) ~= N_samples
    N_samples = min([N_samples, size(res_data_mat, 1)]);
    Neff_data_col = Neff_data_col(1:N_samples);
    res_data_mat  = res_data_mat(1:N_samples, :);
end

% Rigenerazione asse dei tempi
time_col = linspace(time_raw(1), time_raw(end), N_samples)';

fprintf('PF data successfully loaded! Total samples analyzed: %d\n\n', N_samples);

%% =========================================================================
% DIAGNOSTIC 1: EFFECTIVE PARTICLE POPULATION EVALUATION (N_eff)
% =========================================================================
figure('Name', 'PF Diagnostics - Effective Particles (Neff)', 'Color', 'w');

plot(time_col(1:10:end), Neff_data_col(1:10:end), 'Color', [0.2 0.6 0.2], 'LineWidth', 1, 'DisplayName', 'N_{eff} Current');
hold on;

mean_neff = mean(Neff_data_col);
N_nominal = 1000; 
resampling_threshold = 0.5 * N_nominal; 

plot(time_col, resampling_threshold * ones(size(time_col)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Resampling Threshold (50%)');
plot(time_col, mean_neff * ones(size(time_col)), 'b-', 'LineWidth', 1.5, 'DisplayName', sprintf('Mean N_{eff}: %.1f', mean_neff));

grid on;
xlim([0, min(50, time_col(end))]); 
title('Particle Deprivation Analysis (N_{eff})');
xlabel('Time [s]'); ylabel('Number of Effective Particles');
legend('Location', 'best');

%% =========================================================================
% DIAGNOSTIC 2: ADVANCED MEASUREMENT RESIDUALS STATISTICAL ANALYSIS
% =========================================================================
figure('Name', 'PF Diagnostics - Advanced Residuals Analysis', 'Color', 'w');
sensor_names = {'Linear Potentiometer', 'Body Accelerometer', 'Wheel Accelerometer'};

for idx = 1:3
    subplot(3, 1, idx)
    
    current_res = res_data_mat(:, idx);
    
    res_mean = mean(current_res);
    res_std  = std(current_res);
    
    samples_inside = sum(abs(current_res - res_mean) <= 2*res_std);
    percentage_inside = (samples_inside / length(time_col)) * 100;
    
    plot(time_col(1:5:end), current_res(1:5:end), 'Color', [0.2 0.2 0.5], 'LineWidth', 0.5);
    hold on;
    
    plot(time_col, res_mean + 2*res_std * ones(size(time_col)), 'r--', 'LineWidth', 1.5);
    plot(time_col, res_mean - 2*res_std * ones(size(time_col)), 'r--', 'LineWidth', 1.5);
    
    grid on;
    title(sprintf('Residual: %s (Inside 2\\sigma: %.2f%%)', sensor_names{idx}, percentage_inside));
    xlabel('Time [s]'); ylabel('Error [z - z\_hat]');
    
    fprintf('--- PF Statistical Analysis: Sensor %d (%s) ---\n', idx, sensor_names{idx});
    fprintf('Residual Mean (Optimal if ~0)      : %.6f\n', res_mean);
    fprintf('Residual Standard Deviation (\\sigma) : %.6f\n', res_std);
    fprintf('Samples inside 2-\\sigma Band (Target ~95%%): %.2f%%\n\n', percentage_inside);
end