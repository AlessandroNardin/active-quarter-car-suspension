%% test_model.m - verifica del modello (sanity check)
clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));
p = params();
[sys,A,Bu,Bw,C,D] = model_quartercar(p);

fprintf('=== MODELLO QUARTER-CAR ===\n');
fprintf('Stati: %d   Ingressi: %d   Uscite: %d\n', size(A,1), size(sys,2), size(sys,1));
fprintf('Modo carrozzeria: %.2f Hz   Modo ruota: %.2f Hz\n', p.f_body, p.f_wheel);

ev = eig(A);
fprintf('\nAutovalori (parte reale max = %.4g):\n', max(real(ev)));
for k=1:numel(ev), fprintf('  %+8.3f %+8.3fi\n', real(ev(k)), imag(ev(k))); end

% Smorzamenti dei modi meccanici
fprintf('\nDamping dei modi:\n');
v = eig(A); v = v(abs(imag(v))>1);   % modi oscillatori
[~,idx] = sort(abs(imag(v)));
v = v(idx);
for k=1:min(4,numel(v))
    fprintf('  f=%6.2f Hz  zeta=%5.3f\n', abs(v(k))/(2*pi), -real(v(k))/abs(v(k)));
end

% Controllabilita' / osservabilita'
Co = ctrb(A,Bu); Ob = obsv(A,C);
fprintf('\nrank(ctrb)=%d (su %d), rank(obsv)=%d (su %d)\n', rank(Co),size(A,1),rank(Ob),size(A,1));

% Confronto linearizzato vs non lineare a piccolo segnale
x0 = [1e-4;0;0;0;0;0;0;0;0];        % piccola perturbazione
u0 = [0;0;0];
dxl = A*x0 + Bu*[0;0] + Bw*0;
dxn = quartercar_nl(x0, u0, p);
fprintf('\n|dx_lin - dx_nl| (piccolo ds) = %.3e   (deve essere ~0)\n', norm(dxl-dxn));

% Risposta in frequenza da w a uscite di prestazione (picchi ai modi)
figure('Color','w');
opts=bodeoptions; opts.Title.String='G_d: w=zrdot -> uscite (plant, disturbo bianco)';
bode(sys({'zsdd','dt','ds'},{'w'}), opts, {2*pi*0.1, 2*pi*100});
grid on;
saveas(gcf, fullfile('..','results','fig','model_gd.png'));
fprintf('\nFigura salvata.\n');
