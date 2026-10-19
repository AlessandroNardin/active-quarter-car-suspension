function p = params()
% =========================================================================
%  params.m
%  Parametri del modello quarter-car attivo (Traccia n.1 + slide del corso).
%
%  Ritorna una struct `p` con:
%    - parametri nominali (Tabella 1 della traccia),
%    - semi-ampiezze relative delle incertezze (scelte, cfr. relazione),
%    - frequenze dei modi e grandezze derivate di comodo.
%
%  Riferimenti: "Traccia n.1 - Controllo Robusto", Tabella 1.
% =========================================================================

% ---- Massa sospesa (carrozzeria + passeggeri + bagagli) ------------------
p.ms = 350;        % [kg]

% ---- Massa non sospesa (ruota + cerchio + assale) ------------------------
p.mu = 50;         % [kg]

% ---- Sospensione (legge di Duffing: Fs = ks0*ds + alpha*ds^3 + bs*dsdot) -
p.ks0   = 20000;   % rigidezza lineare       [N/m]
p.alpha = 1e5;     % coefficiente cubico      [N/m^3]
p.bs    = 1500;    % smorzamento viscoso      [N*s/m]

% ---- Pneumatico (lineare: Ft = kt*dt + bt*dtdot) -------------------------
p.kt = 180000;     % rigidezza verticale      [N/m]
p.bt = 150;        % smorzamento pneumatico   [N*s/m]

% ---- Attuatore primario u1 (2o ordine + ritardo) -------------------------
p.wn1   = 80;      % pulsazione naturale      [rad/s]  (~12.7 Hz)
p.zeta1 = 0.7;     % smorzamento (risposta piatta 1/sqrt(2))
p.tau1  = 5e-3;    % ritardo di trasporto     [s]

% ---- Attuatore secondario u2 (1o ordine + ritardo) -----------------------
p.taua = 25e-3;    % costante di tempo        [s]  (banda 6.4 Hz)
p.tau2 = 10e-3;    % ritardo di trasporto     [s]

% ---- Gravita' ------------------------------------------------------------
p.g = 9.81;        % [m/s^2]

% ---- Incertezze parametriche (semi-ampiezza relativa, cfr. selezione) ----
p.unc.ms    = 0.20;   % varianza fisica (occupanti)   -> modo carrozzeria
p.unc.ks0   = 0.15;   % tolleranza molle              -> modo carrozzeria
p.unc.bs    = 0.25;   % temperatura/usura smorzatori  -> prestazione
p.unc.kt    = 0.15;   % pressione pneumatico          -> modo ruota
p.unc.tau1  = 0.25;   % ritardo attuatore primario    -> margine di fase
p.unc.tau2  = 0.25;   % ritardo attuatore secondario  -> margine di fase
p.unc.alpha = 0.30;   % Duffing (solo non lineare)

% ---- Frequenze dei modi (riferimento per le specifiche) ------------------
p.f_body  = sqrt(p.ks0/p.ms)          / (2*pi);   % ~1.20 Hz  (comfort)
p.f_wheel = sqrt((p.ks0+p.kt)/p.mu)   / (2*pi);   % ~10.0 Hz  (road-holding)

% ---- Rapporti di smorzamento nominali dei modi ---------------------------
p.zeta_s = p.bs / (2*sqrt(p.ks0*p.ms));
p.zeta_u = (p.bs+p.bt) / (2*sqrt((p.ks0+p.kt)*p.mu));

end
