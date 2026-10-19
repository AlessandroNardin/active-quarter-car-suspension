function [Wz, Wmix, g] = weights_hinf(p, g)
% =========================================================================
%  weights_hinf.m
%  Funzioni peso per la sintesi H-inf.
%
%  - Wz : peso 5x5 sulle USCITE FISICHE  z_phys = [zsdd; dt; ds; u1c; u2c]
%         usato per la formulazione "disturbo esplicito" (hinfsyn).
%  - Wmix : [w1; w2; w3] pesi scalari per la formulazione classica
%         mixed-sensitivity S/KS/T (mixsyn).
%  - g  : struct dei guadagni (per tuning).
%
%  Logica dei pesi:
%    wc  : comfort (zsdd). Enfatizza la banda 4-8 Hz (ISO 2631) e mantiene
%          guadagno utile fino a ~15 Hz. I picchi ai modi sono gia' nel plant.
%    wrh : road-holding (dt). Enfatizza il modo ruota (~10 Hz).
%    wrs : rattle space (ds). Enfatizza le basse frequenze (escursione grande).
%    wu  : sforzo (u1c,u2c). Passa-alto: limita l'azione in alta frequenza.
% =========================================================================
if nargin < 2 || isempty(g)
    g.wc   = 1;       % guadagno comfort
    g.gc_hf= 1;       % extra enfasi in banda comfort
    g.wrh  = 200;     % guadagno road-holding
    g.wrs  = 20;      % guadagno rattle space
    g.gu1  = 2e-4;    % guadagno sforzo u1
    g.gu2  = 2e-4;    % guadagno sforzo u2
    g.wu_hp= 2*pi*30; % frequenza di ginocchio del peso sullo sforzo [rad/s]
    g.scale= 1.0;     % livello globale delle prestazioni (vedi nota sotto)
end
% NOTA: g.scale riduce uniformemente i pesi di prestazione. Il controllore
% ottimo Hinf/mu e' INVARIANTE a una scala uniforme dei pesi; cambia solo il
% valore di gamma/mu (e quindi la "lettura" del requisito ||N||<1). Si sceglie
% g.scale in modo che il livello di prestazione richiesto sia realizzabile.

s = tf('s');

k = g.scale;   % livello globale di prestazione

% comfort: banda 4-8 Hz enfatizzata + guadagno a bassa freq
wc  = k * g.wc * (s + 2*pi*3)/(s + 2*pi*0.3) * g.gc_hf;

% road-holding: enfasi attorno al modo ruota (10 Hz)
wrh = k * g.wrh * (s + 2*pi*1)/(s + 2*pi*0.1) * (s/ (2*pi*9) + 1)/(s/(2*pi*30)+1);

% rattle space: bassa frequenza
wrs = k * g.wrs * (s + 2*pi*2)/(s + 2*pi*0.05);

% sforzo: passa-alto (penalizza solo l'alta frequenza)
wu1 = k * g.gu1 * s/(s + g.wu_hp);
wu2 = k * g.gu2 * s/(s + g.wu_hp);

% assicura dinamica modesta (numeri puliti)
Wz = append(minreal(wc), minreal(wrh), minreal(wrs), minreal(wu1), minreal(wu2));

% ---- pesi dinamici per mixsyn (S/KS/T classico) -------------------------
% W1 (su S): grande a bassa frequenza -> forza |S| piccola in banda
% W2 (su KS): crescente -> limita l'azione in alta frequenza
% W3 (su T): crescente in alta frequenza -> impone il roll-off di T
Wmix.W1 = k * (s/1.5 + 2*pi*10) / (s + 2*pi*10*0.1);  % M=1.5, wb=10 Hz, A=0.1
Wmix.W2 = k * 1e-3 * (s + 2*pi*2) / (s + 2*pi*2000);
Wmix.W3 = (s + 2*pi*50) / (0.05*s + 2*pi*50);         % su T (non scala con k)
end
