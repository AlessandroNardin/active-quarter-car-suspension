# Progetto Controllo Robusto — Traccia n.1 (esempio)

Sospensione attiva quarter-car con due attuatori. Implementazione MATLAB
completa (LQG, H∞, µ-synthesis) + simulazioni di robustezza.

## Contenuto
- `relazione.md` — relazione completa (metodo, risultati, figure, codice).
- `code/` — sorgenti MATLAB.
- `results/` — controllori salvati (`controllers_*.mat`) e figure (`fig/`).
- `report/` — (riservata a eventuali PDF/LaTeX).

## Requisiti
MATLAB con Control System Toolbox, Robust Control Toolbox e Simulink (testato
su R2026a).

## Esecuzione
```matlab
cd code
main          % esegue l'intera pipeline (Task 0..5)
```
Oppure i singoli task:
```matlab
test_model        % verifica del modello
design_lqg        % Task 1 - LQG (con/senza integratore)
design_hinf       % Task 2 - Hinf (mixsyn, hinfsyn, hinfstruct)
design_musyn      % Task 3 - mu-synthesis + mu-analysis
analysis_compare  % Task 4 - confronto ; Task 5 - robustezza
```

## Mappa dei file
| File | Ruolo |
|---|---|
| `params.m` | parametri nominali, incertezze, modi |
| `model_quartercar.m` | modello lineare 9 stati (ss) |
| `model_quartercar_unc.m` | modello incerto (uss/ureal) |
| `quartercar_nl.m` | dinamica non lineare (Duffing) |
| `weights_hinf.m` | funzioni peso di prestazione |
| `genplant_hinf.m` | plant generalizzato (disturbo esplicito) |
| `cl_metrics.m`, `cl_select.m` | valutazione anello chiuso, convenzione di segno |
| `design_*.m` | sintesi dei controllori |
| `analysis_compare.m` | confronto e Monte-Carlo |
