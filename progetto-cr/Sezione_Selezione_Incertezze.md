# Selezione dei parametri incerti

*Sezione della relazione — Progetto di Controllo Robusto: sospensione attiva quarter-car con due attuatori.*

## Introduzione

La sintesi di un controllore robusto richiede di dichiarare esplicitamente **quali parametri del modello sono affetti da incertezza** e **con quale ampiezza**, distinguendoli da quelli assunti perfettamente noti. La traccia richiede una "scelta ragionevole sui pochi parametri che saranno considerati incerti… in relazione anche al loro ruolo nella dinamica del sistema (e.g. numero di occorrenze nelle equazioni del moto)". Questa sezione formula un criterio unico di selezione, ne deriva un insieme di parametri incerti e giustifica ampiezza e ruolo di ciascuno.

## 1. Il criterio di selezione: difficoltà ∧ rilevanza

La scelta poggia su **due assi** valutati congiuntamente.

**(A) Difficoltà conoscitiva.** Un parametro è difficile da conoscere con precisione se è affetto da *almeno uno* dei due seguenti fattori:

- **varianza fisica** in condizioni operative, cioè la grandezza cambia durante il funzionamento del sistema;
- **difficoltà di identificazione**, cioè la grandezza non è stimabile con precisione per motivi strutturali (tolleranze di fabbricazione, nonlinearità, grandezze piccole o poco osservabili). In questo caso il valore è **fisso ma ignoto**, non tempo-variante.

**(B) Rilevanza dinamica.** Un parametro è rilevante se ha **leva sul comportamento del sistema controllato**. Si distinguono due gradi:

- **rilevanza per la robusta stabilità**: il parametro sposta le **frequenze dei modi** (le risonanze) oppure consuma **margine di fase** (i ritardi). Sono gli effetti che possono compromettere la stabilità;
- **rilevanza per la prestazione**: il parametro influenza il transitorio (smorzamento, overshoot, assestamento) senza minacciare la stabilità.

Entrambi i gradi contano: un parametro che incide in modo significativo sulla prestazione — come lo smorzamento — merita di essere incluso.

**Regola di inclusione.** Un parametro è dichiarato incerto se e solo se **difficoltà alta ∧ rilevanza alta**. La congiunzione è essenziale: un parametro facile da conoscere (anche se rilevante) o poco rilevante (anche se incerto) non giustifica l'aumento di complessità. Il numero di parametri incerti determina infatti la **dimensione della matrice `Δ`**: ogni blocco reale in più accresce la dimensionalità della µ e il costo della DK-iteration, rendendo il progetto **più conservativo**. Da qui la richiesta della traccia di tenere i parametri incerti "**pochi**".

## 2. La matrice difficoltà × rilevanza

|  | **Rilevanza ALTA** | **Rilevanza BASSA** |
|---|---|---|
| **Difficoltà ALTA** | **INCERTI** → `ms`, `ks0`, `bs`, `kt`, `τ1`, `τ2` (+ `α` solo non lineare) | esclusi → `bt`, `ωn1`, `τa` |
| **Difficoltà BASSA** | *(nessuno)* | **NOTI** → `mu`, `ζ1`, `g` |

La lettura della matrice è immediata: i parametri incerti sono quelli che, **insieme**, sono difficili da conoscere **e** incidono in modo significativo sul comportamento controllato. `mu` cade nella casella opposta (facile da conoscere, poco rilevante): è l'esclusione più netta e giustificata.

## 3. Rilevanza dinamica: le occorrenze nelle equazioni del moto

Le equazioni del moto sono

$$m_s\ddot z_s=-F_s+u_1,\qquad m_u\ddot z_u=F_s-F_t-u_1+u_2,$$

con `Fs = ks0 δs + α δs³ + bs δ̇s` e `Ft = kt δt + bt δ̇t`. Contando le occorrenze:

| Parametro | Occorrenze | Equazioni coinvolte |
|---|---|---|
| `ks0`, `α`, `bs` | **2** | dentro `Fs`, presente in entrambe le equazioni |
| `ms` | 1 | solo eq. di `ms` |
| `mu`, `kt` | 1 | solo eq. di `mu` |

Le due occorrenze di `ks0`, `bs` e `α` segnalano che essi definiscono la forza **interna** tra le due masse e quindi l'**accoppiamento** dei modi: `ks0` ne fissa la **rigidezza** (le frequenze), `bs` lo **smorzamento**, `α` il comportamento **non lineare**. Le frequenze dei modi valgono `ω_s = √(ks0/ms)` e `ω_u = √((ks0+kt)/mu)`: i parametri che le determinano sono `ms`, `ks0`, `kt` (e `mu`, escluso per la matrice). È questo il fondamento della loro **rilevanza per la stabilità**.

## 4. Parametri selezionati come incerti

### 4.1 Massa sospesa `ms` — ±20%

- **Difficoltà**: alta per **varianza fisica**. `ms` comprende carrozzeria **più passeggeri e bagagli**; con `350 kg` nominali, 2–4 occupanti aggiungono ~70–80 kg per angolo (~**+20%**).
- **Rilevanza**: alta. Fissa il modo della carrozzeria, `ω_s = √(ks0/ms)`; il ±20% porta `ω_s` tra **1.10 Hz e 1.34 Hz**.

### 4.2 Rigidezza di sospensione `ks0` — ±15%

- **Difficoltà**: alta per **difficoltà di identificazione**. La variabilità in esercizio è piccola (temperatura, assestamento), ma la **tolleranza di fabbricazione delle molle** è tipicamente `±5–15%` sul rateo elastico [3], [4]: il valore è fisso ma ignoto.
- **Rilevanza**: alta (2 occorrenze). È il termine di **rigidezza** della forza interna e compare in **entrambe** le frequenze dei modi.

### 4.3 Smorzamento di sospensione `bs` — ±25%

- **Difficoltà**: alta per **varianza fisica**. Gli smorzatori idraulici dipendono **fortemente dalla temperatura** (viscosità dell'olio) e degradano con l'usura, con variazioni del coefficiente che possono superare il ±30% tra condizioni fredde e calde [1], [2].
- **Rilevanza**: alta per la **prestazione** (2 occorrenze). `bs` è il termine di **smorzamento** della forza interna; agisce sulla **parte reale** degli autovalori. I rapporti di smorzamento nominali dei due modi sono

$$\zeta_s\approx\frac{b_s}{2\sqrt{k_{s0}m_s}}\approx0.28,\qquad \zeta_u\approx\frac{b_s+b_t}{2\sqrt{(k_{s0}+k_t)m_u}}\approx0.26,$$

e con `bs` a ±25% variano in circa `0.21–0.35` e `0.20–0.32`. È una variazione che incide su overshoot e tempo di assestamento (comfort), quindi rilevante ai fini della **prestazione robusta**.

### 4.4 Rigidezza del pneumatico `kt` — ±15%

- **Difficoltà**: alta per **varianza fisica**. La rigidezza verticale del pneumatico dipende dalla **pressione di gonfiaggio** (oltre a temperatura e usura); variazioni di pressione del ~20% danno variazioni di `kt` del ~10–15% [5], [6].
- **Rilevanza**: alta. Fissa il modo della ruota, `ω_u = √((ks0+kt)/mu)`; il ±15% porta `ω_u` tra **9.4 Hz e 10.7 Hz** (modo del road-holding).

### 4.5 Ritardi degli attuatori `τ1`, `τ2` — ±25% ciascuno

- **Difficoltà**: alta per **varianza fisica**. I ritardi di trasporto dipendono da elettronica, valvole e comunicazione, tra le grandezze più variabili del sistema.
- **Rilevanza**: alta. Un ritardo introduce uno sfasamento `−ωτ` che **riduce direttamente il margine di fase** e la banda chiudibile: è un effetto di **robusta stabilità**. Sono inclusi entrambi perché la traccia richiede esplicitamente di tenere conto della **dinamica degli attuatori** e i due attuatori agiscono su anelli distinti (comfort e road-holding).

### 4.6 Coefficiente cubico di Duffing `α` — ±30% (solo non lineare)

- **Difficoltà**: alta per **difficoltà di identificazione**: è un termine non lineare identificabile solo sperimentalmente, intrinsecamente poco preciso.
- **Rilevanza**: alta sul comportamento a **grande segnale** (2 occorrenze). Tuttavia, poiché

$$\frac{\partial}{\partial \delta_s}\left(\alpha\,\delta_s^3\right)=3\alpha\,\delta_s^2\Big|_{\delta_s=0}=0,$$

`α` **scompare dal modello linearizzato**: la sua incertezza è **irrilevante per la µ-analysis lineare** e va considerata **solo** nell'analisi non lineare (RAS e simulazioni Monte-Carlo).

## 5. Parametri assunti perfettamente noti

- **`mu` (massa non sospesa)**: **difficoltà bassa** (grandezza pesabile; la varianza è di configurazione, non operativa) e **rilevanza bassa** (una sola occorrenza; effetto su `ω_u` dell'ordine di ±1–2.5%). Esclusione netta.
- **`bt` (smorzamento del pneumatico)**: difficoltà medio-alta (grandezza piccola, poco osservabile) ma rilevanza bassa (una occorrenza, contributo ~2% allo smorzamento); già indicato come noto dalla traccia.
- **`ζ1` (smorzamento attuatore primario)**: è una **scelta di progetto** (risposta piatta, `ζ1 = 1/√2`), non una grandezza fisica variabile.
- **`ωn1` (banda attuatore primario)** e **`τa` (costante di tempo attuatore secondario)**: difficoltà moderata, ma rilevanza bassa perché la dinamica degli attuatori è già coperta dai ritardi. Esclusi per il vincolo "pochi".
- **`g`**: costante fisica.

## 6. Insieme finale di incertezza e validazione

| Parametro | Nominale | Incertezza | Tipo | Criterio |
|---|---|---|---|---|
| `ms` | 350 kg | ±20% | lineare | varianza fisica + modo carrozzeria |
| `ks0` | 20000 N/m | ±15% | lineare | identificazione (tolleranza) + 2 occorrenze |
| `bs` | 1500 N·s/m | ±25% | lineare | varianza fisica + prestazione (2 occorrenze) |
| `kt` | 180000 N/m | ±15% | lineare | varianza fisica (pressione) + modo ruota |
| `τ1` | 5 ms | ±25% | lineare | varianza fisica + margine di fase |
| `τ2` | 10 ms | ±25% | lineare | varianza fisica + margine di fase |
| `α` | 1e5 N/m³ | ±30% | **non lineare** | identificazione + nonlinearità |

I primi sei costituiscono l'insieme per l'analisi di robustezza **lineare** (H∞, µ-analysis, µ-synthesis); `α` si aggiunge per la sola analisi **non lineare** (RAS, Monte-Carlo).

**Validazione a posteriori.** L'insieme usato per la **sintesi** e quello usato per la **verifica** possono non coincidere. Le simulazioni del Task 5 — campionamento dell'insieme di incertezza e confronto di stabilità e prestazione su più impianti — costituiscono la validazione sperimentale della scelta, e permettono di verificare che l'insieme adottato (e l'eventuale esclusione di parametri secondari) non comprometta la robustezza reale del controllore.

## Riferimenti

1. *The influence of temperature on the damping characteristic of hydraulic shock absorbers*, EIN. Disponibile: https://ein.org.pl/The-influence-of-temperature-on-the-damping-characteristic-of-hydraulic-shock-absorbers,158352,0,2.html
2. *An Investigation of the Influence of Temperature and Technical Parameters on the Operation of Hydraulic Shock Absorbers*, Applied Sciences, MDPI, 2022. Disponibile: https://www.mdpi.com/2076-3417/12/24/12765
3. *Spring Design Tolerances Explained*, Western Spring Manufacturing. Disponibile: https://www.westernspring.com/western-spring-resources/spring-design-tolerances-explained/
4. *Standard Spring Tolerances*, Acxess Spring. Disponibile: https://www.acxesspring.com/standard-spring-tolerances-for-custom-and-stock-springs.html
5. *Tyre features in function of the inflation pressure*, NT-Project. Disponibile: http://www.ntproject.com/tec_articolo11eng.htm
6. *Vertical stiffness of standard tires and run-flat tires at different inflation pressures*, ResearchGate. Disponibile: https://www.researchgate.net/figure/Vertical-stiffness-of-standard-tires-and-run-flat-tires-at-different-inflation-pressure_tbl2_335380981
7. S. Skogestad, I. Postlethwaite, *Multivariable Feedback Control: Analysis and Design*.
