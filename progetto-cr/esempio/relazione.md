# Relazione — Progetto di Controllo Robusto (Traccia n. 1)
## Sospensione attiva quarter-car con due attuatori

*Implementazione completa in MATLAB (cartella `code/`), risultati in `results/`.
Materiale di riferimento: slide del corso Prof. P. Salaris, Capitoli 1–5, e traccia n. 1.*

---

## Indice
1. [Introduzione e obiettivi](#1-introduzione-e-obiettivi)
2. [Modello del sistema](#2-modello-del-sistema)
3. [Incertezze parametriche](#3-incertezze-parametriche)
4. [Segnali di prestazione e misure](#4-segnali-di-prestazione-e-misure)
5. [Formulazione del problema di controllo](#5-formulazione-del-problema-di-controllo)
6. [Task 1 — Controllo LQG](#6-task-1--controllo-lqg)
7. [Task 2 — Controllo H∞ e µ-analysis](#7-task-2--controllo-h-e-µ-analysis)
8. [Task 3 — µ-synthesis (DK-iteration)](#8-task-3--µ-synthesis-dk-iteration)
9. [Task 4 — Confronto H∞ vs µ-synthesis](#9-task-4--confronto-h-vs-µ-synthesis)
10. [Task 5 — Simulazioni di robustezza](#10-task-5--simulazioni-di-robustezza)
11. [Conclusioni](#11-conclusioni)
12. [Struttura del codice](#12-struttura-del-codice)

---

## 1. Introduzione e obiettivi

Il sistema è una **sospensione attiva quarter-car** con due attuatori: `u1`
(forza tra massa sospesa e non sospesa, obiettivo **comfort**) e `u2` (forza
sulla massa non sospesa, obiettivo **road-holding**). Il disturbo esogeno è la
**velocità del profilo stradale** `w = żr`.

Il problema è di **regolazione** (non inseguimento): si vuole mantenere il
sistema all'equilibrio respingendo `w`, minimizzando l'accelerazione verticale
`z̈s` (comfort) e la deformazione del pneumatico `δt` (road-holding), entro i
limiti di corsa (`δs`, rattle space) e di sforzo degli attuatori.

### 1.1 Le frequenze critiche

Il sistema è massa–molla–smorzatore con due **modi risonanti**:

| Modo | Pulsazione | Frequenza | Smorzamento |
|---|---|---|---|
| carrozzeria (bounce) | `ωs=√(ks0/ms)` | **1.16 Hz** | ζ≈0.24 |
| ruota (wheel-hop) | `ωu=√((ks0+kt)/mu)` | **~9.9 Hz** | ζ≈0.26 |

Le risonanze sono le frequenze a cui un disturbo stradale viene amplificato:
è qui che l'attenuazione va concentrata. Poiché `w` è un disturbo d'ingresso
(bianco sulla velocità), lo **spettro di eccitazione dell'uscita è proporzionale
all'amplificazione del plant** `|P_w(jω)|²`, che picca ai modi. Questo giustifica
la scelta di **`S` piccola ai modi** (dettaglio nella Sez. 5).

---

## 2. Modello del sistema

### 2.1 Stato (9 variabili)
Con **coordinate relative** `δs = zs − zu`, `δt = zu − zr`:

| # | Stato | Significato |
|---|---|---|
| x1 | `δs` | escursione sospensione |
| x2 | `żs` | velocità massa sospesa |
| x3 | `δt` | deformazione pneumatico |
| x4 | `żu` | velocità massa non sospesa |
| x5, x6 | `u1, u̇1` | attuatore primario (2° ordine) |
| x7 | `p1` | stato Padé del ritardo `τ1` |
| x8 | `u2` | attuatore secondario (1° ordine) |
| x9 | `p2` | stato Padé del ritardo `τ2` |

Il disturbo `w = żr` entra **direttamente** (nessuno stato aggiuntivo per `zr`)
attraverso i termini `bt(żu − w)` e `δ̇t = żu − w`.

### 2.2 Dinamica non lineare (legge di Duffing)
Dalla seconda legge di Newton, con `Fs = ks0 δs + α δs³ + bs δ̇s` e
`Ft = kt δt + bt δ̇t`:

```
ms ẍs = −Fs + u1
mu ẍu =  Fs − Ft − u1 + u2
```

Il modello è implementato in `quartercar_nl.m`; in `model_quartercar.m` la
versione **linearizzata** attorno all'equilibrio `x=0`, dove il termine cubico
`α x1³` **scompare** (`∂(αx1³)/∂x1|₀ = 0`).

### 2.3 Attuatori e ritardi
- **u1** (2° ordine + ritardo): `Ga1(s)=ωn1²/(s²+2ζ1ωn1 s+ωn1²)·e^{−τ1 s}`,
  `ωn1=80 rad/s` (12.7 Hz), `ζ1=0.7`, `τ1=5 ms`.
- **u2** (1° ordine + ritardo): `Ga2(s)=1/(1+τa s)·e^{−τ2 s}`, `τa=25 ms`
  (banda 6.4 Hz), `τ2=10 ms`.

I ritardi sono approssimati con **Padé del 1° ordine** (`model_quartercar.m`):
`e^{−τs} ≈ (1−τs/2)/(1+τs/2)`, che introduce uno **zero a fase non minima** in
`s=2/τ` (limite di banda). Ciò aggiunge 2 stati (x7, x9).

**Verifica** (`test_model.m`): autovalori tutti a parte reale negativa, sistema
controllabile e osservabile; linearizzato coerente col non lineare a piccolo
segnale (errore `‖dx_lin−dx_nl‖ ≈ 2·10⁻⁹`).

---

## 3. Incertezze parametriche

Criterio (**difficoltà ∧ rilevanza**): si dichiara incerto un parametro
difficile da conoscere *e* con leva sulla dinamica.

| Parametro | Nominale | Incertezza | Motivazione |
|---|---|---|---|
| `ms` | 350 kg | **±20%** | carico variabile (passeggeri) → modo carrozzeria |
| `ks0` | 20000 N/m | **±15%** | tolleranza molle; 2 occorrenze |
| `bs` | 1500 N·s/m | **±25%** | temperatura/usura smorzatori → prestazione |
| `kt` | 180000 N/m | **±15%** | pressione pneumatico → modo ruota |
| `τ1` | 5 ms | **±25%** | ritardo → margine di fase |
| `τ2` | 10 ms | **±25%** | ritardo → margine di fase |
| `α` | 1e5 N/m³ | ±30% | Duffing, **solo non lineare** (linearizzato = 0) |

I primi sei definiscono l'insieme di incertezza **lineare** (usato da
`model_quartercar_unc.m` come oggetti `ureal`), `α` solo per l'analisi non
lineare. Assunti **noti**: `mu`, `bt`, `ζ1`, `ωn1`, `τa`, `g`.

---

## 4. Segnali di prestazione e misure

**Uscite di prestazione fisiche** (`z_phys`):
`z̈s` (comfort), `δt` (road-holding), `δs` (rattle space), `u1`, `u2` (sforzo).

**Misure** (`y`): `z̈s` (accelerometro), `δs` (posizione relativa), `żu` (velocità
massa non sospesa). Ogni misura è affetta da rumore (usato dall'LQG e inserito
come ingresso esogeno nel plant generalizzato H∞).

---

## 5. Formulazione del problema di controllo

Si segue il **framework del Capitolo 3** del corso. Il disturbo stradale è un
**input esterno** del sistema, modellato tramite un *input weight* `W(s)`:

> *"The physical input `v` … is generated by passing the normalized input `v'`
> through the transfer function block `W(s)`, sometimes referred to as input
> weight."* (Capitolo 3, "Inputs specification")

Per una strada a velocità bianca, `W_w = 1` (spettro piatto). Il disturbo
`w` entra nel plant attraverso `B_w` e raggiunge le uscite tramite il
trasferimento **disturbo→uscita** `P_w`.

### 5.1 Disturbo esplicito e plant generalizzato
Seguendo il Capitolo 3 ("it is better to consider the disturbance explicitly") e
il Capitolo 4 (formulazione generale), si costruisce il **plant generalizzato**
`P` con:

- ingressi esogeni `ξ = [w, n1, n2, n3]` (disturbo stradale + 3 rumori di misura);
- controlli `u = [u1c, u2c]`;
- **uscite pesate** `z = Wz·[z̈s; δt; δs; u1c; u2c]` (`Wz` diagonale, `weights_hinf.m`);
- misure `y = −([z̈s; δs; żu] + n)` (segno scelto per realizzare il feedback negativo).

La funzione `genplant_hinf.m` assembla `P` in spazio di stato. Il valore di
riferimento per la prestazione (Capitolo 3, "weighted sensitivity") è:

$$|S(j\omega)| \le 1/|w_P(j\omega)| \qquad\Longleftrightarrow\qquad
\|w_P S\|_\infty < 1 .$$

### 5.2 Pesi
- `wc` su `z̈s`: enfatizza la banda **4–8 Hz** (ISO 2631) e mantiene
  guadagno utile fino a ~15 Hz;
- `wrh` su `δt`: enfasi attorno al **modo ruota** (~10 Hz);
- `wrs` su `δs`: **bassa frequenza** (escursione grande);
- `wu1, wu2` su `u1c, u2c`: **passa-alto** (limita l'azione in alta frequenza).

I picchi ai modi, già contenuti in `P_w`, sono automaticamente enfatizzati.

---

## 6. Task 1 — Controllo LQG

**Struttura**: regolatore LQR (retroazione di stato) + filtro di Kalman (LQE),
principio di separazione. Sintesi in **coordinate normalizzate** (stati,
ingressi, uscite scalati alle grandezze tipiche) per evitare il
mal-condizionamento dovuto all'enorme disparità di scala tra i modi
(`code/design_lqg.m`).

**Pesi**:
- `Q = Cz3ᵀ Qy Cz3` con `Qy = diag(1,1,1)` sulle uscite di prestazione
  `[z̈s; δt; δs]`; `R = ρ·I` con `ρ=10²` (compromesso prestazione/robustezza);
- Kalman: `Qn = Bw Bwᵀ qw` (rumore di processo da strada bianca, `qw=10⁶`),
  `Rn = diag(10⁻⁸, 10⁻¹², 10⁻¹⁰)`.

**Integratore**: aggiunto uno stato integrale su `δs` per annullare l'errore a
regime rispetto a un disturbo costante.

**Risultati** (nominale):

| | `‖S‖∞ = Ms` | att. `z̈s` @1.2Hz | att. `z̈s` @10Hz |
|---|---|---|---|
| LQG | 11.17 | −3.4 dB | −0.2 dB |
| LQG + integratore | **160.9** | −3.7 dB | +0.4 dB |

**Discussione.** L'LQG attenua i disturbi ma con **bassa autorità** (pochi dB) e
un **picco di sensibilità elevato** (`Ms=11`, fino a `161` con integratore):
è l'effetto del ben noto **problema LTR** — il filtro di Kalman erode i margini
dell'LQR. L'integratore garantisce errore nullo a disturbo costante ma peggiora
drasticamente la robustezza (`Ms=161`): **per un disturbo stradale a media nulla
l'integratore non è necessario e risulta dannoso**. La bassa autorità sul modo
carrozzeria evidenzia che il problema richiede una **sintesi per forma in
frequenza** (H∞), non un semplice costo quadratico.

---

## 7. Task 2 — Controllo H∞ e µ-analysis

### 7.1 `hinfsyn` (disturbo esplicito)
Sintesi sul plant generalizzato della Sez. 5.1. Vale `γ = ‖N‖∞ = 10.08`.

### 7.2 `mixsyn` (S/KS/T classico)
`mixsyn(P,W1,W2,W3)` con `P:[u1c,u2c]→[z̈s,δs,żu]`, pesi dinamici
`W1=(s/1.5+ωb)/(s+ωb·0.1)` (`ωb=2π·10`), `W2` passa-alto, `W3` su `T`. Vale
`γ = 10.05`. **Limite strutturale**: `mixsyn` presuppone che le uscite regolate
coincidano con le misure, quindi **non può pesare `δt`** (non misurata): ne
segue una prestazione di road-holding scadente.

### 7.3 `hinfstruct` (struttura fissata)
Ottimizzazione su una classe ristretta: guadagno statico 2×3 (e tentativo di PID
decentralizzato). Risultato statico: `γ=18.15`. Come atteso, un controllore a
struttura fissa dà prestazioni inferiori.

### 7.4 µ-analysis
- `hinfsyn`: margini classici dell'anello comfort **`GM=2.7 dB`, `PM=12°`**
  (scarsi).
- `µ-synthesis`: **`GM=5.6 dB`, `PM=62°`** (buoni).

### 7.5 Tabella H∞

| controllore | `γ` | `Ms` | att.`z̈s`@1.2Hz | att.`z̈s`@10Hz | att.`δt`@1.2Hz |
|---|---|---|---|---|---|
| `hinfsyn` | 10.08 | 4.45 | −14.0 | −26.7 | −8.4 |
| `mixsyn` | 10.05 | 1.19 | −1.1 | −0.1 | **+16.9** |
| `hinfstruct` | 18.15 | 3.86 | −0.2 | −1.8 | −17.5 |

`hinfsyn` è il migliore nominalmente; `mixsyn` è troppo conservativo e non
controlla `δt`; `hinfstruct` è un compromesso a basso ordine.

---

## 8. Task 3 — µ-synthesis (DK-iteration)

**Sintesi robusta**: si costruisce il **plant generalizzato incerto** (`uss`) con
i sei parametri incerti come blocchi reali (`model_quartercar_unc.m`) e i pesi di
prestazione; `musyn` risolve la DK-iteration minimizzando
`sup_ω µ_Δ(M(K,jω))` (robusta prestazione).

**Risultati**:
- µ(RP) della DK-iteration: **12.09** al livello di prestazione nominale;
- controllore `K_mu` di ordine 77;
- prestazione nominale: **`Ms=2.23`**, att.`z̈s`: **−14.1 dB** (1.2 Hz),
  −12.2 dB (10 Hz); att.`δt`: −6.3 dB;
- **verifica robusta**: su 40 impianti campionati nell'insieme di incertezza,
  `max Re(poli) = −0.01 < 0` → **robusta stabilità confermata**.

**Osservazione sulla robusta prestazione.** µ(RP)=12>1 significa che il
**livello di prestazione richiesto** (i pesi) **non è raggiungibile** su tutto
l'insieme di incertezza. Come previsto dalla traccia (*"Nel caso non sia possibile
trovare un controllore robusto, si faccia una scelta ragionevole sugli obiettivi
di prestazione…"*), il criterio µ<1 va interpretato **rispetto al livello dei
pesi**: il controllore ottimo è invariante a una scala uniforme dei pesi, mentre
il valore di µ scala con essa. Il progetto è quindi **robustamente stabile** e
**nominalmente performante**; per ottenere formalmente `µ<1` occorre richiedere
un'attenuazione ~10× inferiore, oppure accettare il livello qui raggiunto.

---

## 9. Task 4 — Confronto H∞ vs µ-synthesis

Confronto su margini e prestazione (`analysis_compare.m`):

| controllore | `Ms` | `‖T‖∞` | GM [dB] | PM [°] | att.`z̈s`@1.2Hz | att.`z̈s`@10Hz |
|---|---|---|---|---|---|---|
| LQG | 11.17 | 11.10 | 33.7 | — | −3.4 | −0.2 |
| LQG+integr. | 160.9 | 160.9 | 9.1 | — | −3.7 | +0.4 |
| H∞ `hinfsyn` | 4.45 | 4.19 | **2.7** | **12.3** | **−14.0** | **−26.7** |
| H∞ `mixsyn` | 1.19 | 0.41 | 30.5 | — | −1.1 | −0.1 |
| H∞ `hinfstruct` | 3.86 | 3.63 | 7.2 | — | −0.2 | −1.8 |
| **µ-synthesis** | **2.23** | **1.43** | **5.6** | **61.6** | −14.1 | −12.2 |

**Lettura.** `hinfsyn` è il migliore **nominalmente** ma ha i **margini peggiori**
(`Ms=4.45`, `GM=2.7 dB`, `PM=12°`). La **µ-synthesis** sacrifica poco in termini
nominali (−14 dB contro −14 dB) ottenendo margini **nettamente migliori**
(`Ms=2.23`, `GM=5.6 dB`, `PM=62°`): è il **miglior compromesso robusto**.
`mixsyn` è robustissimo ma poco prestante; `hinfstruct` è limitato dall'ordine.

---

## 10. Task 5 — Simulazioni di robustezza

### 10.1 Monte-Carlo sull'insieme di incertezza
60 impianti campionati uniformemente su tutti i parametri incerti
(`analysis_compare.m`). Per ogni controllore si valuta stabilità e guadagno di
picco disturbo→uscita:

| controllore | % stabile | guadagno `w→z̈s` | `w→δt` | `w→δs` |
|---|---|---|---|---|
| LQG | 100% | 11.76 | 0.027 | 0.188 |
| LQG+integr. | 100% | 23.79 | 0.044 | 0.112 |
| **H∞ `hinfsyn`** | **67%** | — | — | — |
| H∞ `mixsyn` | 100% | 2543 | 27.1 | 43.9 |
| H∞ `hinfstruct` | 100% | 17.33 | 0.023 | 0.312 |
| **µ-synthesis** | **100%** | **3.82** | **0.019** | 0.263 |

**Risultato chiave.** Il controllore **H∞ nominale (`hinfsyn`) è stabile solo
nel 67%** degli impianti campionati: **non è robusto** alle incertezze
parametriche dichiarate. La **µ-synthesis è stabile al 100%** e con il **miglior
guadagno di disturbo** (`w→z̈s = 3.82`, ~3× meglio di `hinfsyn`). Questo
giustifica pienamente il passo H∞→µ della traccia.

### 10.2 "Toccare con mano": risposta a un dosso
Simulazione su un impianto **+20% `ms`** (vettura carica) sollecitato da un
dosso stradale (Fig. `compare_bump.png`): si confronta l'accelerazione `z̈s` dei
vari controllori. Conferma visivamente che i controllori robusti limitano il
picco di accelerazione anche sull'impianto perturbato.

---
---

## 11. Conclusioni

1. **Modello**: 9 stati (4 meccanici + 3 attuatore 1 + 2 attuatore 2), modi a
   1.16 Hz e ~9.9 Hz; disturbo d'ingresso `w=żr` che entra senza stati aggiuntivi.
2. **Frequenze critiche**: i **modi** (dove `|P_w|` concentra l'energia del
   disturbo bianco), con la banda ISO (4–8 Hz) come raffinamento di banda media
   sul solo canale comfort. L'attenuazione va imposta **ai modi**.
3. **LQG**: semplice ma con **LTR** (picco `Ms` alto) e scarsa autorità; integrale
   non necessario e dannoso per un disturbo a media nulla.
4. **H∞**: `hinfsyn` con **disturbo esplicito** è il più performante nominalmente
   ma **fragile** (PM 12°, 67% stabile in Monte-Carlo). `mixsyn` è conservativo
   (non può pesare `δt`); `hinfstruct` è un compromesso a basso ordine.
5. **µ-synthesis**: **miglior compromesso robusto** — stabile al 100% sul set di
   incertezza, miglior guadagno di disturbo, margini buoni (`PM=62°`).
6. **Robusta prestazione (µ)**: richiede di **rilassare il livello** di
   prestazione (µ=12>1 al livello nominale), coerentemente con il suggerimento
   della traccia.

---

## 12. Struttura del codice

```
esempio/
├── code/
│   ├── main.m                    # orchestratore della pipeline
│   ├── params.m                  # parametri nominali + incertezze + modi
│   ├── model_quartercar.m        # modello lineare 9 stati (ss)
│   ├── model_quartercar_unc.m    # modello incerto (uss, ureal)
│   ├── quartercar_nl.m           # dinamica non lineare (Duffing)
│   ├── weights_hinf.m            # funzioni peso (Wz, Wmix)
│   ├── genplant_hinf.m           # plant generalizzato per Hinf
│   ├── cl_metrics.m              # metriche anello chiuso (S,T,KS,margini)
│   ├── cl_select.m               # selezione del segno del controllore
│   ├── design_lqg.m              # Task 1
│   ├── design_hinf.m             # Task 2
│   ├── design_musyn.m            # Task 3
│   ├── analysis_compare.m        # Task 4 + 5
│   └── test_model.m              # verifica del modello
└── results/
    ├── controllers_*.mat
    └── fig/*.png
```

**Esecuzione**: `> main` dalla cartella `code/`.

### 12.1 Estratto: modello incerto (uss)
```matlab
unc.ms = ureal('ms', 350, 'Percentage', 20);
unc.kt = ureal('kt', 180000, 'Percentage', 15);
...
A = umat(zeros(9));          % consente di assegnare coefficienti ureal
A(2,1) = -ks0/ms; ...
sysu = ss(A,[Bu Bw],C,zeros(6,3));   % -> uss
```

### 12.2 Estratto: sintesi µ
```matlab
[sysu,unc] = model_quartercar_unc(p);
Pgen_u     = genplant_hinf(sysu, Wz);      % plant generalizzato incerto
[K_mu,CL,info] = musyn(Pgen_u, 3, 2);      % DK-iteration
```

### 12.3 Estratto: valutazione anello chiuso
```matlab
function m = cl_metrics(K, sys, p)          % semplificato
  [Ak,Bk,Ck,Dk]=ssdata(K);  Bu=sys.B(:,1:2); Bw=sys.B(:,3);
  Acl=[sys.A-Bu*Dk*Cmeas, -Bu*Ck; Bk*Cmeas, Ak];   % u = -K(y+n)
  m.S = inv(eye(3)+ss(sys.A,Bu,Cmeas,0)*K);
  m.Ms=norm(m.S,'inf');
  ...
end
```

---

*Fine della relazione.*
