# Definizione delle performance di controllo

*Sezione della relazione — Progetto di Controllo Robusto: sospensione attiva quarter-car con due attuatori.*

## Introduzione

Stabiliti gli obiettivi (comfort, road-holding, rattle space, sforzo attuatori) e i segnali di prestazione, occorre tradurli in **specifiche di progetto** e, successivamente, in un controllore. Questa sezione spiega come si **definiscono le performance** in controllo robusto — cioè come un requisito fisico diventa un vincolo sulla risposta in frequenza in anello chiuso — e come tale definizione si concretizzi **in modo diverso a seconda del controllore** adottato (LQG, H∞ con `mixsyn`, `hinfsyn`, `hinfstruct`, µ-synthesis).

## 1. Il quadro generale: le funzioni in anello chiuso

Le performance di un sistema in retroazione si leggono da **tre funzioni di trasferimento in anello chiuso**, con `P` il processo e `K` il controllore:

| Funzione | Definizione | Significato fisico |
|---|---|---|
| **Sensibilità** `S` | `(I + PK)^{-1}` | trasferimento **disturbo → uscita**; piccola `S` = buona **reiezione dei disturbi** |
| **Sensibilità complementare** `T` | `PK(I + PK)^{-1}` | trasferimento **rumore → uscita**; governa la **robustezza** ai modelli incerti |
| **Controllo** `KS` | `K(I + PK)^{-1}` | trasferimento **disturbo → sforzo di controllo**; governa la **sollecitazione degli attuatori** |

Vale l'identità fondamentale

$$S + T = I,$$

che è il **trade-off "a letto d'acqua"**: non è possibile rendere piccole `S` e `T` alla stessa frequenza. Se si riduce `S` a bassa frequenza (bene per i disturbi), `T` tende a `1` lì; se si vuole `T` piccola in alta frequenza (robustezza al rumore), `S` tende a `1` lì. **Definire una performance significa decidere dove `S` deve essere piccola, dove `T` deve essere piccola, e quanto picco si tollera nella transizione.**

### 1.1 Il picco di `S` e i margini di robustezza

Il picco della sensibilità

$$M_s=\|S\|_\infty=\max_\omega|S(j\omega)|$$

è la misura più sintetica della robustezza. Esso è legato ai margini classici: in via approssimata `Ms ≈ 1/\sin(PM)`. Ne segue, ad esempio, `Ms = 2 ⇒ PM ≈ 30°`, `Ms = 1.5 ⇒ PM ≈ 42°`. Un vincolo su `Ms` è quindi un vincolo **contemporaneo** su margine di fase e di guadagno.

## 2. Da requisito fisico a specifica in frequenza

Ogni requisito si traduce in un vincolo sulla **forma** di `S`, `T`, `KS` in frequenza:

| Requisito | Specifica in frequenza |
|---|---|
| **Banda passante** | frequenza di incrocio di `S` (`|S| = 1`): "fin dove il controllo corregge" |
| **Attenuazione dei disturbi** | `|S|` (o il trasferimento disturbo→uscita) sotto 0 dB in una banda, con un'attenuazione richiesta |
| **Robustezza (margini)** | picco di `S` limitato (`Ms` piccolo) |
| **Robustezza (modello incerto)** | roll-off di `T` in alta frequenza; formalmente `µ < 1` |
| **Sforzo attuatori** | `|KS|` limitata in alta frequenza |

La traduzione avviene tramite **funzioni peso** `W(s)`, richiedendo che

$$\big\| W(s)\cdot(\text{trasferimento in anello chiuso}) \big\|_\infty < 1 .$$

### 2.1 Forme standard delle funzioni peso

**Peso sulla sensibilità** (performance/disturbo):

$$W_1(s)=\frac{s/M+\omega_B}{s+\omega_B A},\qquad A\ll 1,\ M\approx 2,$$

dove `A` fissa il **guadagno a bassa frequenza** (⇒ reiezione fino a `ωB`), `ωB` è la **banda** entro cui si vuole `|S|` piccola, `M` è il **picco massimo ammesso** di `|S|`.

**Peso sulla sensibilità complementare** (robustezza/rumore):

$$W_3(s)=\frac{s+\omega_{BT}/M_3}{A_3 s+\omega_{BT}},$$

che impone a `|T|` di **calare** oltre `ωBT` (roll-off in alta frequenza).

**Peso sul controllo** (sforzo): `W2(s)`, con guadagno crescente in alta frequenza per limitare `|KS|`.

Le funzioni peso codificano quindi **interamente** la specifica di performance: banda, attenuazione, margini, sforzo.

## 3. Come cambia la definizione in base al controllore

Il quadro `S/T/KS` è comune, ma **il modo in cui la specifica viene espressa e imposta** dipende dal controllore.

### 3.1 Controllo LQG

Nell'LQG la performance non si esprime su `S`, `T`, `KS`, ma tramite un **costo quadratico**

$$J=\int_0^\infty\big(x^\top Q\,x + u^\top R\,u\big)\,dt,$$

ottimizzato separatamente (principio di separazione) da un **regolatore LQR** e da un **filtro di Kalman** (LQE).

- **`Q`** pesa gli stati: `Q` grande sugli stati da mantenere piccoli (es. `δs`, `δt`, `żu`).
- **`R`** pesa gli ingressi: `R` grande per limitare lo sforzo degli attuatori.
- Per legare `Q` ai **segnali fisici** si costruisce `Q = C_z^\top Q_y\, C_z`, dove `C_z` seleziona le uscite di prestazione.
- Il **filtro di Kalman** è definito dalle covarianze dei rumori di **processo** `Q_n` e di **misura** `R_n`: il rapporto `Q_n/R_n` fissa il compromesso tra reattività e filtraggio del rumore.
- L'**integratore** (stato aggiuntivo integrale dell'errore) garantisce **errore nullo a regime** rispetto al disturbo costante.

**Osservazione di robustezza.** L'LQR a retroazione di stato completa gode di margini garantiti (GM `= ∞`, PM `≥ 60°`), ma **l'LQG completo non li conserva in generale** (problema LTR): la presenza del filtro di Kalman può ridurre i margini. Va quindi verificato a posteriori.

### 3.2 H∞ con `mixsyn`

Il comando `mixsyn(P, W1, W2, W3)` risolve direttamente il problema **mix-sensitivity**:

$$K=\arg\min_K\ \left\| \begin{bmatrix} W_1 S \\ W_2 KS \\ W_3 T \end{bmatrix} \right\|_\infty .$$

La specifica di performance è **espressa direttamente dai pesi** `W1` (su `S`), `W2` (su `KS`), `W3` (su `T`). È il modo più immediato per "scolpire" `S`, `T`, `KS`.

**Attenzione alla struttura.** `mixsyn` nasce per un problema di **inseguimento** (`S = (I+PK)^{-1}`). Nel quarter-car il problema è di **reiezione di disturbo**: occorre ricondurvi la strada `w = żr` come disturbo/riferimento, oppure usare `hinfsyn` (sotto). Il confronto `mixsyn` vs `hinfsyn` richiesto dalla traccia mette in luce proprio questa differenza di impostazione.

### 3.3 H∞ con `hinfsyn`

Il comando `hinfsyn(P, nmeas, ncont)` minimizza `‖T_{wz}‖∞` per un **plant generalizzato** `P` costruito esplicitamente:

$$\dot x = A x + B_1 w + B_2 u,\qquad z = C_1 x + D_{11} w + D_{12} u,\qquad y = C_2 x + D_{21} w + D_{22} u,$$

con `w` gli ingressi esogeni (il disturbo stradale), `u` i comandi, `z` le uscite di prestazione pesate e `y` le misure.

Qui la performance è definita **uscite di prestazione pesate**:

$$z=\begin{bmatrix} W_c\,\ddot z_s\\ W_{rh}\,\delta_t\\ W_{rs}\,\delta_s\\ W_u\, u\end{bmatrix},$$

e l'obiettivo è `‖T_{wz}‖∞ < 1`. È la formulazione **più flessibile** e quella che meglio si adatta alla reiezione di disturbo, perché il disturbo entra esplicitamente nel plant generalizzato.

### 3.4 H∞ con `hinfstruct` (controllore strutturato, es. PID)

`hinfstruct` risolve lo **stesso tipo di obiettivo** di `hinfsyn`, ma su una **classe ristretta di controllori** a struttura fissata (es. un PID), di cui ottimizza i pochi parametri (con ottimizzazione non smooth):

$$K(s)=K_p+\frac{K_i}{s}+K_d\,s,\qquad \min_{K_p,K_i,K_d}\ \|T_{wz}(K)\|_\infty .$$

La performance si definisce **allo stesso modo** (funzioni peso sul plant generalizzato), ma con il vincolo di **struttura** del controllore. Ne segue un `‖T_{wz}‖∞` tipicamente **più alto** (meno ottimale), in cambio di un controllore **semplice, a basso ordine e facilmente implementabile**. È il compromesso "prestazione ottimale vs complessità".

### 3.5 µ-synthesis (`musyn`)

La µ-synthesis estende l'H∞ al caso di **incertezza strutturata**. Il problema diventa:

$$K=\arg\min_K\ \sup_\omega\ \mu_\Delta\big(M(K,j\omega)\big),$$

dove `M` è il trasferimento incertezza→incertezza in anello chiuso e `Δ` contiene i blocchi delle incertezze **più un blocco fittizio di performance** `Δ_P`. Si risolve con la **DK-iteration** (`musyn`), che alterna la sintesi di `K` (H∞) e l'ottimizzazione delle matrici di scala `D`.

La definizione di performance qui è **robusta**: non "il trasferimento disturbo→uscita è piccolo", ma "**per ogni** impianto dell'insieme di incertezza, il trasferimento pesato resta `< 1`". In pratica:

- si costruisce il **plant generalizzato incerto** (`uss`) con i pesi di performance (come in H∞) e la **struttura `Δ`** (i sei parametri `ms, ks0, bs, kt, τ1, τ2` + `α` nel non lineare);
- l'obiettivo è **robusta stabilità e robusta prestazione**, cioè `µ < 1`.

### 3.6 µ-analysis (verifica a posteriori)

La µ-analysis **non sintetizza**, ma **verifica** un controllore già ottenuto. Si calcola il valore singolare strutturato `µ_Δ(M(jω))` (comando `mussv`) e:

- **robusta stabilità**: `µ_Δ(M(jω)) < 1` per ogni `ω`, senza il blocco di performance;
- **robusta prestazione**: `µ < 1` includendo il blocco fittizio `Δ_P`.

Si ottengono inoltre il **margine di robusta stabilità** (`robstab`), il **guadagno peggiore** (`wcgain`) e i margini classici (`diskmargin`).

## 4. Riepilogo: definizione della performance per controllore

| Controllore | Come si definisce la performance |
|---|---|
| **LQG** | costo quadratico `J = ∫(xᵀQx + uᵀRu)`; `Q` dalle uscite di prestazione, `R` sullo sforzo; covarianze `Q_n`, `R_n` per il filtro; integratore per errore nullo |
| **H∞ `mixsyn`** | pesi `W1, W2, W3` su `S`, `KS`, `T` (forma standard tracking) |
| **H∞ `hinfsyn`** | plant generalizzato esplicito; pesi sulle uscite `z` (disturbo→uscite) |
| **H∞ `hinfstruct`** | stesso obiettivo H∞ ma su controllore a **struttura fissata** (PID) |
| **µ-synthesis `musyn`** | come H∞ + struttura di incertezza `Δ`; obiettivo **robusta prestazione** (`µ < 1`) via DK-iteration |
| **µ-analysis** | verifica: `µ_Δ(M) < 1` (robusta stabilità) e `µ < 1` con `Δ_P` (robusta prestazione) |

## 5. Sintesi

In tutti i casi vale lo stesso principio: **una performance è una forma imposta a `S` e `T` in frequenza** (dove sono piccole, quanto, con quale picco). Ciò che cambia tra i controllori è **il linguaggio con cui la specifica viene espressa**:

- l'**LQG** la esprime come **costo quadratico** (`Q`, `R`, covarianze);
- l'**H∞** (`mixsyn`/`hinfsyn`/`hinfstruct`) come **norma H∞ pesata** di funzioni in anello chiuso, con pesi `W`;
- la **µ-synthesis** come **norma µ** (robusta), aggiungendo la struttura di incertezza;
- la **µ-analysis** come **verifica** del soddisfacimento di tali vincoli su tutto l'insieme di incertezza.
