# Definizione dei requisiti di controllo

*Sezione della relazione — Progetto di Controllo Robusto: sospensione attiva quarter-car con due attuatori.*

## Introduzione

Definito il modello (9 stati, si veda la sezione *Scelta degli stati*), occorre stabilire **che cosa** il controllore deve ottenere e **quanto bene**. Questa sezione definisce gli obiettivi di controllo, i segnali di prestazione sui quali essi si misurano e i requisiti quantitativi che li traducono in specifiche di progetto. Gli obiettivi fisici non coincidono con i singoli stati, ma con **combinazioni** di essi (le uscite di prestazione); è su queste che verranno costruite le funzioni peso della sintesi H∞/µ e le matrici `Q`, `R` dell'LQG.

## 1. Il problema di controllo

Il sistema opera attorno all'**equilibrio** e il suo compito è **reggere il disturbo esogeno** `w = żr` (la velocità del profilo stradale), mantenendo il veicolo in condizioni di comfort e di aderenza. Non si tratta di inseguire un riferimento, ma di **reiezione di un disturbo**: l'obiettivo è minimizzare l'effetto di `w` sulle grandezze di interesse, entro i limiti fisici del sistema (corsa della sospensione, sforzo e banda degli attuatori).

## 2. Segnali di prestazione (uscite da regolare)

Gli obiettivi fisici si misurano su cinque segnali, tutti esprimibili come combinazioni degli stati e degli ingressi del modello a 9 stati:

| Obiettivo | Segnale | Espressione nel modello | Perché |
|---|---|---|---|
| **Comfort** | accelerazione `z̈s` | `ẋ2 = (−Fs + u1)/ms` | è la grandezza che disturba i passeggeri (ISO 2631) |
| **Road-holding** | deformazione pneumatico `δt` | `x3` | la sua variazione è la variazione del carico sul pneumatico → aderenza |
| **Rattle space** | escursione sospensione `δs` | `x1` | vincolo di corsa: non deve andare a fondocorsa |
| **Sforzo attuatori** | forze `u1`, `u2` | `x5`, `x8` | limiti fisici (saturazione, potenza, calore) |

Si noti che **`z̈s` non è uno stato**: è una combinazione di stati e dell'ingresso `u1`. Analogamente `δs` e `δt` sono sì stati nel nostro modello, ma in generale sono combinazioni lineari delle posizioni. Questo è il motivo per cui i requisiti vanno espressi **sulle uscite**, non sui singoli elementi dello stato.

## 3. Obiettivi di controllo

1. **Regolazione all'equilibrio e reiezione del disturbo.** Mantenere il sistema attorno all'equilibrio annullando (o attenuando) l'effetto del disturbo stradale `w`. È l'obiettivo primario: tutti gli altri si misurano come *quanto* il disturbo residuo eccita ciascun segnale.
2. **Comfort.** Minimizzare l'accelerazione verticale della massa sospesa `z̈s`, con particolare attenzione alla banda attorno al **modo della carrozzeria (~1.2 Hz)** e fino a ~10–15 Hz, dove la sensibilità umana è maggiore.
3. **Road-holding.** Minimizzare la deformazione del pneumatico `δt`, con attenzione alla banda del **modo della ruota (~10 Hz)**, e **mantenere il contatto** ruota-suolo (la forza del pneumatico deve restare compressiva).
4. **Rattle space.** Contenere l'escursione della sospensione `δs` entro i limiti di corsa.
5. **Sforzo attuatori.** Contenere le forze `u1`, `u2` entro i limiti fisici e non richiedere azioni ad alta frequenza (che gli attuatori non possono erogare e che consumerebbero energia inutilmente).

## 4. Requisiti di prestazione (specifiche quantitative)

Gli obiettivi precedenti si traducono in specifiche sul comportamento **in frequenza** del sistema in anello chiuso.

**(a) Banda passante.** L'anello chiuso deve avere guadagno sufficiente (idealmente `|S| < 1`) nella banda in cui occorre respingere i disturbi, cioè circa **da 0 a 15–20 Hz**, coprendo entrambi i modi. Oltre tale banda il guadagno deve calare (roll-off) per non sollecitare gli attuatori e non amplificare il rumore di misura.

**(b) Attenuazione dei disturbi.** Nella banda di interesse, la reiezione di `w` deve essere **quantificata** come attenuazione (in dB) o come riduzione dell'RMS dei segnali di prestazione rispetto al caso passivo. Es.: ridurre l'RMS di `z̈s` di almeno `X` dB attorno ai modi, e mantenere `δt` sotto una soglia.

**(c) Margini di robustezza.** Stabilità e prestazione devono essere robuste rispetto alle incertezze dichiarate (`ms`, `ks0`, `bs`, `kt`, `τ1`, `τ2`, `α`). Si richiedono margini classici adeguati (**GM ≥ 6 dB**, **PM ≥ 30–45°**) e, per il sistema incerto, **valore singolare strutturato `µ < 1`** su tutto lo spettro (robusta stabilità e robusta prestazione).

**(d) Limiti fisici (vincoli).** `|δs|` entro la corsa disponibile, `|u1|`, `|u2|` entro i limiti degli attuatori, forza del pneumatico sempre compressiva (contatto mantenuto).

## 5. Traduzione in specifiche di progetto

I requisiti si realizzano plasmando le **funzioni di trasferimento in anello chiuso**. In un problema di reiezione di disturbo le grandezze chiave sono:

- la **sensibilità** `S` (trasferimento disturbo→uscita): piccola dove si vuole reiezione, cioè nella banda dei modi;
- la **sensibilità complementare** `T` (trasferimento rumore→uscita, e legata alla robustezza): piccola in alta frequenza, con roll-off;
- il **controllo** `KS` (sforzo): limitato in alta frequenza per non saturare gli attuatori.

Il vincolo `S + T = I` (trade-off "a letto d'acqua") rende impossibile rendere piccole entrambe ovunque: la specifica consiste nello **scolpire** `S`, `T`, `KS` con funzioni peso.

| Requisito | Traduzione |
|---|---|
| Comfort / road-holding (reiezione disturbo) | `S` piccola a bassa frequenza → peso `W1` grande lì |
| Banda passante | frequenza di incrocio di `S` (o di `T`) |
| Robustezza ai modelli incerti / rumore | picco di `S` limitato, roll-off di `T` → peso `W3` su `T` |
| Sforzo attuatori | `KS` limitato in alta frequenza → peso `W2` su `KS` |
| Rattle space / limiti | pesi o vincoli sulle uscite corrispondenti |

Nel caso **LQG**, la stessa logica si applica tramite le matrici di peso `Q` (sugli stati, costruita dalle uscite di prestazione) e `R` (sugli ingressi), con l'eventuale **integratore** per garantire errore nullo a regime rispetto al disturbo.

## 6. Misure disponibili

La traccia richiede di scegliere le misure per il controllore. Coerentemente con i segnali di prestazione e con sensori realistici su un veicolo, si adottano:

- **accelerazione della carrozzeria** `z̈s` (accelerometro sul corpo — misura diretta del comfort);
- **escursione della sospensione** `δs` (trasduttore di posizione relativa);
- **velocità della massa non sospesa** `żu` (o, in alternativa, `δt`).

L'insieme esatto delle misure e il relativo rumore di misura saranno definiti in fase di progetto del filtro di Kalman / delle uscite `y`.

## 7. Riepilogo dei requisiti

| # | Requisito | Segnale / grandezza | Specifica |
|---|---|---|---|
| 1 | Reiezione del disturbo | `w = żr` → uscite | attenuazione nella banda 0–15 Hz |
| 2 | Comfort | `z̈s` | minimizzare, banda modi + 1–15 Hz |
| 3 | Road-holding | `δt` | minimizzare, contatto mantenuto |
| 4 | Rattle space | `δs` | entro la corsa |
| 5 | Sforzo attuatori | `u1`, `u2` | entro i limiti, poco in alta frequenza |
| 6 | Banda | `S`, `T` | incrocio ~15–20 Hz, roll-off oltre |
| 7 | Robustezza | `µ`, GM, PM | `µ < 1`, GM ≥ 6 dB, PM ≥ 30–45° |
