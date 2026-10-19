# Modello del disturbo e scelta della banda di controllo

*Sezione della relazione — Progetto di Controllo Robusto: sospensione attiva quarter-car con due attuatori.*

## Introduzione

Il problema è di **regolazione**: l'unico ingresso esogeno è il disturbo stradale `w`, e l'obiettivo è mantenere le grandezze di prestazione (`z̈s`, `δt`, `δs`) vicine a zero nonostante `w` (riferimento `r = 0`). Definire le performance richiede quindi di sapere **dove**, in frequenza, il disturbo eccita il sistema. Questa sezione specifica il modello del disturbo adottato, le semplificazioni introdotte e il criterio con cui si sceglie la **banda di controllo**.

## 1. Il disturbo fisico

Il profilo stradale `zr(t)` è la quota della strada; la sua derivata `w = żr` è la **velocità** del profilo, ed è il disturbo esogeno del modello. Nel sistema il disturbo entra attraverso i due termini della forza del pneumatico, `kt(zu − zr)` e `bt(żu − żr)`; grazie all'uso delle coordinate relative, `w` entra direttamente senza richiedere uno stato aggiuntivo.

Un profilo stradale reale non è arbitrario: la sua densità spettrale di potenza (PSD) **decresce con la frequenza**, come classificato dalla norma **ISO 8608**. La frequenza temporale di eccitazione dipende dalla velocità del veicolo, `f = v·Ω` (velocità × frequenza spaziale): a velocità più alta, la stessa strada eccita frequenze temporali più alte.

## 2. Semplificazione: disturbo bianco

Per il dimensionamento della banda si adotta la semplificazione di assumere **`w = żr` un rumore bianco**, cioè a **PSD costante**.

**Attenzione alla variabile.** È importante assumere bianca la **velocità** `w = żr`, non il profilo `zr`. Infatti:

- se `zr` fosse bianco, allora `w = żr` (la sua derivata) avrebbe PSD crescente come `ω²`, non fisica;
- assumere `w` bianco equivale ad assumere `zr` **rumore bianco integrato** (un cammino casuale), la cui PSD va come `1/ω²`.

Quindi l'ipotesi non è arbitraria: **riproduce il comportamento di una strada reale** (profilo con PSD decrescente, velocità approssimativamente piatta).

## 3. Conseguenza: lo spettro di eccitazione coincide con l'amplificazione del plant

L'eccitazione di una grandezza di uscita è data dal prodotto di **due fattori**: l'amplificazione del sistema e la PSD del disturbo,

$$S_{\text{out}}(\omega)=|G_{wz}(j\omega)|^2\,S_w(\omega),$$

dove `G_{wz}` è il trasferimento disturbo→uscita. Con `S_w(ω)` costante, questa relazione si riduce a

$$S_{\text{out}}(\omega)\ \propto\ |G_{wz}(j\omega)|^2 .$$

Ne segue che, con disturbo bianco, **lo spettro di eccitazione è modellato interamente dal plant**, e poiché il plant amplifica in corrispondenza dei propri **modi**, l'energia in uscita si concentra nelle bande dei modi:

- **modo della carrozzeria ≈ 1.2 Hz** (comfort),
- **modo della ruota ≈ 10 Hz** (road-holding).

È questa la ragione per cui, sotto l'ipotesi di disturbo bianco, è corretto **concentrare la banda di controllo attorno ai modi**.

## 4. Scelta della banda di controllo

La banda di controllo va fissata tenendo conto di **due vincoli opposti**.

**(a) Vincolo inferiore: i modi.** La banda deve coprire le frequenze dove il disturbo eccita davvero le uscite, cioè i modi. Deve quindi estendersi **fino a ~10–15 Hz**, per avere autorità sia sul modo della carrozzeria sia su quello della ruota.

**(b) Vincolo superiore: attuatori e ritardi.** La banda utile non può superare quella consentita dagli attuatori e non può essere rovinata dai ritardi. Nel nostro caso:

- attuatore primario `u1`: banda ≈ **12.7 Hz**;
- attuatore secondario `u2`: banda ≈ **6.4 Hz** (è il collo di bottiglia del road-holding);
- ritardi `τ1 = 5 ms`, `τ2 = 10 ms`, che introducono sfasamento crescente con la frequenza e quindi limitano la banda chiudibile.

Il **modo della ruota (10 Hz)** cade **oltre la banda di `u2`**: questo è il limite strutturale già noto e dichiarato.

**(c) Raffinamento opzionale: sensibilità umana.** La ponderazione ISO 2631 per l'accelerazione verticale non è piatta: ha il massimo attorno a **4–8 Hz**, cioè tra i due modi. Includerla sposta leggermente il baricentro dell'attenzione verso il centro banda. Nel modello semplificato è **trascurata** (si veda la Sezione 5).

## 5. Semplificazioni adottate

Riassumendo, le semplificazioni scelte — da dichiarare nella relazione — sono:

1. **Disturbo bianco sulla velocità**: `w = żr` a PSD costante, equivalente a un profilo stradale integrato (PSD `1/ω²`). Giustificata fisicamente e naturalmente coerente con la struttura di regolazione.
2. **Nessuna ponderazione ISO 2631**: l'attenuazione è specificata ai modi e non corretta con la curva di sensibilità umana.
3. **Banda concentrata sui modi**: l'attenuazione si concentra attorno a 1.2 Hz e 10 Hz, dove il plant amplifica il disturbo bianco.
4. **Coerenza con l'LQG**: l'ipotesi di disturbo bianco si traduce direttamente nella scelta del rumore di processo, `Q_n = B_w B_w^\top\, q`, semplificando la sintesi.

## 6. Limiti e avvertenze della semplificazione

- **Conservatività in alta frequenza.** Un disturbo bianco ha energia anche in alta frequenza, dove la strada reale ne ha poca: l'ipotesi tende a **sovrastimare** il disturbo in alto. Non è dannoso (la banda è comunque limitata dagli attuatori), ma va dichiarato.
- **La banda non dipende dal bianco.** L'ipotesi bianca non cambia il **limite superiore** di banda (attuatori/ritardi): influenza solo il *bersaglio* dell'attenuazione (i modi).
- **Da validare.** La bontà della semplificazione va confermata a posteriori: con un modello di strada più realistico (classe ISO 8608) si può verificare che la banda scelta sia ancora adeguata.

## 7. Sintesi

| Elemento | Scelta |
|---|---|
| Disturbo | `w = żr` (velocità stradale) |
| Modello del disturbo | **rumore bianco** (profilo integrato) |
| Spettro di eccitazione | `∝ |G_{wz}|²` → picchi ai modi (1.2 Hz, 10 Hz) |
| Banda di controllo | copre i modi → **~10–15 Hz** |
| Limite superiore | banda attuatori (`u1` 12.7 Hz, `u2` 6.4 Hz) + ritardi |
| Ponderazione ISO | trascurata (raffinamento opzionale) |
| Traduzione LQG | disturbo bianco → `Q_n = B_w B_wᵀ q` |

## 8. Conclusione

Assumere il disturbo stradale bianco sulla velocità è una semplificazione **legittima e conservativa**: rende lo spettro di eccitazione proporzionale all'amplificazione del plant, giustificando la concentrazione della banda di controllo attorno ai modi del sistema, e si traduce naturalmente nel modello di rumore di processo dell'LQG. I vincoli fisici degli attuatori e dei ritardi restano, indipendenti dall'ipotesi, e fissano il limite superiore della banda effettivamente realizzabile.
