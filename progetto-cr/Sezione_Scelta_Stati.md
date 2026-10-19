# Scelta degli stati e modello del sistema

*Sezione della relazione — Progetto di Controllo Robusto: sospensione attiva quarter-car con due attuatori.*

## Introduzione

Questa sezione definisce il **vettore di stato** del modello quarter-car aumentato con i due attuatori e ne presenta la rappresentazione in spazio di stato. La scelta degli stati non è neutra: essa determina l'ordine del modello, il modo in cui entra il disturbo stradale e la facilità con cui si esprimono i segnali di prestazione. Si mostra come, adottando **coordinate relative**, il disturbo esogeno `w = żr` entri direttamente nella dinamica senza richiedere uno stato aggiuntivo per il profilo stradale, e come i ritardi di trasporto vengano inclusi tramite **approssimazione di Padé del 1° ordine**.

## 1. Scelta delle coordinate: relative invece che assolute

Il modello fisico è descritto dalle posizioni assolute `zs` (massa sospesa) e `zu` (massa non sospesa) e dalle rispettive velocità. Tuttavia, la dinamica dipende solo dalle **deformazioni relative**

$$\delta_s=z_s-z_u\quad\text{(escursione sospensione)},\qquad \delta_t=z_u-z_r\quad\text{(deformazione pneumatico)},$$

mentre la posizione assoluta dell'intero sistema non ha alcun ruolo, perché la strada `zr` è un **ingresso imposto** (non un corpo libero). Conviene quindi adottare come stati meccanici le coordinate relative `δs` e `δt` insieme alle velocità `żs` e `żu`:

$$\big[\,\delta_s,\ \dot z_s,\ \delta_t,\ \dot z_u\,\big].$$

Il vantaggio è duplice:
1. i segnali di prestazione `δs` (rattle space) e `δt` (road-holding) diventano **direttamente stati**;
2. il disturbo `w = żr` entra **direttamente** nella dinamica (si veda la Sezione 4), quindi **non serve uno stato per `zr`**.

## 2. Sottosistema meccanico (4 stati)

Dalla seconda legge di Newton, con `Fs = ks0 δs + α δs³ + bs δ̇s` e `Ft = kt δt + bt δ̇t`:

$$m_s\ddot z_s=-F_s+u_1,\qquad m_u\ddot z_u=F_s-F_t-u_1+u_2 .$$

Derivando le deformazioni relative e ricordando che `δ̇s = żs − żu` e `δ̇t = żu − żr = żu − w`:

$$
\begin{aligned}
\dot\delta_s &= \dot z_s-\dot z_u,\\[2pt]
\ddot z_s &= \frac{-F_s+u_1}{m_s},\\[2pt]
\dot\delta_t &= \dot z_u-w,\\[2pt]
\ddot z_u &= \frac{F_s-F_t-u_1+u_2}{m_u}.
\end{aligned}
$$

I quattro stati meccanici sono pertanto `x1 = δs`, `x2 = żs`, `x3 = δt`, `x4 = żu`.

## 3. Modello degli attuatori e approssimazione dei ritardi (5 stati)

### 3.1 Attuatore primario `u1` — 2° ordine + ritardo `τ1` (3 stati)

$$G_{a1}(s)=\frac{\omega_{n1}^2}{s^2+2\zeta_1\omega_{n1}s+\omega_{n1}^2}\,e^{-\tau_1 s}.$$

La parte del 2° ordine si realizza con due stati `x5 = u1`, `x6 = u̇1` (forma canonica controllabile):

$$
\dot x_5 = x_6,\qquad
\dot x_6 = -\omega_{n1}^2 x_5 - 2\zeta_1\omega_{n1}x_6 + \omega_{n1}^2\,u_{1,\text{rit}} ,
$$

dove `u1,rit` è il comando ritardato. Il ritardo è approssimato con Padé del 1° ordine

$$e^{-\tau_1 s}\approx\frac{1-\tfrac{\tau_1}{2}s}{1+\tfrac{\tau_1}{2}s},$$

che in spazio di stato, con lo stato `x7`, dà

$$
\dot x_7 = -\frac{2}{\tau_1}x_7 + \frac{2}{\tau_1}u_{1c},
\qquad
u_{1,\text{rit}} = 2x_7 - u_{1c}.
$$

Il Padé introduce uno **zero a fase non minima** in `s = 2/τ1`, che cattura correttamente il **limite di banda** imposto dal ritardo.

### 3.2 Attuatore secondario `u2` — 1° ordine + ritardo `τ2` (2 stati)

$$G_{a2}(s)=\frac{1}{1+\tau_a s}\,e^{-\tau_2 s}.$$

La parte del 1° ordine, con stato `x8 = u2`:

$$\dot x_8 = -\frac{1}{\tau_a}x_8 + \frac{1}{\tau_a}\,u_{2,\text{rit}} ,$$

e il Padé del ritardo, con stato `x9`:

$$
\dot x_9 = -\frac{2}{\tau_2}x_9 + \frac{2}{\tau_2}u_{2c},
\qquad
u_{2,\text{rit}} = 2x_9 - u_{2c}.
$$

## 4. Ingresso del disturbo stradale

Il disturbo esogeno è la **velocità** del profilo stradale, `w = żr`. Esso entra nella dinamica in **due punti**, attraverso i due termini della forza del pneumatico:

- nel termine elastico `kt δt` — ma questo dipende solo dallo stato `δt`, non da `zr` separatamente;
- nel termine smorzante `bt δ̇t = bt(żu − w)` — dove compare **direttamente** `w`;
- nell'equazione di `δt`, poiché `δ̇t = żu − w`.

Grazie all'uso delle **coordinate relative**, `zr` **non compare mai da solo**: appare solo tramite `w`. Ne segue che **non è necessario introdurre uno stato per il profilo stradale**, e il disturbo `w` è un semplice ingresso esogeno del modello.

## 5. Vettore di stato completo e conteggio

| # | Stato | Significato | Sottosistema |
|---|---|---|---|
| `x1` | `δs` | escursione sospensione | meccanico |
| `x2` | `żs` | velocità massa sospesa | meccanico |
| `x3` | `δt` | deformazione pneumatico | meccanico |
| `x4` | `żu` | velocità massa non sospesa | meccanico |
| `x5` | `u1` | forza attuatore primario | attuatore 1 (2° ordine) |
| `x6` | `u̇1` | derivata della forza attuatore 1 | attuatore 1 (2° ordine) |
| `x7` | `p1` | stato di Padé del ritardo `τ1` | attuatore 1 (ritardo) |
| `x8` | `u2` | forza attuatore secondario | attuatore 2 (1° ordine) |
| `x9` | `p2` | stato di Padé del ritardo `τ2` | attuatore 2 (ritardo) |

**Totale: 9 stati** = 4 (meccanico) + 3 (attuatore 1) + 2 (attuatore 2).

La traccia indica un plant aumentato di ordine `4 + 2 + 1 = 7`: si tratta del conteggio **senza** gli stati di ritardo (Padé). Includendo i ritardi — scelta adottata per fedeltà, e necessaria perché i ritardi siano trattabili come **incertezze parametriche** su `τ1`, `τ2` — l'ordine sale a **9**. Il profilo stradale non aggiunge stati grazie alle coordinate relative.

## 6. Rappresentazione in spazio di stato

Raccogliendo le equazioni precedenti, il sistema assume la forma

$$\dot x = A\,x + B_u\begin{bmatrix}u_{1c}\\u_{2c}\end{bmatrix} + B_w\,w,\qquad
y = C_y\,x,\qquad z = C_z\,x + D_{zu}\begin{bmatrix}u_{1c}\\u_{2c}\end{bmatrix},$$

con:

- **ingressi di controllo**: `u1c`, `u2c` (comandi agli attuatori);
- **ingresso di disturbo**: `w = żr`;
- **uscite di misura** `y`: da definirsi in base ai sensori scelti;
- **uscite di prestazione** `z`: `z̈s` (comfort), `δt` (road-holding), `δs` (rattle space), `u1`, `u2` (sforzo attuatori).

Le equazioni esplicite sono:

$$
\begin{aligned}
\dot x_1 &= x_2-x_4,\\
\dot x_2 &= \frac{-\big(k_{s0}x_1+\alpha x_1^3+b_s(x_2-x_4)\big)+x_5}{m_s},\\
\dot x_3 &= x_4-w,\\
\dot x_4 &= \frac{\big(k_{s0}x_1+\alpha x_1^3+b_s(x_2-x_4)\big)-\big(k_t x_3+b_t(x_4-w)\big)-x_5+x_8}{m_u},\\
\dot x_5 &= x_6,\\
\dot x_6 &= -\omega_{n1}^2 x_5-2\zeta_1\omega_{n1}x_6+\omega_{n1}^2(2x_7-u_{1c}),\\
\dot x_7 &= -\frac{2}{\tau_1}x_7+\frac{2}{\tau_1}u_{1c},\\
\dot x_8 &= -\frac{1}{\tau_a}x_8+\frac{1}{\tau_a}(2x_9-u_{2c}),\\
\dot x_9 &= -\frac{2}{\tau_2}x_9+\frac{2}{\tau_2}u_{2c}.
\end{aligned}
$$

## 7. Linearizzazione

Il modello è non lineare per la presenza del termine di Duffing `α x1³`. Si linearizza attorno all'**equilibrio** (`x = 0`, corrispondente a deformazioni nulle e attuatori a riposo), dove

$$\frac{\partial}{\partial x_1}\big(\alpha x_1^3\big)=3\alpha x_1^2\Big|_{x_1=0}=0 .$$

Il termine cubico **scompare** nella linearizzazione e il modello diventa lineare, con `Fs = ks0 x1 + bs(x2 − x4)`. La nonlinearità `α` resta rilevante solo nell'analisi non lineare (RAS e simulazioni a grande segnale).

## 8. Modello di verifica

Il modello a 9 stati con Padé è il modello di **sintesi**. Per la **verifica** si utilizzano:
- il modello con il **ritardo esatto** `e^{-τ1 s}`, `e^{-τ2 s}` (il Padé è accurato solo a bassa frequenza);
- il modello **non lineare** completo (con il termine `α x1³`), per la regione di stabilità asintotica e le simulazioni Monte-Carlo.
