# Selezione dei parametri degli attuatori

*Sezione della relazione — Progetto di Controllo Robusto: sospensione attiva quarter-car con due attuatori.*

## Introduzione

Nel modello quarter-car considerato, i due attuatori non sono elementi ideali: la loro dinamica, la loro banda passante e i loro ritardi di trasporto entrano a pieno titolo nel modello del sistema da controllare e ne determinano le prestazioni raggiungibili. La scelta dei parametri degli attuatori — `ωn1`, `ζ1`, `τ1` per l'attuatore primario `u1` (2° ordine con ritardo) e `τa`, `τ2` per l'attuatore secondario `u2` (1° ordine con ritardo) — non è quindi arbitraria, ma va giustificata a partire da ciò che il controllo richiede al sistema. Questa sezione illustra i criteri con cui, nella pratica ingegneristica, si seleziona un attuatore adatto a un dato impianto e mostra come i valori nominali adottati li soddisfino.

## 1. L'attuatore come vincolo di progetto

Ogni attuatore risponde a due domande fondamentali. La prima è di **ampiezza**: è in grado di generare una forza sufficiente a contrastare i disturbi e a imprimere alla massa sospesa l'accelerazione richiesta? La seconda è di **frequenza**: è in grado di muoversi con la rapidità necessaria a inseguire i comandi del controllore?

Da queste due domande discendono tutti i criteri di selezione: la banda passante governa l'autorità in frequenza, la forza governa l'autorità in ampiezza, mentre ritardo, smorzamento, corsa e potenza sono i modi concreti in cui un attuatore fallisce rispetto a questi due requisiti. È importante sottolineare che l'attuatore non è un semplice accessorio: esso è il **vincolo dominante** sulla banda di controllo raggiungibile. Per quanto sofisticato sia il controllore, non si può controllare una dinamica a una frequenza che l'attuatore non è in grado di riprodurre.

## 2. Autorità in frequenza: la banda passante

### 2.1 Il comportamento in frequenza dell'attuatore

L'attuatore primario è descritto da un sistema del secondo ordine con ritardo di trasporto:

$$G_{a1}(s)=\frac{\omega_{n1}^2}{s^2+2\zeta_1\omega_{n1}s+\omega_{n1}^2}\,e^{-\tau_1 s}.$$

Questa funzione di trasferimento descrive un fatto fisico preciso: **quando si comanda una forza, l'attuatore non la produce istantaneamente**. Se si richiede all'attuatore di generare una forza sinusoidale a pulsazione `ω`, la forza effettivamente erogata è la richiesta moltiplicata per `|G_{a1}(jω)|`. Il comportamento al variare della frequenza è il seguente:

- **a bassa frequenza** (`ω ≪ ωn1`): `|G_{a1}| ≈ 1`. L'attuatore riproduce fedelmente il comando ricevuto, sia in ampiezza sia in fase;
- **in prossimità di `ωn1`**: il guadagno scende a `-3 dB` (circa `0.7`) e la fase ha già accumulato `-90°`. L'attuatore comincia a inseguire il comando con un ritardo crescente;
- **oltre `ωn1`**: il guadagno decade come `1/ω²`. L'attuatore non è più in grado di seguire il comando: l'uscita diventa trascurabile rispetto alla richiesta.

In sintesi, l'attuatore si comporta come un **filtro passa-basso applicato ai comandi del controllore**. La conseguenza operativa è netta: si può controllare una dinamica a una certa frequenza soltanto se l'attuatore "passa" quella frequenza. Se il controllore tenta di agire a una frequenza superiore alla banda dell'attuatore, il comando viene emesso attenuato e in ritardo, e lì non vi è alcuna autorità di controllo.

### 2.2 Banda dell'attuatore e reiezione dei disturbi

La capacità di attenuare un disturbo a pulsazione `ω` dipende dal **guadagno d'anello**

$$L(j\omega)=C(j\omega)\,P(j\omega)\,G_a(j\omega),$$

dove `C` è il controllore, `P` il processo meccanico e `G_a` l'attuatore. Se il guadagno `G_a` diminuisce, diminuisce anche `L`, e con esso la reiezione del disturbo a quella frequenza. **La banda dell'attuatore stabilisce quindi fino a quale frequenza è possibile combattere i disturbi.** Poiché i disturbi stradali concentrano la loro energia attorno alle frequenze proprie del veicolo, sono proprio quelle le frequenze alle quali occorre autorità di controllo.

## 3. Le frequenze caratteristiche del sistema

Per stabilire quale banda debbano avere gli attuatori occorre innanzitutto individuare le frequenze alle quali il quarter-car reagisce con maggiore intensità ai disturbi: i suoi **modi risonanti**. Essi si ottengono dalla classica relazione di un sistema massa-molla, `ω = √(k/m)`, applicata alle due masse.

**Modo della massa sospesa (bounce).** La carrozzeria, di massa `ms = 350 kg`, è sostenuta essenzialmente dalla molla di sospensione `ks0 = 20000 N/m`. La sua pulsazione naturale è

$$\omega_s=\sqrt{\frac{k_{s0}}{m_s}}=\sqrt{\frac{20000}{350}}\approx 7.6\ \text{rad/s}\approx 1.2\ \text{Hz}.$$

Questo è il modo del **comfort**: è la frequenza con cui la carrozzeria oscilla verticalmente.

**Modo della massa non sospesa (wheel-hop).** La ruota, di massa `mu = 50 kg`, è compresa tra la molla di sospensione e la molla del pneumatico `kt = 180000 N/m`. Poiché la carrozzeria è molto più pesante e si muove lentamente, la ruota "vede" le due molle in parallelo, e la sua pulsazione naturale è

$$\omega_u=\sqrt{\frac{k_{s0}+k_t}{m_u}}=\sqrt{\frac{200000}{50}}\approx 63\ \text{rad/s}\approx 10\ \text{Hz}.$$

Questo è il modo del **road-holding**: è la frequenza con cui la ruota tende a saltellare, con il rischio di perdita di contatto con la strada.

Questi valori non sono casuali: nella letteratura automobilistica la massa sospesa di un veicolo reale si colloca tipicamente nella banda **1–2 Hz** e quella non sospesa nella banda **10–15 Hz** [7]. Le risonanze sono precisamente le frequenze alle quali l'energia di un dosso stradale viene amplificata: sono quindi le frequenze alle quali il controllo deve necessariamente possedere autorità. Ne segue che **la banda degli attuatori deve trovarsi al di sopra di questi modi**.

## 4. Dal requisito di banda alla scelta dei parametri

Se il controllo deve agire sul modo della carrozzeria (1.2 Hz), un attuatore con banda di circa 12 Hz risulta circa **dieci volte più veloce**: alla frequenza di interesse esso è praticamente trasparente (guadagno `≈ 1`, fase `≈ 0`) e riproduce fedelmente il comando. È questa la ragione della regola pratica secondo cui l'attuatore, o l'eventuale anello interno che lo pilota, deve essere **5–10 volte più veloce** dell'anello che deve servire [5]: non si tratta di una convenzione, ma del margine necessario affinché l'attuatore non introduca attenuazione e ritardo proprio nella banda in cui il controllo lavora. In termini generali, se l'anello di controllo è attivo fino a una pulsazione `ωc`, si richiede `ω_attuatore ≳ 3–10·ωc`.

Applicando questo criterio al caso in esame, per il **comfort** serve una banda ben superiore a `1.2 Hz`, mentre per il **road-holding** serve una banda ben superiore a `10 Hz`.

## 5. Il ritardo di trasporto

Se la banda è il requisito di progetto, il **ritardo di trasporto** `τ` è il fattore che più di ogni altro ne limita la realizzazione; è per questo che `τ1`, `τa` e `τ2` figurano tra i parametri da scegliere.

Il significato fisico del ritardo è che l'attuatore esegue il comando **in ritardo**: il controllo agisce sul presente utilizzando informazioni già invecchiate. L'effetto è intuitivo e paragonabile al tentativo di bilanciare una scopa in equilibrio sul palmo della mano con gli occhi bendati e un ritardo di reazione di mezzo secondo: si osserva l'inclinazione, si corregge, ma la correzione giunge quando la scopa è già passata dall'altra parte; si finisce così per **correggere in eccesso**, innescando oscillazioni crescenti fino all'instabilità.

Dal punto di vista matematico, un ritardo `e^{-τs}` introduce uno sfasamento `−ωτ` che **cresce linearmente con la frequenza** senza aggiungere alcuno smorzamento. Il sistema a retroazione diventa instabile quando la fase complessiva dell'anello raggiunge `−180°`. Il **margine di fase** `PM = 180° + ∠L(jωc)`, che è la misura della robustezza dell'anello, viene quindi ridotto dal ritardo secondo

$$\mathrm{PM}_{\text{eff}}=\mathrm{PM}_0-\omega_c\,\tau .$$

Ne segue la regola pratica di mantenere `ωc·τ` al di sotto di circa `0.3–0.5 rad`, cioè di scegliere un ritardo **piccolo rispetto alla scala di tempo dell'anello** [4]. Un'attuatore del secondo ordine si comporta, ai fini della banda, come un **ritardo equivalente**

$$T_{eq}=\frac{2\zeta}{\omega_n},$$

al quale il ritardo reale si somma. Il legame diretto tra ritardo totale efficace `T_eff` e banda chiudibile è, con criterio di taratura "magnitude optimum",

$$\mathrm{BW}\approx\frac{1}{\sqrt{2}\,T_{eff}}$$

[3]. Ne consegue che **maggiore è il ritardo, minore è la banda ottenibile**: il ritardo, più della forza, costituisce il limite fisico ultimo alle prestazioni.

## 6. Lo smorzamento

Il parametro `ζ1` fissa lo **smorzamento** dell'attuatore primario. Il motivo della sua presenza è nuovamente leggibile dalla funzione `G_{a1}(jω)`: se `ζ1` è piccolo, l'attuatore **risuona su se stesso**, e alla pulsazione `ωn1` il guadagno può superare l'unità, presentando un picco di ampiezza

$$M_r=\frac{1}{2\zeta\sqrt{1-\zeta^2}}\qquad(\zeta<1/\sqrt{2}).$$

Ad esempio, con `ζ = 0.5` il picco vale circa `1.15`, cioè un'amplificazione del 15%; con `ζ = 1/√2 ≈ 0.71` si ottiene `M_r = 1`, ovvero **nessun picco** e risposta in frequenza massimamente piatta. Un picco nell'attuatore introduce, alla relativa frequenza, guadagno aggiuntivo e rotazione di fase all'interno dell'anello, con il rischio di indurre oscillazioni. La scelta `ζ = 1/√2` rappresenta quindi lo smorzamento ottimale canonico [3]: non è un valore arbitrario, bensì quello che rende la risposta il miglior compromesso tra rapidità e sovraelongazione.

## 7. Autorità in ampiezza: la forza

Il requisito di frequenza non è sufficiente: un attuatore veloce ma debole è inutile. La forza necessaria si dimensiona con la seconda legge di Newton, `F = m·a`. Per imprimere alla carrozzeria di `350 kg` un'accelerazione di `1 m/s²` occorrono `350 N`; per contrastare un dosso di `1 cm`, la forza del pneumatico vale all'incirca `k_t·0.01 ≈ 1800 N`. Nelle sospensioni attive reali si lavora quindi nell'ordine dei **kN per angolo ruota**, con consumi di potenza dell'ordine dei **kW** per veicolo [6]. Nel presente progetto la traccia non richiede il dimensionamento della forza (non compare in tabella), ma il vincolo va comunque tenuto presente: la scelta di banda e ritardo non può prescindere dalla capacità dell'attuatore di erogare la forza necessaria.

## 8. Sintesi della selezione e verifica dei valori nominali

Il ragionamento di selezione può essere riassunto nella sequenza seguente: (i) individuare le frequenze caratteristiche del sistema, che sono quelle dei disturbi da respingere; (ii) tradurle in un requisito di banda passante per gli attuatori; (iii) verificare che la forza disponibile sia sufficiente; (iv) contenere il ritardo entro una frazione della scala di tempo dell'anello; (v) scegliere uno smorzamento prossimo al valore di risposta piatta. Applicando tale procedura ai valori nominali della Tabella 1 si ottiene quanto segue.

- **`ωn1 = 80 rad/s ≈ 12.7 Hz`.** La banda dell'attuatore primario è circa dieci volte il modo della carrozzeria (`1.2 Hz`) e supera il modo della ruota (`10 Hz`): garantisce autorità sia sul comfort sia, in misura maggiore, sul road-holding.
- **`ζ1 = 0.7 = 1/√2`.** Smorzamento a risposta piatta, privo di picco risonante.
- **`τ1 = 5 ms`.** Alla pulsazione `ωn1` introduce uno sfasamento `\omega_{n1}\tau_1 = 80\cdot0.005 \approx 0.4\ \text{rad} \approx 23°`, accettabile; il ritardo equivalente `T_{eq} = 2\zeta/\omega_n = 1.4/80 \approx 17.5\ \text{ms}` lascia comunque una banda utile dell'ordine di alcune decine di rad/s.
- **`τa = 25 ms` e `τ2 = 10 ms`.** L'attuatore secondario ha banda `1/τa = 40 rad/s ≈ 6.4 Hz` e ritardo aggiuntivo `τ2 = 10 ms`; il ritardo equivalente complessivo risulta dell'ordine di `τa + τ2 ≈ 35 ms`.

I valori della Tabella 1 soddisfano dunque i criteri di selezione: vi è separazione tra la banda degli attuatori e i modi del sistema, lo smorzamento è ottimale e i ritardi sono piccoli rispetto alle costanti di tempo in gioco.

## 9. Osservazioni critiche

Una lettura attenta evidenzia un punto meritevole di nota. L'attuatore secondario `u2`, preposto al road-holding, presenta una banda (`6.4 Hz`) **inferiore** al modo della ruota (`10 Hz`) e un ritardo non trascurabile (`τ2 = 10 ms`): la sua autorità sul wheel-hop è quindi limitata e la sua dinamica costituisce il collo di bottiglia del road-holding. Questa circostanza non è un errore, ma una scelta realistica — un attuatore al mozzo è intrinsecamente più lento e meno potente di uno idraulico — ed è coerente con il principio che attuatori diversi occupano posizioni diverse sulla frontiera forza-banda [6], [8]. Ai fini del progetto essa va **dichiarata esplicitamente**: spiega perché, nel seguito, il raggiungimento dei requisiti di road-holding sarà più oneroso di quelli di comfort, e perché la dinamica degli attuatori dovrà essere inclusa nell'analisi di robustezza.

## Riferimenti

1. K. J. Åström, R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*. Disponibile: https://fbswiki.org/wiki/index.php/Main_Page
2. S. Skogestad, I. Postlethwaite, *Multivariable Feedback Control: Analysis and Design*. Disponibile: https://skoge.folk.ntnu.no/skoge/book/ps/bookall.pdf
3. Plexim, *Bandwidth Limits in Control Loops: A Delay-Based Perspective*. Disponibile: https://forum.plexim.com/t/bandwidth-limits-in-control-loops-a-delay-based-perspective/3004
4. MathWorks, *Analyze Control Systems with Delays*. Disponibile: https://www.mathworks.com/help/control/ug/analyzing-control-systems-with-delays.html
5. Motion Control Tips, *Why is the bandwidth of a servo control loop important?*. Disponibile: https://www.motioncontroltips.com/why-is-the-bandwidth-of-a-servo-control-loop-important/
6. IEEE Technology Navigator, *Active Suspensions*. Disponibile: https://technav.ieee.org/topic/active-suspensions/
7. N. C. Cheung, *Study of Art of Automotive Active Suspensions*. Disponibile: http://www.norbert.idv.hk/Files_Papers/C101.pdf
8. H. Peng et al., *Analysis of Active Suspension Systems with Hydraulic Actuators*, University of Michigan. Disponibile: https://huei.engin.umich.edu/wp-content/uploads/sites/186/2015/02/IAVSD-2003.pdf
9. *Semi-Active Suspension Systems: From Advanced Control Algorithms to Vehicle Dynamics*, MDPI Sensors. Disponibile: https://www.mdpi.com/1424-8220/26/15/4736
10. *Advances in Active Suspension Systems*, ScienceDirect. Disponibile: https://www.sciencedirect.com/science/article/pii/S2095809923003983
