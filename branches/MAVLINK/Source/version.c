//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \file
///             Version history
//
//  CHANGES     --------------------------- BETA01 ----------------------------
//              - FUNZIONA SOLO ROVESCIANDO IL SEGNO DELL'ACCELERAZIONE X E DEL
//                BECCHEGGIO RISPETTO ALLA VERSIONE CHE GIRA SU VISUAL STUDIO!!
//                (Non ho capito perchè, il codice è identico. Verificato sia
//                la trasmissione e la ricezione dei valori dei sensori, che
//                l'ordine di invio della matrice DCM al programma python)
//              - adottato orientamento assi di UAVdevBoard
//                (vedi intestazione di simulation.c)
//              - rinormalizzazione e compensazione della deriva ogni 20 ms
//              - regolati i guadagni della compensazione (pitch+roll e yaw)
//                per la simulazione con l'aeromodello di Xplane
//              DUBBI:
//              -
//              DA FARE:
//              - regolare meglio i guadagni
//
//              --------------------------- BETA02 ----------------------------
//              - Parte dei problemi si sono risolti modificando il programma
//                visualizzazione in Python.
//              - Rinunciato a tradurre direttamente codice di Premerlani.
//              - Confrontato il codice con l'ultima versione di ArduIMU.
//              - Verificato l'orientamento degli assi:
//                  ArduIMU
//                      X positivo in avanti
//                      Y positivo verso destra
//                      Z positivo verso il basso
//                      beccheggio positivo con muso in alto
//                      rollio positivo con ala dx in basso
//                      imbardata positiva in senso orario
//                  X-Plane
//                      X positivo indietro
//                      Y positivo verso sinistra
//                      Z positivo verso il basso
//                      beccheggio positivo con muso in alto
//                      rollio positivo con ala dx in basso
//                      imbardata positiva in senso orario
//              - Corretto l'ordine e il segno dei sensori del simulatore
//              - Corretti errori nel calcolo di Accel_Adjust e Compensate_Drift
//              - Aileron control:
//                  modificato i calcoli con l'orientamento degli assi del
//                  codice ArduIMU.
//              - Elevator control:
//                  modificato il calcolo di pitch e di pitch rate con
//                  l'orientamento degli assi del codice ArduIMU.
//                  KS1 deve avere un valore alto (0.5) altrimenti l'aereo
//                  non risponde.
//              - Guadagni:
//                  giroscopi troppo sensibili, riportato GYRO_GAIN a 0.6
//                  vibrazione durante le virate, azzerati ROLL_KP e ROLL_KD
//                  picchia durante le virate, aumentato PITCH_KP e abbassato
//                  PITCH_KD
//
//              --------------------------- BETA03 ----------------------------
//              - modificati i calcoli di AileronCtrl in seguito alla verifica
//                degli assi
//              - ripristinata la sottrazione del feedback proporzionale di
//                pitch nel calcolo di aileron_accum: funziona solo cosi'.
//              - abbassato PITCHROLL_KI a 0.000002
//              - aggiunta la miscelazione tra elevatore e alettoni per il
//                controllo del beccheggio durante le virate
//              - abbassato DIR_KP a 0.015 per ridurre sovrelongazione durante
//                le virate
//              - alzato DIR_KI a 0.05f per aumentare precisione di traiettoria
//              - abbassato DIR_KD a 0.2 per prova
//              - aumentati ROLL_KP a 0.1 e ROLL_KD a 0.12 dopo la correzione
//                di segno in AileronCtrl
//              - aggiunto filtraggio sensori nel main
//              - modificata stringa GPS per invio dati di satellite validi
//              DA FARE:
//              - implementare gain scheduling che hanno aggiunto nell'ultima
//                revisione del codice ArduIMU (compensazione deriva pitch e
//                roll, guadagno degli accelerometri aumentato quando indicano
//                quasi 1 g)
//
//              --------------------------- BETA04 ----------------------------
//              - navigazione e gestione waypoints spostata da gps.c a nav.c(pp)
//              - modificata numerazione waypoints:
//                  0 corrisponde al punto di lancio, 1 al primo waypoint
//                  in assenza di waypoint funziona solo RTL
//              - aumentato DIR_KP a 0.04 per diminuire raggio di curvatura
//                abbassato DIR_KI a 0.04 dopo prove con simulatore
//                aumentato DIR_KD a 0.25 dopo prove con simulatore
//              - aggiunto suffisso 'f' a tutte le funzioni su float:
//                  cos(), sin(), sqrt() -> cosf(), sinf(), sqrtf()
//              DA FARE:
//              - implementare gain scheduling che hanno aggiunto nell'ultima
//                revisione del codice ArduIMU (compensazione deriva pitch e
//                roll, guadagno degli accelerometri aumentato quando indicano
//                quasi 1 g)
//
//============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

