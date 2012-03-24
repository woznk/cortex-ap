/**
  @page Appunti
  
  @verbatim
  @endverbatim

@par Simulazione

     per rendere il codice più semplice, creare un task di lettura dei MEMS
     che invia un messaggio con i valori dei sensori al task attitude.
     In questo modo, il task di lettura dei MEMS è più facilmente sostituibile 
     dal task di telemetria che può inviare i valori di sensori al suo posto.

@par GPS

     Il buffer di ricezione dell'UART non contiene un'intera frase NMEA.
     In funzionamento normale il buffer non si riempie per la bassa velocità 
     di comunicazione. In simulazione, il buffer è "forzato" dalla telemetria
     che lavora a 115200 e viene sovrascritto prima che il task di navigazione
     riesca ad elaborare la stringa.

@par Log

     Il task di log potrebbe funzionare a tempo e salvare su SD i dati utili 
     leggendoli autonomamente mediante funzioni di interfaccia.
     Problemi di inconsistenza dei dati potrebbero essere risolti con semafori
     che "occupano" il dato quando deve essere aggiornato.

@par FAT FS

     Il file system viene montato sia dal task di log che dal task di navigazione.
     Spostare il montaggio del file system nel programma principale.

@par How to use it ? 

@b 

@note
   
 * <h3><center>&copy; Lorentz</center></h3>
 */
