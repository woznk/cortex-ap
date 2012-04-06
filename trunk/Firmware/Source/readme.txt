/**
  @page Appunti
  
  @verbatim
  @endverbatim

@par Simulazione

     Per semplificare il codice, creare un task di lettura dei MEMS che accoda
     un messaggio al task attitude con i valori dei sensori.
     In questo modo, il task di lettura dei MEMS è più facilmente sostituibile 
     dal task di telemetria che può inviare i valori di sensori al suo posto.

@par Log

     Il task di log potrebbe funzionare a tempo e salvare su SD i dati utili 
     leggendoli autonomamente mediante funzioni di interfaccia.
     Problemi di inconsistenza dei dati potrebbero essere risolti con semafori
     che "occupano" il dato quando deve essere aggiornato.

@par FAT FS

     Il file system viene montato sia dal task di log che dal task di navigazione.
     Spostare il montaggio del file system nel programma principale.

@par Altimetro

     
@par Navigation
     
     Calcolare le differenze di longitudine e latitudine come: 

         Delta Lat = Lat2 - Lat1
         Delta Lon = (Lon2 - Lon1) * cos((Lat1 + Lat2)/2)

     Calcolare la distanza dal punto come:

         Distance = sqrt(Delta Lon ^ 2 + Delta Lat ^ 2) * 111320

@par How to use it ? 

@b 

@note
   
 * <h3><center>&copy; Lorentz</center></h3>
 */
