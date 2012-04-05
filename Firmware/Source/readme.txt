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

@par Python socket
     esemio di invio e ricezione stringhe tramite socket in Python:

	################ sender
	import socket

	UDP_IP="127.0.0.1"
	UDP_PORT=5005
	MESSAGE="Hello, World!"
	avalue = 1.234
	DCM = [[1,0,0],[0,0,0],[0,0,0],
	       [0,0,0],[0,1,0],[0,0,0],
	       [0,0,0],[0,0,0],[0,0,1]]

	print "UDP target IP:", UDP_IP
	print "UDP target port:", UDP_PORT
	print "message:", MESSAGE
	print "DCM:", DCM

	sock = socket.socket( socket.AF_INET, # Internet
			      socket.SOCK_DGRAM ) # UDP
	sock.sendto( MESSAGE, (UDP_IP, UDP_PORT) )
	sock.sendto( str(DCM), (UDP_IP, UDP_PORT) )

	################ receiver
	import socket

	UDP_IP="127.0.0.1"
	UDP_PORT=5005

	sock = socket.socket( socket.AF_INET,     # Internet
			      socket.SOCK_DGRAM ) # UDP
	sock.bind( (UDP_IP,UDP_PORT) )

	while True:
	    data, addr = sock.recvfrom( 1024 )    # buffer size is 1024 bytes
	    print "received message:", data

@par How to use it ? 

@b 

@note
   
 * <h3><center>&copy; Lorentz</center></h3>
 */
