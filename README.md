A prototype for wireless communication between a server ESP and multiple client ESPs. 

Setup:

Flash server.ino to server ESP. Copy the MAC Address from the serial and paste it into uint8_t SERVER_MAC[6] on the client. 
Flash client.ino to the client(s) ESP. Edit CLIENT_ID and MY_ROLE as necessary. 
Check HELLO RECEIVED on server and HELLO SENT on client
Charge GPIO 25 (Datalogger) or GPIO 26 (Sampler) on the server ESP.
Expected output:

Server:              
ACK role={#} trigger_id={#}
ACK confirmed from client {#}
all ACKs received {continuous}

Client:
Triggered: {#}

Test stale client purge:

Remove client ESP from power. Expected: Removing stale client ID={#} on server. 


