A prototype for wireless communication between a server ESP and multiple client ESPs. 

Setup:

Flash server.ino to server ESP. Copy the MAC Address from the serial and paste it into uint8_t SERVER_MAC[6] on the client. 
Flash client.ino to the client(s) ESP. Edit CLIENT_ID and MY_ROLE as necessary. 
Check HELLO RECEIVED on server and HELLO SENT on client
Charge GPIO 38 (Datalogger) or GPIO 39 (Sampler) on the server ESP.
Expected output:

Server:              
ACK role={#} trigger_id={#}
ACK confirmed from client {#}
all ACKs received {continuous}

Client:
Triggered: {#}

Test stale client purge:

Remove client ESP from power. Expected: Removing stale client ID={#} on server (after CLIENT_TIMEOUT_MS, 200 s). 

Sensor buoy (aquas-sensor-buoy-esp32) on-demand readings:

The buoy only takes a reading when the server triggers one. It spends the rest of the time in deep sleep and wakes every CHECKIN_INTERVAL_US (60 s) to send a HELLO. The server replies with the pending trigger, or MSG_NOOP if there is none, and the buoy goes straight back to sleep.
A trigger can take up to 60 s to reach the buoy. The server holds a trigger for TRIGGER_TTL_MS (180 s) before logging it as expired.
A triggered reading takes about 30 s. The server then prints DATA followed by STAT (status 1 = data delivered, 0 = data send failed).
