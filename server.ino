#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define VALID_PH    (1 << 0)
#define VALID_TEMP  (1 << 1)
#define VALID_DO    (1 << 2)
#define VALID_EC    (1 << 3)
#define VALID_TURB  (1 << 4)


 


enum MsgType : uint8_t {
  MSG_HELLO   = 1,
  MSG_TRIGGER = 2,
  MSG_ACK     = 3,
  MSG_STAT     = 4,
  MSG_DATA      = 5,
  MSG_NOOP      = 6   // reply to HELLO when no trigger is pending
};

enum ClientRole : uint8_t {
  ROLE_SAMPLER    = 1,
  ROLE_DATALOGGER = 2,
};


struct __attribute__((packed)) Packet {
  uint8_t  msg_type;
  uint8_t  role;
  uint32_t client_id;
  uint32_t trigger_id;
  uint32_t secret_tag; // optional
  uint32_t status;
};

struct __attribute__((packed)) DataPacket {
  // Header -- same first fields as Packet, so the server can switch on msg_type
  uint8_t  msg_type;      // MSG_DATA
  uint8_t  role;
  uint32_t client_id;
  uint32_t secret_tag;
  uint32_t seq;           // increments every send; lets the server spot drops/dupes
 
  // RTC timestamp
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
 
  // Sensor readings (parsed from the EZO text buffers)
  float    ph;
  float    temperature_c;
  float    dissolved_oxygen;
  float    conductivity;
  float    turbidity_ntu;
 
  uint8_t  valid_mask;    // which of the readings above are real values
  int32_t  error_code;    // system_error_code
}; 

static_assert(sizeof(DataPacket) <= 250, "DataPacket exceeds ESP-NOW payload limit");



// How long a trigger stays pending, waiting for sleeping clients to check in.
// Must exceed the client check-in interval (60 s) plus a reading cycle (~40 s).
static const uint32_t TRIGGER_TTL_MS = 180000;  // 3 minutes

static const uint32_t SECRET_TAG = 0xA0A00001;
// GPIO config
static const int GPIO_DATALOGGER_TRIGGER = 38;
static const int GPIO_SAMPLER_TRIGGER    = 39;

struct ClientInfo {
  uint8_t mac[6];
  uint8_t role;
  uint32_t client_id;
  uint32_t lastSeenMs;
};
//timeout constant for stale client purge
// Sleeping clients check in every 60 s and go quiet for ~40 s while reading
static const uint32_t CLIENT_TIMEOUT_MS = 200000;  // 200 seconds


static const int MAX_CLIENTS = 10;
ClientInfo clients[MAX_CLIENTS];
int clientCount = 0;

//track which clients have sent ACK
struct PendingTrigger {
  bool active;
  uint8_t role;
  uint32_t trigger_id;
  bool ackReceived[MAX_CLIENTS];
  uint32_t createdMs;
  bool expected[MAX_CLIENTS];
};
//Samplers
PendingTrigger pendingSampler{};
//Triggers
PendingTrigger pendingDatalogger{};

uint32_t triggerCounterSampler = 0;
uint32_t triggerCounterDatalogger = 0;

bool samplerWasHigh = false;
bool dataloggerWasHigh = false;

PendingTrigger* pendingFor(uint8_t role) {
  return (role == ROLE_SAMPLER) ? &pendingSampler : &pendingDatalogger;
}

void sendControl(const uint8_t* mac, uint8_t msgType, uint8_t role, uint32_t triggerId) {
  Packet t{};
  t.msg_type = msgType;
  t.role = role;
  t.client_id = 0; // not needed from server
  t.trigger_id = triggerId;
  t.secret_tag = SECRET_TAG;
  esp_now_send(mac, (uint8_t*)&t, sizeof(Packet));
}

// Returns the client's index, or -1 if the table is full
int upsertClient(const uint8_t* mac, uint8_t role, uint32_t client_id) {
  for (int i = 0; i < clientCount; i++) {
    if (memcmp(clients[i].mac, mac, 6) == 0) {
      clients[i].role = role;
      clients[i].client_id = client_id;
      clients[i].lastSeenMs = millis();
      return i;
    }
  }
  if (clientCount >= MAX_CLIENTS) return -1;
  memcpy(clients[clientCount].mac, mac, 6);
  clients[clientCount].role = role;
  clients[clientCount].client_id = client_id;
  clients[clientCount].lastSeenMs = millis();

  //add peer
  esp_now_peer_info_t peer{}; 
  memcpy(peer.peer_addr, mac, 6); 
  peer.channel = 0; 
  peer.encrypt = false; 
  esp_now_add_peer(&peer);

  // Fresh slot: clear any trigger state left over from a removed client
  pendingSampler.ackReceived[clientCount] = pendingSampler.expected[clientCount] = false;
  pendingDatalogger.ackReceived[clientCount] = pendingDatalogger.expected[clientCount] = false;
  return clientCount++;
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len < 1) return;
  uint8_t msg_type = data[0];

  // ---------- Sensor data ----------
  if (msg_type == MSG_DATA) {
    if (len != sizeof(DataPacket)) return;
    DataPacket d;
    memcpy(&d, data, sizeof(DataPacket));
    if (d.secret_tag != SECRET_TAG) return;

    upsertClient(info->src_addr, d.role, d.client_id);  // any packet counts as "seen"
    handleDataPacket(info->src_addr, d);
    return;
  }

  // ---------- Control packets (HELLO, ACK, ...) ----------
  if (len != sizeof(Packet)) return;
  Packet p;
  memcpy(&p, data, sizeof(Packet));
  if (p.secret_tag != SECRET_TAG) return;

  int idx = upsertClient(info->src_addr, p.role, p.client_id);  // any packet counts as "seen"

  if (p.msg_type == MSG_HELLO) {
    Serial.println("HELLO received");

    // Sleeping clients only listen right after a HELLO, so answer every HELLO:
    // the pending trigger if this client hasn't ACKed it yet, otherwise NOOP
    // (lets the client go back to sleep immediately).
    PendingTrigger* pt = pendingFor(p.role);
    if (idx >= 0 && pt->active && !pt->ackReceived[idx]) {
      pt->expected[idx] = true;  // include clients that joined after the trigger
      sendControl(info->src_addr, MSG_TRIGGER, p.role, pt->trigger_id);
      Serial.print("Delivered pending trigger_id=");
      Serial.println(pt->trigger_id);
    } else {
      sendControl(info->src_addr, MSG_NOOP, p.role, 0);
    }

  } else if (p.msg_type == MSG_STAT) {
    Serial.print("STAT from client "); Serial.print(p.client_id);
    Serial.print(" trigger_id="); Serial.print(p.trigger_id);
    Serial.print(" status="); Serial.println(p.status);

  } else if (p.msg_type == MSG_ACK) {
    PendingTrigger* pt = pendingFor(p.role);
    if (!pt->active) {
      Serial.println("role not active");
      return;
    }
    if (p.trigger_id != pt->trigger_id) {
      Serial.println("wrong trigger_id");
      return;
    }
    for (int i = 0; i < clientCount; i++) {
      if (clients[i].client_id == p.client_id && clients[i].role == p.role) {
        pt->ackReceived[i] = true;
        Serial.print("ACK confirmed from client ");
        Serial.println(p.client_id);
      }
    }
    Serial.print("ACK role="); Serial.print(p.role);
    Serial.print(" trigger_id="); Serial.println(p.trigger_id);
  }
}

void handleDataPacket(const uint8_t* mac, const DataPacket& d) {
  Serial.printf("DATA from client %lu (seq %lu) %04u-%02u-%02u %02u:%02u:%02u\n",
                (unsigned long)d.client_id, (unsigned long)d.seq,
                d.year, d.month, d.day, d.hour, d.minute, d.second);
  if (d.valid_mask & VALID_PH)   Serial.printf("  pH:   %.3f\n", d.ph);
  if (d.valid_mask & VALID_TEMP) Serial.printf("  Temp: %.2f\n", d.temperature_c);
  if (d.valid_mask & VALID_DO)   Serial.printf("  DO:   %.2f\n", d.dissolved_oxygen);
  if (d.valid_mask & VALID_EC)   Serial.printf("  EC:   %.1f\n", d.conductivity);
  if (d.valid_mask & VALID_TURB) Serial.printf("  NTU:  %.2f\n", d.turbidity_ntu);
  Serial.printf("  Error code: %ld\n", (long)d.error_code);
}

bool initEspNowServer() {
  WiFi.mode(WIFI_STA);
  Serial.print("Server MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  return true;
}

// Record a pending trigger for the role and send it right away to known clients.
// Awake clients (e.g. the sampler) get it immediately; sleeping clients get it
// as the reply to their next HELLO, until TRIGGER_TTL_MS expires.
void sendTriggerToRole(uint8_t role, uint32_t trigger_id) {
  PendingTrigger* pt = pendingFor(role);

  pt->active = true;
  pt->role = role;
  pt->trigger_id = trigger_id;
  pt->createdMs = millis();

  //set all clients to ack received = false, AND maintain a list of the expected clients
  for (int i = 0; i < MAX_CLIENTS; i++) {
    pt->ackReceived[i] = false;

    if (i < clientCount && clients[i].role == role) {
      pt->expected[i] = true;
    } else {
      pt->expected[i] = false;
    }
  }
  for (int i = 0; i < clientCount; i++) {
    if (clients[i].role != role) continue;
    sendControl(clients[i].mac, MSG_TRIGGER, role, trigger_id);  // peer added in upsertClient
  }
}

// Complete the trigger once every expected client has ACKed, or expire it after TRIGGER_TTL_MS.
// No timed resends: sleeping clients get the trigger again on their next HELLO.
void checkPending(PendingTrigger* pt) {

  if (!pt->active) return;

  bool anyExpected = false;
  bool allAcked = true;
  for (int i = 0; i < clientCount; i++) {
    //check expected and ack received - don't need to check role (include in expected)
    if (!pt->expected[i]) continue;
    anyExpected = true;
    if (!pt->ackReceived[i]) allAcked = false;
  }

  if (anyExpected && allAcked) {
    Serial.println("all ACKs received");
    pt->active = false;
    return;
  }

  if (millis() - pt->createdMs > TRIGGER_TTL_MS) {
    Serial.print("Trigger EXPIRED role="); Serial.print(pt->role);
    Serial.print(" trigger_id="); Serial.println(pt->trigger_id);
    if (!anyExpected) {
      Serial.println("  no client checked in");
    }
    for (int i = 0; i < clientCount; i++) {
      if (pt->expected[i] && !pt->ackReceived[i]) {
        Serial.print("  no ACK from client ");
        Serial.println(clients[i].client_id);
      }
    }
    pt->active = false;
  }
}

//remove a client
void removeClient(int index) {

  Serial.print("Removing stale client ID=");
  Serial.println(clients[index].client_id);

  // Remove ESP-NOW peer
  esp_now_del_peer(clients[index].mac);

  // Shift array left
  for (int i = index; i < clientCount - 1; i++) {
    clients[i] = clients[i + 1];

    // Also shift ACK/expected state for pending triggers
    pendingSampler.ackReceived[i] = pendingSampler.ackReceived[i + 1];
    pendingDatalogger.ackReceived[i] = pendingDatalogger.ackReceived[i + 1];
    pendingSampler.expected[i] = pendingSampler.expected[i + 1];
    pendingDatalogger.expected[i] = pendingDatalogger.expected[i + 1];
  }

  clientCount--;
}
//remove any stale client
void purgeStaleClients() {

  for (int i = 0; i < clientCount; ) {

    uint32_t age = millis() - clients[i].lastSeenMs;

    if (age > CLIENT_TIMEOUT_MS) {
      removeClient(i);
      // DO NOT increment i
      // because array shifted
    } else {
      i++;
    }
  }
}



void setup() {
  Serial.begin(115200);
  
  delay(1000);
  
  Serial.println("BOOT");
  Serial.println("server");
  
  pinMode(GPIO_DATALOGGER_TRIGGER, INPUT);
  pinMode(GPIO_SAMPLER_TRIGGER, INPUT);

  //set stable mac address
  WiFi.mode(WIFI_STA);  
  uint8_t newMAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  esp_wifi_set_mac(WIFI_IF_STA, newMAC);


  //print mac address to copy to client
   
  delay(100);

  Serial.print("Server MAC Address: ");
  Serial.println(WiFi.macAddress());

  while (!initEspNowServer()) {
    delay(500);
  }

  // Random start so trigger IDs don't repeat after a server reboot
  // (clients ignore a trigger_id they've already handled)
  triggerCounterSampler = esp_random();
  triggerCounterDatalogger = esp_random();

}

void loop() {
  
  
  bool samplerHigh = (digitalRead(GPIO_SAMPLER_TRIGGER) == HIGH);
  bool dataloggerHigh = (digitalRead(GPIO_DATALOGGER_TRIGGER) == HIGH);

  // Rising edge detect
  if (samplerHigh && !samplerWasHigh) {
    triggerCounterSampler++;
    sendTriggerToRole(ROLE_SAMPLER, triggerCounterSampler);
  }
  if (dataloggerHigh && !dataloggerWasHigh) {
    triggerCounterDatalogger++;
    sendTriggerToRole(ROLE_DATALOGGER, triggerCounterDatalogger);
  }
  //check for unACKed clients
  checkPending(&pendingSampler);
  checkPending(&pendingDatalogger);



  samplerWasHigh = samplerHigh;
  dataloggerWasHigh = dataloggerHigh;

  //purge clients that haven't sent an ACK in the past 2 seconds
  static uint32_t lastPurgeMs = 0;
  if (millis() - lastPurgeMs > 2000) {  // every 2 seconds
    purgeStaleClients();
    lastPurgeMs = millis();
  }
  
   

}
