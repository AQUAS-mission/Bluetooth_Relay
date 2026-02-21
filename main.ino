
#include <WiFi.h>
#include <esp_now.h>

enum MsgType : uint8_t {
  MSG_HELLO   = 1,
  MSG_TRIGGER = 2,
  MSG_ACK     = 3,
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
};

//track which client's have sent ACK
struct PendingTrigger {
  bool active;
  uint8_t role;
  uint32_t trigger_id;
  bool ackReceived[MAX_CLIENTS];
  uint32_t sentTimeMs;
  int retryCount;
  bool expected[MAX_CLIENTS];
};

//Samplers
PendingTrigger pendingSampler{};
//Triggers
PendingTrigger pendingDatalogger{};
//retry constants
static const uint32_t RETRY_INTERVALS_MS[] = {150, 300, 600};
static const int MAX_RETRIES = 3;

static const uint32_t SECRET_TAG = 0xAQUA0001;
// GPIO config
static const int GPIO_DATALOGGER_TRIGGER = 25;
static const int GPIO_SAMPLER_TRIGGER    = 26;

struct ClientInfo {
  uint8_t mac[6];
  uint8_t role;
  uint32_t client_id;
  uint32_t lastSeenMs;
};
//timeout constant for stale client purge
static const uint32_t CLIENT_TIMEOUT_MS = 10000;  // 10 seconds


static const int MAX_CLIENTS = 10;
ClientInfo clients[MAX_CLIENTS];
int clientCount = 0;

uint32_t triggerCounterSampler = 0;
uint32_t triggerCounterDatalogger = 0;

bool samplerWasHigh = false;
bool dataloggerWasHigh = false;

void upsertClient(const uint8_t* mac, uint8_t role, uint32_t client_id) {
  for (int i = 0; i < clientCount; i++) {
    if (memcmp(clients[i].mac, mac, 6) == 0) {
      clients[i].role = role;
      clients[i].client_id = client_id;
      clients[i].lastSeenMs = millis();
      return;
    }
  }
  if (clientCount >= MAX_CLIENTS) return;
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
  clientCount++;
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(Packet)) return;
  Packet p;
  memcpy(&p, data, sizeof(Packet));

  if (p.secret_tag != SECRET_TAG) return;

  if (p.msg_type == MSG_HELLO) {
    upsertClient(info->src_addr, p.role, p.client_id);
    Serial.println("HELLO received");
  } else if (p.msg_type == MSG_ACK) {
    PendingTrigger* pt = (p.role == ROLE_SAMPLER)
                       ? &pendingSampler
                       : &pendingDatalogger;
    if (!pt->active) {
      Serial.print("role not active");
      return;
    }
    if (p.trigger_id != pt->trigger_id){
      Serial.print("wrong trigger_id");
      return;
    } 
    for (int i = 0; i < clientCount; i++){
      if (clients[i].client_id == p.client_id && clients[i].role == p.role)
        pt->ackReceived[i] = true;
      Serial.print("ACK confirmed from client ");
      Serial.println(p.client_id);
    }
    Serial.print("ACK role="); Serial.print(p.role);
    Serial.print(" trigger_id="); Serial.println(p.trigger_id);
  }
}

bool initEspNowServer() {
  WiFi.mode(WIFI_STA);
  Serial.print("Server MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  return true;
}

void sendTriggerToRole(uint8_t role, uint32_t trigger_id) {
  PendingTrigger* pt = (role == ROLE_SAMPLER)
                       ? &pendingSampler
                       : &pendingDatalogger;

  pt->active = true;
  pt->role = role;
  pt->trigger_id = trigger_id;
  pt->sentTimeMs = millis();
  pt->retryCount = 0;

  //set all clients to ack received = false, AND maintain a list of the expected clients
  for (int i = 0; i < MAX_CLIENTS; i++) {
    pt->ackReceived[i] = false;

    if (i < clientCount && clients[i].role == role) {
      pt->expected[i] = true;
    } else {
      pt->expected[i] = false;
    }
  }
  Packet t;
  t.msg_type = MSG_TRIGGER;
  t.role = role;
  t.client_id = 0; // not needed from server
  t.trigger_id = trigger_id;
  t.secret_tag = SECRET_TAG;
  

  for (int i = 0; i < clientCount; i++) {
    if (clients[i].role != role) continue;

    // Ensure peer exists
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, clients[i].mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer); // OK if already exists

    esp_now_send(clients[i].mac, (uint8_t*)&t, sizeof(Packet));
  }
}
//check all clients and resend connection to ones that haven't ACKed
void resendPending(PendingTrigger* pt) {

  Packet t;
  t.msg_type = MSG_TRIGGER;
  t.role = pt->role;
  t.client_id = 0;
  t.trigger_id = pt->trigger_id;
  t.secret_tag = SECRET_TAG;

  for (int i = 0; i < clientCount; i++) {

    if (clients[i].role != pt->role) continue;

    // Only resend to clients that haven't ACKed
    if (pt->ackReceived[i]) continue;

    esp_now_send(clients[i].mac, (uint8_t*)&t, sizeof(Packet));
  }
}

void checkPending(PendingTrigger* pt) {

  if (!pt->active) return;

  bool allAcked = true;

  for (int i = 0; i < clientCount; i++) {
    //check expected and ack received - don't need to check role (include in expected)
    if (pt->expected[i] &&
    !pt->ackReceived[i]) {
      allAcked = false;
      break;
    }
  }

  if (allAcked) {
    Serial.println("all ACKs received");
    pt->active = false;
    return;
  }
  
  if (pt->retryCount < MAX_RETRIES &&
    millis() - pt->sentTimeMs > RETRY_INTERVALS_MS[pt->retryCount]) {

    pt->retryCount++;
    pt->sentTimeMs = millis();

    if (pt->retryCount >= MAX_RETRIES) {
      Serial.println("Trigger FAILED (missing ACKs)");
      pt->active = false;
    }

    Serial.print("Retry #");
    Serial.println(pt->retryCount);

    resendPending(pt);
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

    // Also shift ACK state for pending triggers
    pendingSampler.ackReceived[i] = pendingSampler.ackReceived[i + 1];
    pendingDatalogger.ackReceived[i] = pendingDatalogger.ackReceived[i + 1];
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
  pinMode(GPIO_DATALOGGER_TRIGGER, INPUT);
  pinMode(GPIO_SAMPLER_TRIGGER, INPUT);

  while (!initEspNowServer()) {
    delay(500);
  }
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
   
  delay(5);
}
