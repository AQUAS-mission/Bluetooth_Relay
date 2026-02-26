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

static const uint32_t SECRET_TAG = 0xA0A00001;

// Config
static const ClientRole MY_ROLE = ROLE_SAMPLER; // or ROLE_DATALOGGER
static const uint32_t CLIENT_ID = 0x12345678;   // set per device

// Server MAC address (fill in after reading Serial from server)
uint8_t SERVER_MAC[6] = {0xEC,0x64,0xC9,0x7B,0x8E,0x64};

volatile bool haveTrigger = false;
volatile uint32_t lastTriggerId = 0;
volatile bool lastSendSuccess = false;

//confirm ACK receipt
void onSend(const wifi_tx_info_t *tx_info,
            esp_now_send_status_t status)
{
  lastSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(Packet)) return;
  Packet p;
  memcpy(&p, data, sizeof(Packet));

  if (p.secret_tag != SECRET_TAG) return;
  if (p.msg_type != MSG_TRIGGER) return;
  if (p.role != MY_ROLE) return;

  // Record and ACK immediately
  lastTriggerId = p.trigger_id;
  haveTrigger = true;

  Packet ack;
  ack.msg_type = MSG_ACK;
  ack.role = MY_ROLE;
  ack.client_id = CLIENT_ID;
  ack.trigger_id = lastTriggerId;
  ack.secret_tag = SECRET_TAG;

  esp_now_send(SERVER_MAC, (uint8_t*)&ack, sizeof(Packet));
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return false;

  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, SERVER_MAC, 6);
  peer.channel = 0;    // 0 = current channel
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  return true;
}

bool sendHello() {
  Packet hello;
  hello.msg_type = MSG_HELLO;
  hello.role = MY_ROLE;
  hello.client_id = CLIENT_ID;
  hello.trigger_id = 0;
  hello.secret_tag = SECRET_TAG;

  lastSendSuccess = false;

  esp_now_send(SERVER_MAC, (uint8_t*)&hello, sizeof(Packet));

  delay(10); // allow callback to fire

  return lastSendSuccess;

 
}

void setup() {
  Serial.begin(115200);
  Serial.println("client");
  randomSeed(esp_random());
  // Retry with backoff
  uint32_t backoffMs = 250;
  while (!initEspNow()) {

  uint32_t jitter = backoffMs * 0.2;
  uint32_t delayMs = random(backoffMs - jitter, backoffMs + jitter);

  delay(delayMs);

  backoffMs = min(backoffMs * 2, (uint32_t)10000);
}

  sendHello();
}

void loop() {
  static uint32_t lastHello = 0;
  static uint32_t helloInterval = 5000;

  if (millis() - lastHello > helloInterval) {
  
  if (!sendHello()) {
      Serial.println("HELLO send failed");
    }
    else{
      lastHello = millis();
    }

  // Add ±10% jitter
  uint32_t jitter = 500;
  helloInterval = 5000 + random(-jitter, jitter);


  }
  

  if (haveTrigger) {
    haveTrigger = false;

    // Placeholder action
    Serial.print("Triggered: ");
    Serial.println(lastTriggerId);

    // Do the actual sampler/datalogger start action here.
  }
}
