#include <WiFi.h>
#include <esp_now.h>

enum MsgType : uint8_t {
  MSG_HELLO   = 1,
  MSG_TRIGGER = 2,
  MSG_ACK     = 3,
  MSG_STAT     = 4,
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






//secret tag for server AUTH
static const uint32_t SECRET_TAG = 0xA0A00001;

// Config
static const ClientRole MY_ROLE = ROLE_SAMPLER; // or ROLE_DATALOGGER
static const uint32_t CLIENT_ID = 0x12345678;   // set per device

// Server MAC address (fill in after reading Serial from server)
uint8_t SERVER_MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

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
  Packet p{};
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
  Packet hello{};
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




//** ALL SAMPLING-SPECIFIC LOGIC **//

// sampler pin definitions


static const int CONTAINER_PINS[3] = {16, 17, 18};
static const int OUTFLOW_SOLENOID_PIN = 15;
static const int SENSOR_PINS[3] = {4, 5, 6};
static const int PUMP_PIN = 21;
static const int SAMPLE_TRIGGER_PIN = 7;


// Container status struct
struct ContainerStatus {
  int container_pin;
  int sensor_pin;
  bool is_filled;
};

// Tracks which container to fill
ContainerStatus containers[3];
int currentSampleContainer = 0; 

//tracks the status of the most recent sample attempt
static int sampleStatus = 0;

// Turn pump on/off
void setPump(bool on) {
 
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);

}


// Purge system: flush lines for specified duration (ms)
void purge(unsigned long duration) {
  
  // Close all sample solenoids
  for (int i = 0; i < 3; i++) {
    digitalWrite(containers[i].container_pin, LOW);
  }

  Serial.println("Purging begins...");

  // Open outflow solenoid
  digitalWrite(OUTFLOW_SOLENOID_PIN, HIGH);
  setPump(true);

  //delay(duration);

  // Stop pump and close outflow
  setPump(false);
  digitalWrite(OUTFLOW_SOLENOID_PIN, LOW);
 
  Serial.println("Purging finished.");
}


// Force reset all container status
void forceReset() {
  
  for (int i = 0; i < 3; i++) {
    containers[i].is_filled = false;
  }
  currentSampleContainer = 0;
  Serial.println("Force reset: All container status reset to false");

}


// Sample into the current container (guardrails included)
void sample() {
 
  if (currentSampleContainer >= 3) {
    Serial.println("All containers filled.");
    sampleStatus = 1;
    return;
  }

  ContainerStatus &csc = containers[currentSampleContainer];



  // Check if already filled
  if (digitalRead(csc.sensor_pin) == LOW) {
    Serial.print("Container "); Serial.print(currentSampleContainer + 1);
    Serial.println(" is already filled.");
    csc.is_filled = true;
    currentSampleContainer++;
    sampleStatus = 2;
    return;
  }

  // Clear to sample: purge first to clean pipeline.
  purge(10000);


  // Begin sampling
  digitalWrite(csc.container_pin, HIGH);  // Open solenoid
  setPump(true);

  Serial.print("Sampling container ");
  Serial.println(currentSampleContainer + 1);
  unsigned long startTime = millis();
  unsigned long timeout = 10000;  // 10 seconds safety timeout

  // Wait until the container is filled or timeout
  while (digitalRead(csc.sensor_pin) == HIGH) {
    if (millis() - startTime > timeout) {
      Serial.println("Sampling timeout: sensor did not trigger.");
      break;
    }
  }

  // Stop pump and close solenoid
  setPump(false);
  digitalWrite(csc.container_pin, LOW);
  Serial.print("Sampling for container ");
  Serial.print(currentSampleContainer + 1);
  Serial.println(" COMPLETED.");

  // Mark container as filled and move to next
  if(digitalRead(csc.sensor_pin) == LOW){ // LOW = water detected, HIGH = no water
    csc.is_filled = true;
  }  
  currentSampleContainer++;

  Serial.print("Sample collected in container ");
  Serial.println(currentSampleContainer);
  sampleStatus = 3;

}

bool sendSampleStatus(uint32_t sampleStatus, uint32_t triggerId) {
  
  Packet status{};
  status.msg_type = MSG_STAT;
  status.role = MY_ROLE;
  status.client_id = CLIENT_ID;
  status.trigger_id = triggerId;
  status.secret_tag = SECRET_TAG;
  status.status = sampleStatus;

  lastSendSuccess = false;

  esp_now_send(SERVER_MAC, (uint8_t*)&status, sizeof(Packet));

  delay(5); // allow callback to fire
 
  
  return lastSendSuccess;
 

 
}

bool handleTrigger(uint32_t triggerId){
  

  Serial.print("Running sample for trigger: ");
  Serial.println(triggerId);
  sample();
  return sendSampleStatus(sampleStatus, triggerId);

}



void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("sampler");
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
//set pin modes

pinMode(PUMP_PIN, OUTPUT);
pinMode(OUTFLOW_SOLENOID_PIN, OUTPUT);

//initialize each container struct

 for (int i = 0; i < 3; i++) {
  
  containers[i].container_pin = CONTAINER_PINS[i];
  containers[i].sensor_pin = SENSOR_PINS[i];
  containers[i].is_filled = false;
  digitalWrite(containers[i].container_pin, LOW);
  pinMode(containers[i].container_pin, OUTPUT);
  pinMode(containers[i].sensor_pin, INPUT_PULLUP);
  }
  
  
  //initialize pin states
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(OUTFLOW_SOLENOID_PIN, LOW);


  
}

void loop() {
  static uint32_t lastHello = 0;
  static uint32_t helloInterval = 5000;

  if (millis() - lastHello > helloInterval) {
    
    if (!sendHello()){
      Serial.println("HELLO Send Failed");
    }
    else{
      Serial.println("HELLO Send Succeeded");
      
    }
    lastHello = millis();
    

    // Add ±10% jitter
    uint32_t jitter = 500;
    helloInterval = 5000 + random(-jitter, jitter);
  }

  if (haveTrigger) {
    haveTrigger = false;

    Serial.print("Triggered: ");
    Serial.println(lastTriggerId);
    if (!handleTrigger(lastTriggerId)){
      Serial.println("sample status send FAILED");
    }
    Serial.print("sample status: ");
    Serial.println(sampleStatus);
    
    
  }
}
