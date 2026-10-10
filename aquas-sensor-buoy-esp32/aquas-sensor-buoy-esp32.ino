#include <Ezo_i2c.h>
#include <Wire.h>
#include <Ezo_i2c_util.h>
#include <SD.h>
#include <SPI.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <DS3231.h>
#include <WiFi.h>
#include <sequencer3.h>
#include <sequencer4.h>
#include <driver/rtc_io.h>  // Add RTC GPIO support
#include <esp_now.h>

#define VALID_PH    (1 << 0)
#define VALID_TEMP  (1 << 1)
#define VALID_DO    (1 << 2)
#define VALID_EC    (1 << 3)
#define VALID_TURB  (1 << 4)

static int sampleStatus = 0;

enum MsgType : uint8_t {
  MSG_HELLO   = 1,
  MSG_TRIGGER = 2,
  MSG_ACK     = 3,
  MSG_STAT     = 4,
  MSG_DATA      = 5,
  MSG_NOOP      = 6   // server reply to HELLO when no trigger is pending
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






//secret tag for server AUTH
static const uint32_t SECRET_TAG = 0xA0A00001;

// Config
static const ClientRole MY_ROLE = ROLE_SAMPLER; // or ROLE_DATALOGGER
static const uint32_t CLIENT_ID = 0x12345678;   // set per device

// Server MAC address (fill in after reading Serial from server)
uint8_t SERVER_MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// Check-in timing: the buoy wakes every CHECKIN_INTERVAL_US, sends a HELLO and
// waits briefly for the server's reply (MSG_TRIGGER or MSG_NOOP).
// Worst-case trigger latency = CHECKIN_INTERVAL_US.
const uint64_t CHECKIN_INTERVAL_US = 60000000ULL;  // 60 s
const uint32_t CHECKIN_LISTEN_MS   = 50;           // wait for server reply
const uint32_t SEND_CB_TIMEOUT_MS  = 20;           // wait for esp_now send callback

// Survive deep sleep
RTC_DATA_ATTR uint32_t lastHandledTriggerId = 0;   // dedupe re-delivered triggers
RTC_DATA_ATTR uint32_t dataSeq = 0;                // DataPacket sequence number

uint32_t activeTriggerId = 0;   // trigger being serviced this wake (0 = none)

volatile bool haveTrigger = false;
volatile uint32_t lastTriggerId = 0;
volatile bool lastSendSuccess = false;
volatile bool sendDone = false;
volatile bool replyReceived = false;
volatile bool ackSent = false;

//confirm ACK receipt
void onSend(const wifi_tx_info_t *tx_info,
            esp_now_send_status_t status)
{
  lastSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  sendDone = true;
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(Packet)) return;
  Packet p{};
  memcpy(&p, data, sizeof(Packet));

  if (p.secret_tag != SECRET_TAG) return;

  if (p.msg_type == MSG_NOOP) {
    replyReceived = true;
    return;
  }

  if (p.msg_type != MSG_TRIGGER) return;
  if (p.role != MY_ROLE) return;

  // Ignore a trigger we've already serviced (server re-delivers if our ACK was lost),
  // but still ACK it below so the server stops re-sending.
  if (p.trigger_id != lastHandledTriggerId) {
    lastTriggerId = p.trigger_id;
    haveTrigger = true;
  }

  Packet ack{};
  ack.msg_type = MSG_ACK;
  ack.role = MY_ROLE;
  ack.client_id = CLIENT_ID;
  ack.trigger_id = p.trigger_id;
  ack.secret_tag = SECRET_TAG;

  lastSendSuccess = false;
  sendDone = false;
  ackSent = true;
  esp_now_send(SERVER_MAC, (uint8_t*)&ack, sizeof(Packet));
  replyReceived = true;   // set last so checkIn() sees ackSent/sendDone in order
}

// Wait for the send callback of the last esp_now_send; returns delivery status.
// Reset sendDone before calling esp_now_send.
bool waitSendDone() {
  uint32_t start = millis();
  while (!sendDone && millis() - start < SEND_CB_TIMEOUT_MS) delay(1);
  return sendDone && lastSendSuccess;
}

bool initEspNow() {
  WiFi.persistent(false);   // don't write WiFi config to flash on every wake
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
  sendDone = false;

  if (esp_now_send(SERVER_MAC, (uint8_t*)&hello, sizeof(Packet)) != ESP_OK) return false;

  return waitSendDone();
}

void radioOff() {
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
}

// Send HELLO and listen briefly for the server's reply.
// Returns true if a new (not yet handled) trigger arrived.
bool checkIn() {
  haveTrigger = false;
  replyReceived = false;
  ackSent = false;

  if (!sendHello()) {
    Serial.println("HELLO not delivered (server unreachable)");
    return false;
  }

  uint32_t start = millis();
  while (!replyReceived && millis() - start < CHECKIN_LISTEN_MS) delay(1);

  // Let the ACK sent from onRecv go out before the radio is turned off
  if (ackSent) waitSendDone();

  return haveTrigger;
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
  sendDone = false;

  if (esp_now_send(SERVER_MAC, (uint8_t*)&status, sizeof(Packet)) != ESP_OK) return false;

  return waitSendDone();
}
static bool lastDataSendOk = false;

bool handleTrigger(uint32_t triggerId){
  

  Serial.print("Reporting status for trigger: ");
  Serial.println(triggerId);
  sampleStatus = lastDataSendOk ? 1 : 0;   // 1 = data delivered, 0 = data send failed
  return sendSampleStatus(sampleStatus, triggerId);

}


/*
 * ESP32 Sensor Buoy Operation Pattern:
 * 
 * 1. CHECK-IN (every wake, every CHECKIN_INTERVAL_US):
 *    - Radio on, send HELLO, wait briefly for the server's reply
 *    - No trigger: radio off, straight back to deep sleep (no SD/sensors touched)
 *    - Trigger: radio off, continue to full setup
 *
 * 2. SETUP (only when triggered):
 *    - Initialize SD card
 *    - Initialize ADC for turbidity sensor
 *    - Calibrate ADC
 *    - Wake up interlink channels
 *    - Initialize the sequencer
 *
 * 3. SEQUENCER OPERATION:
 *    - step1: Wake sensors and send read commands
 *    - step2: Receive readings and send EC command
 *    - step3: Receive EC, read turbidity, send data + status (radio on briefly),
 *             write data, sleep sensors
 *    - sleepStep: Sleep interlink channels and go to deep sleep
 *
*/

// SD Card power control pin (HIGH = power on, LOW = power off)
const int sdCardPowerPin = 4;
// SD Card CS pin
const int sdCardCSPin = 5;

// Interlink isolated channel disable pin. HIGH = disable
// Note: Pins 25,26 are RTC GPIO pins that can maintain state during deep sleep
const int interlinkIsolatedDisablePin = 25;  // RTC_GPIO6 - maintains state during sleep
// Interlink non-isolated channel disable pin. LOW = disable
const int interlinkNonIsolatedDisablePin = 26;  // RTC_GPIO7 - maintains state during sleep

// Turbidity sensor pin (ADC1_CH6 on GPIO34)
const int turbidityPin = 34;

String filename = "/sensor.csv";

// RTC object
DS3231 rtc;

// ****************************************
// EZO interlink sensor configuration
Ezo_board DO = Ezo_board(97, "DO");     //dissolved oxygen
Ezo_board PH = Ezo_board(99, "PH");     //ph
Ezo_board EC = Ezo_board(100, "EC");    //electrical conductivity
Ezo_board RTD = Ezo_board(102, "RTD");  //temperature
// ****************************************

// ****************************************
// SENSOR MEMORY MANAGEMENT

// Single buffers to hold the current sensor readings
char ph_receive_buffer[32];
char rtd_receive_buffer[32];
char do_receive_buffer[32];
char ec_receive_buffer[32];

// Turbidity reading variable
float turbidityNTU = 0.0;

// ADC calibration
esp_adc_cal_characteristics_t adc_chars;

// Overall system error code (0 = success)
// Error Code System:
// 0 = Success (all sensors working)
// 1 = RTC error (time not available)
// 2 = pH sensor error
// 3 = RTD (temperature) sensor error
// 4 = DO (dissolved oxygen) sensor error
// 5 = EC (electrical conductivity) sensor error
// 6 = Turbidity sensor error
// 7 = Multiple errors present
int system_error_code = 0;

// ****************************************

// Diagnostic functions are now in a separate file: diagnostics.ino
// Include that file in your project when you need to run diagnostics

// Sequencer step functions
void step1();
void step2();
void step3();
void sleepStep();

Sequencer4 readSequence(&step1, 1000, &step2, 1000, &step3, 1000, &sleepStep, 1000);

// Function to read turbidity from the sensor, based on temperature compensation and conversion from voltage to NTU
float readTurbidity(float temperature) {
  // Read analog value from turbidity sensor
  int sensorValue = analogRead(turbidityPin);

  // Convert to voltage (ESP32 ADC is 12-bit, 0-3.3V)
  float voltage = sensorValue * (3.3 / 4095.0);

  // Convert voltage to NTU using quadratic formula for 3.3V operation
  // Based on DFRobot SEN0189 calibration adjusted for 3.3V: y = -1120.4x² + 5742.3x - 4352.9
  float ntu = -1120.4 * voltage * voltage + 5742.3 * voltage - 4352.9;

  // Apply temperature compensation: turbidity readings typically increase by ~2% per °C above 20°C
  float tempCompensation = 1.0 + 0.02 * (temperature - 20.0);
  ntu = ntu / tempCompensation;

  // Ensure NTU is not negative
  if (ntu < 0) {
    ntu = 0;
  }

  return ntu;
}

void powerOnSDCard() {
  Serial.println("Powering on SD card...");
  digitalWrite(sdCardPowerPin, HIGH);
  delay(100);  // Give SD card time to power up
}

void powerOffSDCard() {
  Serial.println("Powering off SD card...");
  digitalWrite(sdCardPowerPin, LOW);
  delay(50);   // Brief delay to ensure power off
}

void initSD() {
  // Power on SD card first
  powerOnSDCard();
  
  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(sdCardCSPin)) {
    Serial.println("SD card failed, or not present");
    Serial.println("Check wiring and SD card");
    // Don't hang (that would keep the buoy awake forever); data is still sent by radio
    return;
  }
  Serial.println("SD card initialized.");

  delay(100);

  // Create CSV file with headers if it doesn't exist
  if (!SD.exists(filename)) {
    Serial.println("File doesn't exist, creating new file...");
    File dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
      Serial.println("File opened successfully, writing headers...");
      dataFile.println("timestamp,ph,temperature,dissolved_oxygen,electrical_conductivity,turbidity_ntu,error_code");
      dataFile.close();
      Serial.println("Created new CSV file with headers");
    } else {
      Serial.println("Error creating CSV file");
    }
  } else {
    Serial.println("CSV file already exists");
  }
}

void setup() {
  // Initialize Serial first for debugging
  Serial.begin(115200);

  // Setup timer wakeup for the next check-in
  esp_sleep_enable_timer_wakeup(CHECKIN_INTERVAL_US);

  bool coldBoot = esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER;
  if (coldBoot) {
    Serial.println("ESP32 Sensor Buoy Starting...");
    // First power-up: make sure the sensors are off while we sleep between check-ins
    rtc_gpio_init((gpio_num_t)interlinkIsolatedDisablePin);
    rtc_gpio_init((gpio_num_t)interlinkNonIsolatedDisablePin);
    sleepInterlinkChannels();
    rtc_gpio_force_hold_en_all();
  }

  // ---------- Fast path: check in with the server before touching anything else ----------
  bool triggered = initEspNow() && checkIn();
  radioOff();

  if (!triggered) {
    Serial.printf("No trigger, awake %lu ms, sleeping\n", millis());
    Serial.flush();
    esp_deep_sleep_start();   // interlink pins are still force-held from the last sleep
  }

  activeTriggerId = lastTriggerId;
  lastHandledTriggerId = activeTriggerId;
  Serial.printf("Triggered: %lu - taking a reading\n", (unsigned long)activeTriggerId);

  // ---------- Full setup for a reading ----------
  // Set up pins
  pinMode(sdCardPowerPin, OUTPUT);
  
  // Initialize interlink control pins as RTC GPIO for persistent state during sleep
  rtc_gpio_init((gpio_num_t)interlinkIsolatedDisablePin);
  rtc_gpio_init((gpio_num_t)interlinkNonIsolatedDisablePin);
  
  Serial.printf("Interlink pins configured as RTC GPIO: %d, %d\n", interlinkIsolatedDisablePin, interlinkNonIsolatedDisablePin);

  // Initialize I2C with proper configuration
  Serial.println("Initializing I2C communication...");
  Wire.begin();

  // Configure I2C with proper timing and pull-up settings
  Wire.setTimeOut(5000);  // 5 second timeout
  Wire.setClock(100000);  // Set to 100kHz for better reliability

  Serial.printf("I2C initialized on SDA: %d, SCL: %d\n", SDA, SCL);
  Serial.println("I2C configured: 100kHz clock, 5s timeout");

  // Initialize SD card
  initSD();

  // Initialize ADC for turbidity sensor
  Serial.println("Initializing ADC for turbidity sensor...");
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // 0-3.3V range
  Serial.printf("ADC configured: 12-bit resolution, 0-3.3V range\n");

  // Calibrate ADC
  Serial.println("Calibrating ADC...");
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  Serial.println("ADC calibration complete");

  // Disable force hold from previous sleep cycle
  rtc_gpio_force_hold_dis_all();
  
  // Wake up interlink channels
  Serial.println("Waking up interlink channels...");
  wakeInterlinkChannels();

  Serial.println("System ready - data will be saved to " + filename);

  // Initialize the sequencer (like Arduino version)
  readSequence.reset();
  Serial.println("Sequencer initialized - ready to run");
}

void loop() {
  // Run the sequencer (like Arduino version)
  readSequence.run();

  // Check for serial commands (like Arduino version)
  checkSerialCommands();

  // Small delay to prevent overwhelming the system
  delay(100);
}

// Steps of sensor readings and sleeping
void step1() {
  // Wake sensors and delay for time until stable reading
  wakeSensors();

  // send a read command
  DO.send_read_cmd();
  PH.send_read_cmd();
  RTD.send_read_cmd();
}

void step2() {
  enum Ezo_board::errors myerr = PH.receive_cmd(ph_receive_buffer, 32);
  RTD.receive_cmd(rtd_receive_buffer, 32);
  DO.receive_cmd(do_receive_buffer, 32);

  if ((RTD.get_error() == Ezo_board::SUCCESS) && (RTD.get_last_received_reading() > -1000.0)) {
    EC.send_read_with_temp_comp(RTD.get_last_received_reading());
  } else {
    // Default case: EC with 25˚C default temperature
    EC.send_read_with_temp_comp(25.0);
  }
}

void step3() {
  EC.receive_cmd(ec_receive_buffer, 32);

  // Read turbidity with temperature compensation
  float temperature = RTD.get_last_received_reading();
  if (temperature <= -1000.0) {
    temperature = 25.0;  // Default temperature if RTD reading failed
  }
  turbidityNTU = readTurbidity(temperature);

  // Now we have all sensor readings - radio on only long enough to send data + status
  if (initEspNow()) {
    lastDataSendOk = sendData();
    if (!handleTrigger(activeTriggerId)) {
      Serial.println("status send FAILED");
    }
  } else {
    Serial.println("ESP-NOW init failed - data not sent");
    lastDataSendOk = false;
  }
  radioOff();

  writeDataToFile();

  // Sleep sensors
  sleepSensors();
}

 

 
 

// Parse an EZO response buffer ("7.012", "*ER", "", "1413,706,0.69,1.000" ...).
// Takes the first numeric field; returns false if there isn't one.
static bool parseReading(const char* buf, float& out) {
  if (buf == nullptr || buf[0] == '\0') return false;
  char* end;
  float v = strtof(buf, &end);
  if (end == buf) return false;   // no number at the start (e.g. "*ER")
  out = v;
  return true;
}
 
bool sendData() {
  DataPacket pkt{};
  pkt.msg_type   = MSG_DATA;
  pkt.role       = MY_ROLE;
  pkt.client_id  = CLIENT_ID;
  pkt.secret_tag = SECRET_TAG;
  pkt.seq        = dataSeq++;   // RTC memory, so it keeps counting across deep sleep
 
  // Timestamp from the same RTC used for the SD log
  RTCDateTime dt = rtc.getDateTime();
  pkt.year   = dt.year;
  pkt.month  = dt.month;
  pkt.day    = dt.day;
  pkt.hour   = dt.hour;
  pkt.minute = dt.minute;
  pkt.second = dt.second;
 
  // Sensor readings (parse into a local: references can't bind to packed fields)
  float v;
  if (parseReading(ph_receive_buffer,  v)) { pkt.ph = v;               pkt.valid_mask |= VALID_PH; }
  if (parseReading(rtd_receive_buffer, v)) { pkt.temperature_c = v;    pkt.valid_mask |= VALID_TEMP; }
  if (parseReading(do_receive_buffer,  v)) { pkt.dissolved_oxygen = v; pkt.valid_mask |= VALID_DO; }
  if (parseReading(ec_receive_buffer,  v)) { pkt.conductivity = v;     pkt.valid_mask |= VALID_EC; }
 
  pkt.turbidity_ntu = turbidityNTU;
  pkt.valid_mask   |= VALID_TURB;
 
  pkt.error_code = (int32_t)system_error_code;
 
  lastSendSuccess = false;
  sendDone = false;

  esp_err_t err = esp_now_send(SERVER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  if (err != ESP_OK) {
    Serial.printf("esp_now_send failed: %s\n", esp_err_to_name(err));
    return false;
  }

  bool ok = waitSendDone();
  Serial.printf("Data packet #%lu %s\n", (unsigned long)pkt.seq,
                ok ? "delivered" : "NOT acknowledged");
  return ok;
}


void writeDataToFile() {
  File dataFile = SD.open(filename, FILE_APPEND);

  if (dataFile) {
    // Get current timestamp from RTC
    RTCDateTime dt = rtc.getDateTime();

    // Format timestamp as YYYY-MM-DD HH:MM:SS
    char timestamp[64];
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d",
             dt.year, dt.month, dt.day,
             dt.hour, dt.minute, dt.second);

    // Write timestamp
    dataFile.print(timestamp);
    dataFile.print(",");

    // Write sensor data
    dataFile.print(ph_receive_buffer);
    dataFile.print(",");
    dataFile.print(rtd_receive_buffer);
    dataFile.print(",");
    dataFile.print(do_receive_buffer);
    dataFile.print(",");
    dataFile.print(ec_receive_buffer);
    dataFile.print(",");
    dataFile.print(turbidityNTU, 2);  // Print with 2 decimal places
    dataFile.print(",");
    dataFile.print(system_error_code);  // Print error code
    dataFile.println();

    dataFile.close();
    Serial.println("Data saved to SD card");

    // Print to Serial for debugging
    Serial.print("Timestamp: ");
    Serial.println(timestamp);
    Serial.print("pH: ");
    Serial.println(ph_receive_buffer);
    Serial.print("Temperature: ");
    Serial.println(rtd_receive_buffer);
    Serial.print("DO: ");
    Serial.println(do_receive_buffer);
    Serial.print("EC: ");
    Serial.println(ec_receive_buffer);
    Serial.print("Turbidity: ");
    Serial.println(turbidityNTU, 2);
    Serial.print("Error Code: ");
    Serial.println(system_error_code);

  } else {
    Serial.println("Error opening " + filename + " for writing");
    // Fallback to Serial output if SD card fails
    Serial.println("Fallback - printing to Serial:");
    Serial.print(ph_receive_buffer);
    Serial.print(";");
    Serial.print(rtd_receive_buffer);
    Serial.print(";");
    Serial.print(do_receive_buffer);
    Serial.print(";");
    Serial.print(ec_receive_buffer);
    Serial.print(";");
    Serial.print(turbidityNTU, 2);
    Serial.print(";");
    Serial.print(system_error_code);
    Serial.println();
  }
}

// Sleep all sensors indefinitely until any other command is issued.
void sleepSensors() {
  Serial.println("Sending sleep commands to all sensors...");
  
  // Send sleep command to each sensor with delays to avoid overwhelming I2C bus
  DO.send_cmd("Sleep");
  delay(500);  // Small delay between commands
  
  PH.send_cmd("Sleep");
  delay(500);
  
  EC.send_cmd("Sleep");
  delay(500);
  
  RTD.send_cmd("Sleep");
  delay(500);
  
  // Give sensors time to process sleep commands
  Serial.println("Waiting for sensors to enter sleep mode...");
  delay(1000);  // 1 second delay for sensors to process
  
  Serial.println("ALL sensors should now be sleeping...");
  
  // Verify sensors are sleeping by checking if they respond (they shouldn't)
  Serial.println("Verifying sensors are in sleep mode...");
  delay(500);  // Additional delay before verification
  
  // Try to send a status command to RTD - if it's sleeping, this should fail
  // This is a simple verification that the sensor is not responding
  Wire.beginTransmission(102);  // RTD address
  if (Wire.endTransmission() != 0) {
    Serial.println("RTD sensor appears to be sleeping (no response)");
  } else {
    Serial.println("WARNING: RTD sensor may still be awake!");
  }
}

// Send an arbitrary command to wake all sensors + delay
void wakeSensors() {
  char* wakeCommand = "Status";
  DO.send_cmd(wakeCommand);
  PH.send_cmd(wakeCommand);
  EC.send_cmd(wakeCommand);
  RTD.send_cmd(wakeCommand);
  Serial.println("All sensors waking, delaying for 20s...");
  delay(20000);
}

void sleepInterlinkChannels() {
  // Use RTC GPIO to set values that persist during deep sleep
  rtc_gpio_set_direction((gpio_num_t)interlinkNonIsolatedDisablePin, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_direction((gpio_num_t)interlinkIsolatedDisablePin, RTC_GPIO_MODE_OUTPUT_ONLY);
  
  rtc_gpio_set_level((gpio_num_t)interlinkNonIsolatedDisablePin, 0);  // LOW = disable
  rtc_gpio_set_level((gpio_num_t)interlinkIsolatedDisablePin, 1);     // HIGH = disable
  
  // Enable hold to latch these values before sleep
  rtc_gpio_hold_en((gpio_num_t)interlinkNonIsolatedDisablePin);
  rtc_gpio_hold_en((gpio_num_t)interlinkIsolatedDisablePin);
  
  Serial.println("Sleeping interlink channels using RTC GPIO...");
}

void wakeInterlinkChannels() {
  // Disable hold first
  rtc_gpio_hold_dis((gpio_num_t)interlinkNonIsolatedDisablePin);
  rtc_gpio_hold_dis((gpio_num_t)interlinkIsolatedDisablePin);
  
  // Use RTC GPIO to set values
  rtc_gpio_set_direction((gpio_num_t)interlinkNonIsolatedDisablePin, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_direction((gpio_num_t)interlinkIsolatedDisablePin, RTC_GPIO_MODE_OUTPUT_ONLY);
  
  rtc_gpio_set_level((gpio_num_t)interlinkNonIsolatedDisablePin, 1);  // HIGH = enable
  rtc_gpio_set_level((gpio_num_t)interlinkIsolatedDisablePin, 0);     // LOW = enable
  
  Serial.println("Waking interlink channels using RTC GPIO...");
}

void sleepStep() {
  Serial.println("Sleep step completed - going to deep sleep...");

  // Sensors were already put to sleep at the end of step3

  // Sleep interlink channels using RTC GPIO (values will persist during sleep)
  sleepInterlinkChannels();

  // Power off SD card to save power during sleep
  Serial.println("Powering off SD card...");
  powerOffSDCard();

  // Additional safety: disable I2C to prevent communication attempts during sleep
  Serial.println("Disabling I2C communication...");
  Wire.end();
  
  // Small delay to ensure I2C is fully disabled
  delay(100);

  // Force hold all RTC GPIO pins to ensure their values persist during deep sleep
  Serial.println("Enabling RTC GPIO hold for deep sleep...");
  rtc_gpio_force_hold_en_all();

  Serial.flush();  // Ensure all serial data is sent

  Serial.printf("Going to deep sleep until next check-in (%llu us)\n", CHECKIN_INTERVAL_US);

  // Go to deep sleep – timer based wakeup configured in setup()
  esp_deep_sleep_start();

  // This line should never be reached
  Serial.println("This should never be printed");
}

// Function to check for serial commands
void checkSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "help") {
      Serial.println("Available commands:");
      Serial.println("  help - Show this help");
      Serial.println("  status - Show system status");
    } else if (command == "status") {
      Serial.println("=== SYSTEM STATUS ===");
      Serial.printf("System error code: %d\n", system_error_code);
      Serial.printf("CSV file: %s\n", filename.c_str());

      // Check file size
      File file = SD.open(filename, "r");
      if (file) {
        Serial.printf("File size: %d bytes\n", file.size());
        file.close();
      } else {
        Serial.println("File not accessible");
      }

      Serial.println("=== END STATUS ===");
    } else if (command.length() > 0) {
      Serial.printf("Unknown command: '%s'\n", command.c_str());
      Serial.println("Type 'help' for available commands");
    }
  }
}