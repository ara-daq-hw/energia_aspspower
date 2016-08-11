#include <ArduinoJson.h>
#include <Wire.h>
#include <MspFlash.h>


#define CURRENT P4_4
#define VIN_MON P4_5
#define SERIAL_SELECT P2_1

const unsigned char __attribute__((section(".infob"))) default_disable[4] = { 0, 0, 0, 0 };

const int __attribute__((section("infod"))) calib[32] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                                          0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
char __attribute__((section(".noinit"))) cmd_buffer[128];
unsigned char cmd_buffer_ptr = 0;


static const uint8_t enables[4] = { P4_0, P4_1, P4_2, P4_7 };
static const int senses[4] = { P2_4, P2_2, P3_7, P4_6 };

#define HOUSEKEEPING_PERIOD 1000
unsigned long housekeepingTime = 0;
typedef enum housekeepingState {
  output_ON = 0,
  output_V = 1,
  output_I = 2,
  output_T = 3
} housekeepingState_t;

housekeepingState_t housekeepingState = output_ON;

#define CURRENT_WAIT_PERIOD 100
#define CURRENT_PERIOD 5

unsigned long currentTime = 0;
unsigned char current_channel = 0;
unsigned char current_average_count = 0;

int current_measure[4] = { 0, 0, 0, 0};
unsigned int current_temp = 0;
unsigned int last_voltage = 0;

unsigned int readVin() {
  return analogRead(VIN_MON);
}

unsigned int readSense(unsigned int channel) {
  return analogRead(senses[channel]);
}

// Our analog inputs are
// 0: P2_4, A4, OA1I0
// 1: P2_2, A2, OA0I1
// 2: P3_7, A7, OA1IA
// 3: P4_6, A15,OA1IB
// VIN: P4_5, A14
// OA0 output: P4_4, A13
// OA1 input: P2_3, A3
// OA1 output: P4_3, A12
// We want to hook up our op-amps as a difference amplifier.
// OA0: IN+ = A14 (OA0IB), IN- = OA0 output (internal)
// OA1: IN+ = A4/2/7/15, IN- = A3 (OA1I1)
// Then the output is on A13 (P4_4)
// OA0 is a unity-gain buffer
// OA1 is a general purpose op-amp
// OA0CTL0 = (OAP_3) | (OAPM_1) | (OAADC_3);
// OA0CTL1 = (OAFC_2);
// OA1CTL0 = (OAN_1) | (channel << 4) | (OAPM_1) | (OAADC_3);
// OA1CTL1 = (OAFC_0);
//
void readCurrent() {
  if (current_average_count == 64) {
	  unsigned char tmp;
	  current_temp += analogRead(CURRENT);
          current_temp = current_temp >> 1;
	  current_measure[current_channel] = (last_voltage << 5) + calib[current_channel] - current_temp;
	  current_temp = 0;
	  current_average_count = 0;

	  current_channel++;
	  current_channel = current_channel % 4;
	  tmp = OA1CTL0;
	  tmp &= (~OAP_3);
	  tmp |= (current_channel << 4);
	  OA1CTL0 = tmp;

	  currentTime = millis() + CURRENT_WAIT_PERIOD;
	  return;
  } else {
          if (!current_average_count) last_voltage = readVin();
	  current_temp += analogRead(CURRENT);
	  current_average_count++;
	  currentTime = millis() + CURRENT_PERIOD;
  }
}

void setup()
{  
  unsigned char err;

  // Add something here to grab data from info flash to determine if
  // we let all of them turn on.
  for (unsigned int i=0;i<4;i++) {
    if (default_disable[i]) {
      digitalWrite(enables[i], 0);
      pinMode(enables[i], OUTPUT);
    } else {
      pinMode(enables[i], INPUT);
    }
  }
  // Switch P2_1 to go to 15V_SW1_SENSE
  pinMode(SERIAL_SELECT, OUTPUT);
  digitalWrite(SERIAL_SELECT, 1);
  // Set up the OA.

  // OA0 is just a unity-gain follower on VIN_MON.
  OA0CTL1 = (OAFC_2);
  // Slow slew, output to external pin A12.
  OA0CTL0 = (OAPM_1) | (OAADC0) | (OAADC1);
  // OA1 is a general-purpose op-amp.
  OA1CTL1 = (OAFC_0);
  // start with channel 0.
  OA1CTL0 = (OAN_1) | (OAP_0) | (OAPM_1) | (OAADC0) | (OAADC1);
  currentTime = millis();

  Serial.begin(9600);
  Serial.println("{\"log\":\"reboot\"}");
  Wire.begin();
  Wire.beginTransmission(0x4C);
  Wire.write(0x09);
  Wire.write(0x4);
  err = Wire.endTransmission();
  if (err) {
	  Serial.print("{\"dbg\":\"I2C err\"}");
  }
  housekeepingTime = millis() + HOUSEKEEPING_PERIOD;
}

char readTMP422(unsigned int ch) {
	unsigned int tmp;
	unsigned char err;

	Wire.beginTransmission(0x4C);
	Wire.write(ch);
	err = Wire.endTransmission();
	if (err) {
		Serial.print("{\"dbg\":\"I2C err\"}");
	}
	Wire.requestFrom(0x4C, 1);
	tmp = 0;
	if (Wire.available()) tmp = Wire.read();
	tmp -= 64;
	return tmp;
}

void doHousekeeping() {
  unsigned int i;
  unsigned int count;
  int tmp;
  
  switch (housekeepingState) {
    case output_ON:
      Serial.print("{\"on\":[");
      count = 0;
      for (i=0;i<4;i++) {
        if (digitalRead(enables[i])) {
          if (count) Serial.print(",");
          Serial.print(i);
          count++;
        }
      }
      Serial.println("]}");
      housekeepingState = output_V;
      break;
    case output_V:
      Serial.print("{\"v\":[");
      Serial.print(last_voltage);
      Serial.print(",");
      Serial.print(analogRead(128+11));
      Serial.println("]}");
      housekeepingState = output_I;
      break;
    case output_I:
      Serial.print("{\"i\":[");
      for (i=0;i<4;i++) {
        if (i) Serial.print(",");
        Serial.print(current_measure[i]);
      }
      Serial.println("]}");
      housekeepingState = output_T;
      break;
    case output_T:
      Serial.print("{\"t\":[");
      Serial.print(analogRead(128+10));
      for (i=0;i<3;i++) {
        Serial.print(",");
        tmp = readTMP422(i);
        Serial.print(tmp);
      }
      Serial.println("]}");
      housekeepingState = output_ON;
      break;
  }
  housekeepingTime = millis() + HOUSEKEEPING_PERIOD;
}

void loop()
{
  if ((long) (millis() - currentTime) > 0)
    readCurrent();

  if ((long) (millis() - housekeepingTime) > 0)
    doHousekeeping();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      cmd_buffer[cmd_buffer_ptr] = 0;
      parseJsonInput();
      cmd_buffer_ptr = 0;
    } else {
      cmd_buffer[cmd_buffer_ptr] = c;
      if (cmd_buffer_ptr < 127) cmd_buffer_ptr++;
    }
  }
}

void parseJsonInput() {
  StaticJsonBuffer<64> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(cmd_buffer);
  if (!root.success()) {
    Serial.println("{\"dbg\":\"invalid JSON\"}");
    return;
  }
  if (root.containsKey("set")) {
    JsonArray& setArray = root["set"];
    unsigned char setMask = setArray[0];
    unsigned char setVal = setArray[1];
    unsigned int i;
    for (i=0;i<4;i++) {
      if (setMask & (1<<i)) {
        if (setVal & (1<<i))
          pinMode(enables[i],INPUT);
        else {
          digitalWrite(enables[i], 0);
          pinMode(enables[i], OUTPUT);
        }
      }
    }
  }
  if (root.containsKey("disable")) {
    unsigned char tmp_default[4] = { 0, 0, 0, 0};
    JsonArray& disableArray = root["disable"];
    for (size_t i=0;i<disableArray.size();i++) {
      unsigned int ch;
      ch = disableArray[i];
      if (ch<4) {
        tmp_default[ch] = 1;
      }
    }
    Flash.erase(SEGMENT_B);
    Flash.write(SEGMENT_B, tmp_default, 4);
  }
  if (root.containsKey("calib")) {
    int tmp_calib[32];
    int val;
    unsigned char ch;
    unsigned int i;
    JsonArray& calibArray = root["calib"];
    for (i=0;i<32;i++) {
      tmp_calib[i] = calib[i];
    }
    ch = calibArray[0];
    val = calibArray[1];
    Serial.print("{\"log\":\"update cal");
    Serial.print(ch);
    Serial.print(" to ");
    Serial.print(val);
    Serial.println("\"}");
    tmp_calib[ch] = val;
    Flash.erase(SEGMENT_D);
    Flash.write(SEGMENT_D, (unsigned char *) tmp_calib, 64);
  }
}
