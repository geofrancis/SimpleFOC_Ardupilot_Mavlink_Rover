
#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_servo_output_raw.h"
#include <SimpleFOC.h>






HallSensor sensorL = HallSensor(18, 19, 15, 1);  // U V W Pole Pairs
void doAL() {
  sensorL.handleA();
}
void doBL() {
  sensorL.handleB();
}
void doCL() {
  sensorL.handleC();
}

HallSensor sensorR = HallSensor(5, 23, 13, 1);  // U V W Pole Pairs
void doAR() {
  sensorR.handleA();
}
void doBR() {
  sensorR.handleB();
}
void doCR() {
  sensorR.handleC();
}

HallSensor sensor1 = HallSensor(5, 23, 13, 1);  // U V W Pole Pairs
void doA1() {
  sensor1.handleA();
}
void doB1() {
  sensor1.handleB();
}
void doC1() {
  sensor1.handleC();
}

//Motor parameters: Set the number of pole pairs according to the motor
BLDCMotor motorL = BLDCMotor(1);
BLDCDriver3PWM driverL = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motorR = BLDCMotor(1);
BLDCDriver3PWM driverR = BLDCDriver3PWM(26, 27, 14, 21);


unsigned long previousMillis = 0;  // will store last time LED was updated
const long telem = 1000;

int leftoutput = 0;
int rightoutput = 0;

int DI1O = 0;
int FCHB = 0;
int FCOK = 0;

int BASEMODE = 0;
int armed;
int active = 0;


int MAXRPM = 100;




float targetL = 0;
float velocityL = 0;
float voltageqL = 0;
float currentqL = 0;

float targetR = 0;
float velocityR = 0;
float voltageqR = 0;
float currentqR = 0;







void setup() {
  FOC_SETUP();

  xTaskCreatePinnedToCore(
    FOC,        // Function name of the task
    "NMEANAV",  // Name of the task (e.g. for debugging)
    50000,      // Stack size (bytes)
    NULL,       // Parameter to pass
    1,          // Task priority
    NULL,       // Task handle
    1           // run on Core 1
  );

  uint8_t system_id = 1;
  uint8_t component_id = 158;
  uint8_t severity = 1;
  uint16_t id = 0;
  uint8_t chunk_seq = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(system_id, component_id, &msg, 2, "HOVERBOARD STARTUP", id, chunk_seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);

  request_Mavlink();
}


void loop() {
  MavLink_RC();
  motorL.monitor();
  motorR.monitor();


  if (FCOK == 0) {
    motorL.move(0);
    motorR.move(0);
  }


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= telem) {
    previousMillis = currentMillis;
    MAVLINK_HB1();
    //    MAVLINK_ESC_1();

    if (DI1O == 1) { Mavlink_Telemetry(); }
    if (DI1O == 3) { FCHBC(); }
    if (DI1O == 4) { FOC_telemetry(); }
    DI1O++;
    if (DI1O > 4) { DI1O = 1; }
    Serial.print("                                   DI1O  ");
    Serial.println(DI1O);
  } 
}

void FOC(void* parameter) {
  while (1) {
    motorL.loopFOC();
    motorR.loopFOC();
  }
}
