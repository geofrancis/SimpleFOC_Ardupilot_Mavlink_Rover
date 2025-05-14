
#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_servo_output_raw.h"
#include <SimpleFOC.h>


int DZ = 20; // dead

int MAXRPM = 100;

//Target variable
float target_velocity = 0;
int ESC = 1;  //board number
int Vlimit = 1;


BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);



//Serial command setting
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}


unsigned long previousMillis = 0;  // will store last time LED was updated
const long telem = 2000;

int leftoutput = 0;
int rightoutput = 0;

int DI1O = 0;
int FCHB = 0;
int FCOK = 0;

int BASEMODE = 0;
int armed;
int active = 0;






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
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);  //1 = High, 0 = Low





  xTaskCreatePinnedToCore(
    FOC,        // Function name of the task
    "NMEANAV",  // Name of the task (e.g. for debugging)
    50000,      // Stack size (bytes)
    NULL,       // Parameter to pass
    1,          // Task priority
    NULL,       // Task handle
    0           // run on Core 0
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
  //motorL.monitor();
 // motorR.monitor();
int Vlimit = 3;

  if (FCOK == 0) {
  //  motor.move(0);
   // motor1.move(0);
  }


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= telem) {
    previousMillis = currentMillis;
    MAVLINK_HB1();
    //    MAVLINK_ESC_1();

    if (DI1O == 1) { Mavlink_Telemetry(); }
    if (DI1O == 2) { FCHBC(); }
    //if (DI1O == 3) { FOC_telemetry(); }
    //if (DI1O == 4) { sleepcheck(); }
    DI1O++;
    if (DI1O > 4) { DI1O = 1; }
    Serial.print("                                   DI1O  ");
    Serial.println(DI1O);
  }
}

void FOC(void* parameter) {
  while (1) {
    motor.move(target_velocity);
    motor1.move(target_velocity);

    //User newsletter
    command.run();
  }
}




void sleepcheck() {

  if (ESC == 1) {
    if (FCOK == 0) { esp_deep_sleep_start(); }
    if (armed == 0) { esp_deep_sleep_start(); }
    if (active == 0) { esp_deep_sleep_start(); }
  }


  if (ESC == 2) {
    if (FCOK == 0) { esp_deep_sleep_start(); }
    if (armed == 0) { esp_deep_sleep_start(); }
    //if (active == 0) { esp_deep_sleep_start(); }
  }

  if (ESC == 3) {
    if (FCOK == 0) { esp_deep_sleep_start(); }
    if (armed == 0) { esp_deep_sleep_start(); }
    if (active == 0) { esp_deep_sleep_start(); }
  }
}
