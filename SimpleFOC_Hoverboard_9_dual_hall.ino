
#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_servo_output_raw.h"
#include <SimpleFOC.h>


//    ONLY WORKS WITH 3.1.1 ESP32
int DZ = 20;  // dead
int MAXRPM = 200;
float target_velocity = 1;

int ESC = 140;  //board number
//int ESC = 141;  //board number
//int ESC = 142;  //board number


float Vlimit = 1.5;

HallSensor sensor = HallSensor(18, 15, 19, 15);

void doA() {
  sensor.handleA();
}
void doB() {
  sensor.handleB();
}
void doC() {
  sensor.handleC();
}

HallSensor sensor1 = HallSensor(5, 23, 13, 15);
void doA1() {
  sensor1.handleA();
}
void doB1() {
  sensor1.handleB();
}
void doC1() {
  sensor1.handleC();
}


BLDCMotor motor = BLDCMotor(15);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 15);

BLDCMotor motor1 = BLDCMotor(15);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 15);


int leftoutputraw = 1500;
int rightoutputraw = 1500;

unsigned long previousMillis = 0;
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


uint8_t system_id = 1;
uint8_t component_id = 158;
uint8_t severity = 1;
uint16_t id = 0;
uint8_t chunk_seq = 0;





void setup() {
  Serial.begin(115200);  //Main serial port for console output
  Serial1.begin(500000, SERIAL_8N1, 17, 16);
  //Serial2.begin(38400, SERIAL_8N1, 2, 4);  //GPS+AIS

  FOC_SETUPR();
  FOC_SETUPL();
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);  //1 = High, 0 = Low

  STARTUPMSG();
}


void loop() {
  //motor.monitor();
  //motor1.monitor();
  motor.loopFOC();
  motor1.loopFOC();
  // MavLink_RC();

  //FOC_Speed();

  motor.move(target_velocity);
  motor1.move(target_velocity);






  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= telem) {
    previousMillis = currentMillis;
    MAVLINK_HB();
    // MAVLINK_ESC_1();

    if (DI1O == 1) { Mavlink_Telemetry(); }
    if (DI1O == 2) { FCHBC(); }
    if (DI1O == 3) { FOC_telemetry(); }
    // if (DI1O == 4) { sleepcheck(); }
    DI1O++;
    if (DI1O > 4) { DI1O = 1; }
    // Serial.print("                               DI1O  ");
    // Serial.println(DI1O);
  }
}
