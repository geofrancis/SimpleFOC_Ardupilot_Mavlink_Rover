
void request_Mavlink() {
  //Request Data from Pixhawk
  uint8_t _system_id = 255;       // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 158;    // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1;     // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0;  // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0xA;  //number of times per second to request the data in hex
  uint8_t _start_stop = 1;           //1 = start, 0 = stop
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  Serial1.write(buf, len);  //Write data to serial port
}




void MavLink_RC() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available()) {
    uint8_t c = Serial1.read();

    //Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {

            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            HBWATCH();

            if ((hb.type) == 10) {
              if ((hb.custom_mode) == 4) {
                Serial.println("HOLD MOTOR ON");
                active = 0;
              }
              if ((hb.custom_mode) != 4) {
                Serial.println("HOLD MOTOR OFF");
                active = 1;
              }
            }
            Serial.print("\nFlight Mode: ");
            Serial.println(hb.custom_mode);
            BASEMODE = (hb.base_mode);
            ;
            if (BASEMODE == 193) {
              armed = 1;
              Serial.println("------------------------------------------------------------------------------------ARMED");
            }

            if (BASEMODE == 65) {
              Serial.println("------------------------------------------------------------------------------------DISARMED");
              armed = 0;
            }

            //  Serial.print("Type: ");
            //  Serial.println(hb.type);
            //  Serial.print("Autopilot: ");
            //  Serial.println(hb.autopilot);
            Serial.print("Base Mode: ");
            Serial.println(hb.base_mode);
            //  Serial.print("System Status: ");
            // Serial.println(hb.system_status);
            //   Serial.print("Mavlink Version: ");
            //   Serial.println(hb.mavlink_version);
            //    Serial.println();
          }
          break;

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:  // #35
          {
            mavlink_servo_output_raw_t SERVOCHANNEL;
            mavlink_msg_servo_output_raw_decode(&msg, &SERVOCHANNEL);
            leftoutputraw = (SERVOCHANNEL.servo1_raw);
            rightoutputraw = (SERVOCHANNEL.servo2_raw);
            Serial.print(rightoutputraw);
            Serial.print(leftoutputraw);
          }
          break;
      }
    }
  }
}


void FCHBC() {
  Serial.print("FCHB ");
  // Serial.println(FCHB);
  if (FCHB == 0) {
    Serial.println("-------------------------------------------------------------NO FC HEARTBEAT");
    Serial.println(FCOK);
    Serial.println(FCHB);
    FCOK = 0;
  }
  if (FCHB > 1) {
    Serial.print("Rover ");
    Serial.println(FCOK);
    Serial.print(FCHB);
    Serial.println("Beats ");
    Serial.println("----------------------------------------------------------FC HEARTBEAT");
    FCHB = 0;
    FCOK = 1;
  }
}




void HBWATCH() {
  FCHB++;
  Serial.print("HB");
}




void MAVLINK_HB() {
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
  uint32_t custom_mode = 1;                  ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int type = MAV_TYPE_SERVO;
  // Pack the message
  // Serial.print("mavhb1");
  mavlink_msg_heartbeat_pack(1, ESC, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}



void Mavlink_Telemetry() {

  mavlink_message_t msg;
  uint32_t time_boot_ms = millis();
  const char* name;
  float value;

  if (ESC == 140) { name = "RPM_reqL1"; }
  if (ESC == 141) { name = "RPM_reqL2"; }
  if (ESC == 142) { name = "RPM_reqL3"; }
  value = (targetL);
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);


  if (ESC == 140) { name = "RPML1"; }
  if (ESC == 141) { name = "RPML2"; }
  if (ESC == 142) { name = "RPML3"; }
  value = velocityL;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);


  if (ESC == 140) { name = "VoltL1"; }
  if (ESC == 141) { name = "VoltL2"; }
  if (ESC == 142) { name = "VoltL3"; }
  value = voltageqL;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);

  if (ESC == 140) { name = "AMPL1"; }
  if (ESC == 141) { name = "AMPL2"; }
  if (ESC == 142) { name = "AMPL3"; }
  value = currentqL;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);



  if (ESC == 140) { name = "RPM_reqR1"; }
  if (ESC == 141) { name = "RPM_reqR2"; }
  if (ESC == 142) { name = "RPM_reqR3"; }
  value = (targetR);
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);


  if (ESC == 140) { name = "RPMR1"; }
  if (ESC == 141) { name = "RPMR2"; }
  if (ESC == 142) { name = "RPMR3"; }
  value = velocityR;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);


  if (ESC == 140) { name = "VoltR1"; }
  if (ESC == 141) { name = "VoltR2"; }
  if (ESC == 142) { name = "VoltR3"; }
  value = voltageqR;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);

  if (ESC == 140) { name = "AMPR1"; }
  if (ESC == 141) { name = "AMPR2"; }
  if (ESC == 142) { name = "AMPR3"; }
  value = currentqR;
  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
  //Serial.print("V1: ");
  //Serial.println(VOLT1);

  mavlink_msg_named_value_float_pack(1, ESC, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void STARTUPMSG(){

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(system_id, component_id, &msg, 2, "HOVERBOARD 1 STARTUP", id, chunk_seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}




void MAVLINK_ESC_1() {
  // Serial.print("ESC1");
  mavlink_message_t msg;
  uint64_t time_usec = 1;
  uint8_t index = 1; /*<  Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.*/

  int32_t rrpm[] = { 200, 400, 800, 300 };
  float Voltage[] = { 20, 40, 80, 30 };
  float current[] = { 20, 40, 80, 30 };

  mavlink_msg_esc_status_pack(1, 143, &msg, 0, time_usec, rrpm, Voltage, current);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}
