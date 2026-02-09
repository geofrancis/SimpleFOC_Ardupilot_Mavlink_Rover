void FOC_SETUPL() {
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 24;
  motor.voltage_sensor_align = 1;
  motor.velocity_index_search = 3;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  //motor.controller = MotionControlType::velocity;

  motor.PID_velocity.P = 0.06;
  motor.PID_velocity.I = 0.3;
  motor.PID_velocity.D = 0;
  motor.P_angle.P = 20;
  motor.voltage_limit = 1.5;
  motor.PID_velocity.output_ramp = 100;
  motor.LPF_velocity.Tf = 0.01f;
  motor.velocity_limit = 30;

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();
}




void FOC_SETUPR() {
  sensor1.init();
  sensor1.enableInterrupts(doA1, doB1, doC1);
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 24;
  motor1.voltage_sensor_align = 1;
  motor.velocity_index_search = 3;
  //motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //motor1.controller = MotionControlType::torque;

  motor1.controller = MotionControlType::velocity;
  motor1.PID_velocity.P = 0.03;
  motor1.PID_velocity.I = 0.3;
  motor1.PID_velocity.D = 0;
  motor1.P_angle.P = 20;
  motor1.voltage_limit = 1.5;
  motor1.PID_velocity.output_ramp = 100;
  motor1.LPF_velocity.Tf = 0.01f;
  motor1.velocity_limit = 30;

  motor1.useMonitoring(Serial);
  motor1.init();
  motor1.initFOC();
}



void FOC_Speed() {

  if (leftoutputraw > (1500 + DZ)) { leftoutput = map(leftoutputraw, (1500 + DZ), 2000, 0, MAXRPM); }
  if (leftoutputraw < (1500 + DZ) && leftoutputraw > (1500 - DZ)) { leftoutput = 0; }
  if (leftoutputraw < (1500 - DZ)) { leftoutput = map(leftoutputraw, (1500 - DZ), 1000, 0, -MAXRPM); }

  if (rightoutputraw > (1500 + DZ)) { rightoutput = map(rightoutputraw, (1500 + DZ), 2000, 0, MAXRPM); }
  if (rightoutputraw < (1500 + DZ) && rightoutputraw > (1500 - DZ)) { rightoutput = 0; }
  if (rightoutputraw < (1500 - DZ)) { rightoutput = map(rightoutputraw, (1500 - DZ), 1000, 0, -MAXRPM); }

  if (FCOK == 1) {
    motor.move(rightoutput);
    motor1.move(leftoutput);
  }

  if (FCOK == 0) {
    motor.move(0);
    motor1.move(0);
  }
}




void FOC_telemetry() {

  targetL = motor.target;
  velocityL = motor.shaft_velocity;
  voltageqL = motor.voltage.q;
  currentqL = motor.current.q;
  PhaseCurrent_s currentsL = current_senseL.getPhaseCurrents();
  lca = (currentsL.a * 1000);
  lcb = (currentsL.b * 1000);
  lcc = (currentsL.c * 1000);
  lc = current_senseL.getDCCurrent();



  targetR = motor1.target;
  velocityR = motor1.shaft_velocity;
  voltageqR = motor1.voltage.q;
  currentqR = motor1.current.q;
  PhaseCurrent_s currentsR = current_senseR.getPhaseCurrents();
  rca = (currentsR.a * 1000);
  rcb = (currentsR.b * 1000);
  rcc = (currentsR.c * 1000);
  rc = current_senseR.getDCCurrent();

}








void FOC_telemetry_Print() {

  Serial.println("");
  Serial.print("targetL ");
  Serial.println(targetL);
  Serial.print("velocityL ");
  Serial.println(velocityL);
  Serial.print("voltageqL ");
  Serial.println(voltageqL);
  Serial.print("currentqL ");
  Serial.println(currentqL);
  Serial.print("target_velocity ");
  Serial.println(target_velocity);



  Serial.print(lca);  // milli Amps
  Serial.print("\t");
  Serial.print(lcb);  // milli Amps
  Serial.print("\t");
  Serial.print(lcc);  // milli Amps
  Serial.print("\t");
  Serial.println(lc);  // milli Amps
  Serial.println();



  Serial.println("");
  Serial.print("target ");
  Serial.println(targetR);
  Serial.print("velocityR ");
  Serial.println(velocityR);
  Serial.print("voltageq ");
  Serial.println(voltageqR);
  Serial.print("currentqR ");
  Serial.println(currentqR);




  Serial.print(rca);  // milli Amps
  Serial.print("\t");
  Serial.print(rcb);  // milli Amps
  Serial.print("\t");
  Serial.print(rcc);  // milli Amps
  Serial.print("\t");
  Serial.println(rc);  // milli Amps





}
