void FOC_SETUPL() {
  SimpleFOCDebug::enable(&Serial);

  // initialize encoder sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 1;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();

  Serial.println(F("Motor L ready."));
}



void FOC_SETUPR() {

  sensor1.init();
  sensor1.enableInterrupts(doA1, doB1, doC1);
  motor1.linkSensor(&sensor1);

  driver1.voltage_power_supply = 24;
  driver1.init();
  motor1.linkDriver(&driver1);

  motor1.voltage_sensor_align = 1;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.controller = MotionControlType::torque;
  motor1.useMonitoring(Serial);

  motor1.init();
  motor1.initFOC();

  Serial.println(F("Motor R ready."));
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

  targetR = motor1.target;
  velocityR = motor1.shaft_velocity;
  voltageqR = motor1.voltage.q;
  currentqR = motor1.current.q;

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
}
