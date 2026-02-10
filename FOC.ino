void FOC_SETUPL() {

  sensorL.init();

  driverL.voltage_power_supply = 24;

  sensorL.enableInterrupts(doAL, doBL, doCL);
  motorL.linkSensor(&sensorL);

  motorL.voltage_sensor_align = 1;
  motorL.velocity_index_search = 3;

  motorL.controller = MotionControlType::velocity;


  //motorL.controller = MotionControlType::torque;
  //motorL.torque_controller = TorqueControlType::foc_current;
  //motorL.foc_modulation = FOCModulationType::SpaceVectorPWM;

  //motorL.controller = MotionControlType::velocity_openloop;


  motorL.PID_velocity.P = 0.06;
  motorL.PID_velocity.I = 0.3;
  motorL.PID_velocity.D = 0;
  motorL.P_angle.P = 20;
  motorL.voltage_limit = 1.5;
  motorL.PID_velocity.output_ramp = 100;
  motorL.LPF_velocity.Tf = 0.01f;
  motorL.velocity_limit = 30;


  motorL.current_limit = 2;  //根据情况修改电压电流限制，也可注释取消此处代码


  current_senseL.init();
  current_senseL.gain_b *= -1;
  current_senseL.gain_a *= -1;
  //  current_senseL.skip_align = true;
  motorL.linkCurrentSense(&current_senseL);

  motorL.PID_current_q.P = 2;
  motorL.PID_current_q.I = 800;
  motorL.PID_current_d.P = 2;
  motorL.PID_current_d.I = 800;
  motorL.LPF_current_q.Tf = 0.002;  // 1ms default
  motorL.LPF_current_d.Tf = 0.002;  // 1ms default

  motorL.PID_velocity.P = 0.1;
  motorL.PID_velocity.I = 1;
  motorL.PID_velocity.D = 0;

  motorL.velocity_limit = 40;
  motorL.useMonitoring(Serial);
  motorL.init();
  motorL.initFOC();
}










void FOC_SETUPR() {
  sensorR.init();
  sensorR.enableInterrupts(doAR, doBR, doCR);
  motorR.linkSensor(&sensorR);
  driverR.voltage_power_supply = 24;
  motorR.voltage_sensor_align = 1;
  motorL.velocity_index_search = 3;


  motorR.controller = MotionControlType::velocity;

  //motorR.controller = MotionControlType::torque;
  //motorR.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //motorR.torque_controller = TorqueControlType::foc_current;
  //motorR.controller = MotionControlType::velocity_openloop;

  motorR.PID_velocity.P = 0.03;
  motorR.PID_velocity.I = 0.3;
  motorR.PID_velocity.D = 0;
  motorR.P_angle.P = 20;
  motorR.voltage_limit = 1.5;
  motorR.PID_velocity.output_ramp = 100;
  motorR.LPF_velocity.Tf = 0.01f;
  motorR.velocity_limit = 30;
  motorR.current_limit = 2;

  current_senseR.init();
  current_senseR.gain_b *= -1;
  current_senseR.gain_a *= -1;
  //  current_senseR.skip_align = true;
  motorR.linkCurrentSense(&current_senseR);
  motorR.torque_controller = TorqueControlType::foc_current;
  motorR.controller = MotionControlType::torque;

  motorR.PID_current_q.P = 2;
  motorR.PID_current_q.I = 800;
  motorR.PID_current_d.P = 2;
  motorR.PID_current_d.I = 800;
  motorR.LPF_current_q.Tf = 0.002;  // 1ms default
  motorR.LPF_current_d.Tf = 0.002;  // 1ms default

  motorR.PID_velocity.P = 0.1;
  motorR.PID_velocity.I = 1;
  motorR.PID_velocity.D = 0;

  motorR.velocity_limit = 40;

  motorR.useMonitoring(Serial);
  motorR.init();
  motorR.initFOC();
}



void FOC_Speed() {

  if (leftoutputraw > (1500 + DZ)) { leftoutput = map(leftoutputraw, (1500 + DZ), 2000, 0, MAXRPM); }
  if (leftoutputraw < (1500 + DZ) && leftoutputraw > (1500 - DZ)) { leftoutput = 0; }
  if (leftoutputraw < (1500 - DZ)) { leftoutput = map(leftoutputraw, (1500 - DZ), 1000, 0, -MAXRPM); }

  if (rightoutputraw > (1500 + DZ)) { rightoutput = map(rightoutputraw, (1500 + DZ), 2000, 0, MAXRPM); }
  if (rightoutputraw < (1500 + DZ) && rightoutputraw > (1500 - DZ)) { rightoutput = 0; }
  if (rightoutputraw < (1500 - DZ)) { rightoutput = map(rightoutputraw, (1500 - DZ), 1000, 0, -MAXRPM); }

  if (FCOK == 1) {
    motorL.move(rightoutput);
    motorR.move(leftoutput);
  }

  if (FCOK == 0) {
    motorL.move(0);
    motorR.move(0);
  }
}




void FOC_telemetry() {

  targetL = motorL.target;
  velocityL = motorL.shaft_velocity;
  voltageqL = motorL.voltage.q;
  currentqL = motorL.current.q;
  PhaseCurrent_s currentsL = current_senseL.getPhaseCurrents();
  lca = (currentsL.a * 1000);
  lcb = (currentsL.b * 1000);
  lcc = (currentsL.c * 1000);
  lc = current_senseL.getDCCurrent();



  targetR = motorR.target;
  velocityR = motorR.shaft_velocity;
  voltageqR = motorR.voltage.q;
  currentqR = motorR.current.q;
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
