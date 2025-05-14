void FOC_SETUP() {

  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = Vlimit;   // [V]
  motor.velocity_limit = 40; // [rad/s]
  
  driver1.voltage_power_supply = 24;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = Vlimit;   // [V]
  motor1.velocity_limit = 40; // [rad/s]

 
  //Open loop control mode setting
  motor.controller = MotionControlType::velocity_openloop;
  motor1.controller = MotionControlType::velocity_openloop;

  //Initialize the hardware
  motor.init();
  motor1.init();


  //Add T command
  command.add('T', doTarget, "target velocity");


  Serial.println(F("Motor ready."));
}



void FOC_telemetry() {

  // current target value
  targetL = motor.target;
  velocityL = motor.shaft_velocity;
  voltageqL = motor.voltage.q;
  currentqL = motor.current.q;

  targetR = motor1.target;
  velocityR = motor1.shaft_velocity;
  voltageqR = motor1.voltage.q;
  currentqR = motor1.current.q;
}
