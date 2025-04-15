void FOC_SETUP() {


  sensorL.init();
  sensorR.init();
  sensorL.enableInterrupts(doAL, doBL, doCL);
  sensorR.enableInterrupts(doAR, doBR, doCR);
  motorL.linkSensor(&sensorL);
  motorR.linkSensor(&sensorR);
  driverL.voltage_power_supply = 24;
  driverL.init();
  driverR.voltage_power_supply = 24;
  driverR.init();
  motorL.linkDriver(&driverL);
  motorR.linkDriver(&driverR);

  // aligning voltage [V]
  motorL.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motorL.velocity_index_search = 3;

  //Motion Control Mode Settings
  motorL.controller = MotionControlType::velocity;
  motorR.controller = MotionControlType::velocity;

  //Speed PI Loop Settings
  motorL.PID_velocity.P = 0.01;
  motorR.PID_velocity.P = 0.01;
  motorL.PID_velocity.I = 0.1;
  motorR.PID_velocity.I = 0.1;
  motorL.PID_velocity.D = 0;
  motorR.PID_velocity.D = 0;
  //Angle P Ring Settings
  motorL.P_angle.P = 20;
  motorR.P_angle.P = 20;
  //Maximum Motor Limit Voltage
  motorL.voltage_limit = 6;
  motorR.voltage_limit = 6;

  motorL.PID_velocity.output_ramp = 1000;
  motorR.PID_velocity.output_ramp = 1000;

  //Velocity low pass filter time constant
  motorL.LPF_velocity.Tf = 0.01f;
  motorR.LPF_velocity.Tf = 0.01f;

  //Set maximum speed limit
  motorL.velocity_limit = 45;
  motorR.velocity_limit = 45;


  Serial.begin(115200);
  motorL.useMonitoring(Serial);
  motorR.useMonitoring(Serial);

  motorL.monitor_variables = _MON_TARGET | _MON_VEL | _MON_VOLT_Q | _MON_CURR_Q;
  motorR.monitor_variables = _MON_TARGET | _MON_VEL | _MON_VOLT_Q | _MON_CURR_Q;
  motorL.monitor_downsample = 100;  // default 10
  motorR.monitor_downsample = 100;  // default 10

  //Initialize Motor
  motorL.init();
  motorR.init();
  //Initialize FOC
  motorL.initFOC();
  motorR.initFOC();

  Serial.println(F("Motor ready."));
}



void FOC_telemetry() {

  // current target value
  targetL = motorL.target;
  velocityL = motorL.shaft_velocity;
  voltageqL = motorL.voltage.q;
  currentqL = motorL.current.q;

  targetR = motorR.target;
  velocityR = motorR.shaft_velocity;
  voltageqR = motorR.voltage.q;
  currentqR = motorR.current.q;
}
