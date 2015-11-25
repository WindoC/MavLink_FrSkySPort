// Used to calculate an average vibration level using accelerometers
#define accBufferSize 5
int32_t accXBuffer[accBufferSize];
int32_t accYBuffer[accBufferSize];
int32_t accZBuffer[accBufferSize];
int nrSamplesX = 0;
int nrSamplesY = 0;
int nrSamplesZ = 0;

// Used to calculate the average voltage/current between each frksy poll-request.
// A bit overkill since we most of the time only receive one sample from mavlink between each poll.
// voltageMinimum is used to report the lowest value received through mavlink between each poll frsky poll-request.
int32_t currentSum = 0;
uint16_t currentCount = 0;
uint32_t voltageSum = 0;
uint16_t voltageCount = 0;
uint16_t voltageMinimum = 0;

// Don't respond to FAS/FLVSS request until it looks like the voltage received through mavlink as stabilized.
// This is a try to eliminate most of the low voltage alarms recevied upon model power up.
boolean voltageStabilized = false;
uint16_t voltageLast = 0;

// Store a voltage reading received through mavlink
void storeVoltageReading(uint16_t value)
{
  // Try to determine if the voltage has stabilized
  if(voltageStabilized == false)
  {
    // if we have a mavlink voltage, and its less then 0.5V higher then the last sample we got
    if(value > 3000 && (value - voltageLast)<500)
    {
      // The voltage seems to have stabilized
      voltageStabilized = true;
    }
    else
    {
      // Reported voltage seems to still increase. Save this sample
      voltageLast = value; 
    }
    return;
  }

  // Store this reading so we can return the average if we get multiple readings between the polls
  voltageSum += value;
  voltageCount++;
  // Update the minimu voltage if this is lover
  if(voltageMinimum < 1 || value < voltageMinimum)
    voltageMinimum = value;
}

// Store a current reading received through mavlink
void storeCurrentReading(int16_t value)
{
  // Only store if the voltage seems to have stabilized
  if(!voltageStabilized)
    return;
  currentSum += value;
  currentCount++;
}

// Calculates and returns the average voltage value received through mavlink since the last time this function was called.
// After the function is called the average is cleared.
// Return 0 if we have no voltage reading
uint16_t readAndResetAverageVoltage()
{
  if(voltageCount < 1)
    return 0;
    
#ifdef DEBUG_AVERAGE_VOLTAGE
  debugSerial.print(millis());
  debugSerial.print("\tNumber of samples for voltage average: ");
  debugSerial.print(voltageCount);
  debugSerial.println();      
#endif

  uint16_t avg = voltageSum / voltageCount;

  voltageSum = 0;
  voltageCount = 0;

  return avg;
}

// Return the lowest voltage reading received through mavlink since the last time function was called.
// After the function is called the value is cleard.
// Return 0 if we have no new reading
uint16_t readAndResetMinimumVoltage()
{
  uint16_t tmp = voltageMinimum;
  voltageMinimum = 0;
  return tmp;  
}

// Calculates and returns the average current value received through mavlink since the last time this function was called.
// After the function is called the average is cleared.
// Return 0 if we have no voltage reading
uint16_t readAndResetAverageCurrent()
{
  if(currentCount < 1)
    return 0;
  
  uint16_t avg = currentSum >= 0 ? currentSum / currentCount : 0;

  currentSum = 0;
  currentCount = 0;

  return avg;
}

void storeAccX(int32_t value)
{
  if(nrSamplesX < accBufferSize)
  {
    nrSamplesX++;
  }
  uint8_t i;
  for(i=accBufferSize-1;i>0;i--)
  {
    accXBuffer[i]=accXBuffer[i-1];
  }
  accXBuffer[0] = value;
}
void storeAccY(int32_t value)
{
  if(nrSamplesY < accBufferSize)
  {
    nrSamplesY++;
  }
  uint8_t i;
  for(i=accBufferSize-1;i>0;i--)
  {
    accYBuffer[i]=accYBuffer[i-1];
  }
  accYBuffer[0] = value;
}
void storeAccZ(int32_t value)
{
  if(nrSamplesZ < accBufferSize)
  {
    nrSamplesZ++;
  }
  uint8_t i;
  for(i=accBufferSize-1;i>0;i--)
  {
    accZBuffer[i]=accZBuffer[i-1];
  }
  accZBuffer[0] = value;
}

int32_t fetchAccX()
{
  int32_t min=32000;
  int32_t max=-32000;
  for(int i=0; i<nrSamplesX; i++)
  {
    if(accXBuffer[i]<min)
    {
      min = accXBuffer[i];
    }
    if(accXBuffer[i]>max)
    {
      max = accXBuffer[i];
    }
  }
  return max - min;
}

int32_t fetchAccY()
{
  int32_t min=32000;
  int32_t max=-32000;
  for(int i=0; i<nrSamplesY; i++)
  {
    if(accYBuffer[i]<min)
    {
      min = accYBuffer[i];
    }
    if(accYBuffer[i]>max)
    {
      max = accYBuffer[i];
    }
  }
  return max - min;
}

int32_t fetchAccZ()
{
  int32_t min=32000;
  int32_t max=-32000;
  for(int i=0; i<nrSamplesZ; i++)
  {
    if(accZBuffer[i]<min)
    {
      min = accZBuffer[i];
    }
    if(accZBuffer[i]>max)
    {
      max = accZBuffer[i];
    }
  }
  return max - min;
}


void parseStatusText(int32_t severity, String text)
{
  uint16_t textId = 0;
  
  if(text == "")                                                          textId = 0;

  //APMrover2/GCS_Mavlink.cpp
  else if(text == F("Initialising APM..."))                                                            textId = 1;

  else if(text == F("Unsupported preflight calibration"))                                                            textId = 2;
  else if(text == F("command received:"))                                                            textId = 3;

  //APMrover2/Log.cpp
  else if(text == F("ERASING LOGS"))                                                            textId = 4;
  else if(text == F("No dataflash card inserted"))                                                            textId = 5;

  //APMrover2/Steering.cpp
  else if(text == F("AUTO triggered off"))                                                            textId = 6;
  else if(text == F("Triggered AUTO with pin"))                                                            textId = 7;
  else if(text.startsWith(F("Triggered AUTO")))                                                   textId = 8;

  //APMrover2/commands.cpp
  else if(text == F("Resetting prev_WP"))                                                            textId = 9;
  else if(text == F("init home"))                                                            textId = 10;

  //APMrover2/commands_logic.cpp
  else if(text == F("verify_conditon: Unsupported command"))                                                            textId = 11;
  else if(text == F("Reached Destination"))                                                            textId = 12;
  else if(text.startsWith(F("Cruise speed:")))                                                   textId = 13;
  else if(text.startsWith(F("Cruise throttle:")))                                                   textId = 14;
  else if(text.startsWith(F("Executing command ID")))                                                   textId = 15;

  else if(text == F("No commands. Can't set AUTO - setting HOLD"))                                                            textId = 16;

  else if(text.startsWith(F("Passed Waypoint")))                                                   textId = 17;
  else if(text.startsWith(F("Reached Destination: Distance away")))                                                   textId = 18;
  else if(text.startsWith(F("Reached Waypoint")))                                                   textId = 19;


  //APMrover2/navigation.cpp
  else if(text == F("<navigate> WP error - distance < 0"))                                                            textId = 20;

  //APMrover2/sensors.cpp
  else if(text == F("Calibrating barometer"))                                                            textId = 21;
  else if(text == F("barometer calibration complete"))                                                            textId = 22;
/*
  else if(text == F("Obstacle passed"))                                                            textId = 23;
  else if(text.startsWith(F("Sonar obstacle")))                                                   textId = 24;
  else if(text.startsWith(F("Sonar1 obstacle")))                                                   textId = 25;
  else if(text.startsWith(F("Sonar2 obstacle")))                                                   textId = 26;
*/
  //APMrover2/system.cpp
  else if(text == F("<startup_ground> GROUND START"))                                                            textId = 27;
  else if(text == F("<startup_ground> With Delay"))                                                            textId = 28;
  else if(text.endsWith(F("Ready to drive.")))                                                   textId = 29;
  else if(text == F("Beginning INS calibration"))                                                            textId = 30;
  else if(text == F("Warming up ADC..."))                                                            textId = 31;

  else if(text == F("Failsafe ended"))                                                            textId = 32;
  else if(text.startsWith(F("Failsafe trigger")))                                                   textId = 33;

  //AntennaTracker/GCS_Mavlink.cpp
  else if(text == F("new HOME received"))                                                            textId = 34;

  //AntennaTracker/system.cpp
  else if(text.endsWith(F("Ready to track.")))                                                   textId = 35;
/*
  //ArduCopter/GCS_Mavlink.cpp
  else if(text == F("error setting rally point"))                                                            textId = 36;
  else if(text == F("bad rally point index"))                                                            textId = 37;
  else if(text == F("bad rally point message ID"))                                                            textId = 38;
  else if(text == F("bad rally point message count"))                                                            textId = 39;
  else if(text == F("failed to set rally point"))                                                            textId = 40;

  //ArduCopter/Log.cpp
  else if(text.startsWith(F("Erasing logs")))                                                   textId = 41;
  else if(text.startsWith(F("Log erase complete")))                                                   textId = 42;
  else if(text == F("No dataflash inserted"))                                                            textId = 43;
*/
  //ArduCopter/commands_logic.cpp
  else if(text.startsWith(F("Reached Command")))                                                   textId = 44;

  //ArduCopter/compassmot.cpp
  else if(text == F("CURRENT"))                                                            textId = 45;
  else if(text == F("Calibration Successful!"))                                                            textId = 46;
  else if(text == F("Failed!"))                                                            textId = 47;
  else if(text == F("Not landed"))                                                            textId = 48;
  else if(text == F("RC not calibrated"))                                                            textId = 49;
  else if(text == F("STARTING CALIBRATION"))                                                            textId = 50;
  else if(text == F("THROTTLE"))                                                            textId = 51;
  else if(text == F("check compass"))                                                            textId = 52;
  else if(text.startsWith(F("compass disabled")))                                                   textId = 53;
  else if(text == F("thr not zero"))                                                            textId = 54;

  //ArduCopter/control_autotune.cpp
  else if(text == F("AutoTune: Failed"))                                                            textId = 55;
  else if(text == F("AutoTune: Saved Gains"))                                                            textId = 56;
  else if(text == F("AutoTune: Started"))                                                            textId = 57;
  else if(text == F("AutoTune: Stopped"))                                                            textId = 58;
  else if(text == F("AutoTune: Success"))                                                            textId = 59;

  //ArduCopter/crash_check.cpp
  else if(text == F("Crash: Disarming"))                                                            textId = 60;
  else if(text == F("Parachute: Landed"))                                                            textId = 61;
  else if(text == F("Parachute: Released!"))                                                            textId = 62;
  else if(text == F("Parachute: Too Low"))                                                            textId = 63;

  //ArduCopter/ekf_check.cpp
  else if(text == F("EKF variance"))                                                            textId = 64;

  //ArduCopter/esc_calibration.cpp
  else if(text == F("ESC Calibration: auto calibration"))                                                            textId = 65;
  else if(text == F("ESC Calibration: passing pilot throttle to ESCs"))                                                            textId = 66;
  else if(text == F("ESC Calibration: push safety switch"))                                                            textId = 67;
  else if(text == F("ESC Calibration: restart board"))                                                            textId = 68;
/*
  //ArduCopter/events.cpp
  else if(text == F("Low Battery!"))                                                            textId = 69;
*/
  //ArduCopter/motor_test.cpp
  else if(text == F("Motor Test: RC not calibrated"))                                                            textId = 70;
  else if(text == F("Motor Test: Safety Switch"))                                                            textId = 71;
  else if(text == F("Motor Test: vehicle not landed"))                                                            textId = 72;


  //ArduCopter/motors.cpp
  else if(text == F("ARMING MOTORS"))                                                            textId = 73;
  else if(text == F("Arm: Accelerometers not healthy"))                                                            textId = 74;
  else if(text == F("Arm: Altitude disparity"))                                                            textId = 75;
  else if(text == F("Arm: Barometer not healthy"))                                                            textId = 76;
  else if(text == F("Arm: Check Battery"))                                                            textId = 77;
  else if(text == F("Arm: Collective below Failsafe"))                                                            textId = 78;
  else if(text == F("Arm: Collective too high"))                                                            textId = 79;
  else if(text == F("Arm: Gyro calibration failed"))                                                            textId = 80;
  else if(text == F("Arm: Gyros not healthy"))                                                            textId = 81;
  else if(text == F("Arm: Leaning"))                                                            textId = 82;
  else if(text == F("Arm: Mode not armable"))                                                            textId = 83;
  else if(text == F("Arm: Motor Emergency Stopped"))                                                            textId = 84;
  else if(text == F("Arm: Motor Interlock Enabled"))                                                            textId = 85;
  else if(text == F("Arm: Rotor Control Engaged"))                                                            textId = 86;
  else if(text == F("Arm: Safety Switch"))                                                            textId = 87;
  else if(text == F("Arm: Throttle below Failsafe"))                                                            textId = 88;
  else if(text == F("Arm: Throttle too high"))                                                            textId = 89;
  else if(text == F("Arm: Waiting for Nav Checks"))                                                            textId = 90;
  else if(text == F("Arm: check fence"))                                                            textId = 91;
  else if(text == F("DISARMING MOTORS"))                                                            textId = 92;
  else if(text == F("Locate Copter Alarm!"))                                                            textId = 93;
  else if(text == F("PreArm: ACRO_BAL_ROLL/PITCH"))                                                            textId = 94;
  else if(text == F("PreArm: Accelerometers not healthy"))                                                            textId = 95;
  else if(text == F("PreArm: Accels not calibrated"))                                                            textId = 96;
  else if(text == F("PreArm: Altitude disparity"))                                                            textId = 97;
  else if(text == F("PreArm: Barometer not healthy"))                                                            textId = 98;
  else if(text == F("PreArm: Check ANGLE_MAX"))                                                            textId = 99;
  else if(text == F("PreArm: Check Battery"))                                                            textId = 100;
  else if(text == F("PreArm: Check Board Voltage"))                                                            textId = 101;
  else if(text == F("PreArm: Check FS_THR_VALUE"))                                                            textId = 102;
  else if(text == F("PreArm: Check Heli Parameters"))                                                            textId = 103;
  else if(text == F("PreArm: Check mag field"))                                                            textId = 104;
  else if(text == F("PreArm: Collective below Failsafe"))                                                            textId = 105;
  else if(text == F("PreArm: Compass not calibrated"))                                                            textId = 106;
  else if(text == F("PreArm: Compass not healthy"))                                                            textId = 107;
  else if(text == F("PreArm: Compass offsets too high"))                                                            textId = 108;
  else if(text == F("PreArm: Duplicate Aux Switch Options"))                                                            textId = 109;
  else if(text == F("PreArm: EKF compass variance"))                                                            textId = 110;
  else if(text == F("PreArm: EKF-home variance"))                                                            textId = 111;
  else if(text == F("PreArm: Gyros not healthy"))                                                            textId = 112;
  else if(text == F("PreArm: High GPS HDOP"))                                                            textId = 113;
  else if(text == F("PreArm: Interlock/E-Stop Conflict"))                                                            textId = 114;
  else if(text == F("PreArm: Motor Emergency Stopped"))                                                            textId = 115;
  else if(text == F("PreArm: Motor Interlock Enabled"))                                                            textId = 116;
  else if(text == F("PreArm: Need 3D Fix"))                                                            textId = 117;
  else if(text == F("PreArm: RC not calibrated"))                                                            textId = 118;
  else if(text == F("PreArm: Throttle below Failsafe"))                                                            textId = 119;
  else if(text == F("PreArm: Waiting for Nav Checks"))                                                            textId = 120;
  else if(text == F("PreArm: check fence"))                                                            textId = 121;
  else if(text == F("PreArm: check range finder"))                                                            textId = 122;
  else if(text == F("PreArm: inconsistent Accelerometers"))                                                            textId = 123;
  else if(text == F("PreArm: inconsistent Gyros"))                                                            textId = 124;
  else if(text == F("PreArm: inconsistent compasses"))                                                            textId = 125;

  //ArduCopter/switches.cpp
  else if(text == F("Trim saved"))                                                            textId = 126;

  //ArduCopter/system.cpp
//  else if(text == F("GROUND START"))                                                            textId = 127;
//  else if(text == F("Waiting for first HIL_STATE message"))                                                            textId = 128;

  //ArduPlane/ArduPlane.cpp
//  else if(text == F("Disable fence failed (autodisable)"))                                                            textId = 129;
//  else if(text == F("Disable fence floor failed (autodisable)"))                                                            textId = 130;
//  else if(text == F("Fence disabled (autodisable)"))                                                            textId = 131;
//  else if(text == F("Fence floor disabled (auto disable)"))                                                            textId = 132;
//  else if(text.startsWith(F("FBWA tdrag mode")))                                                   textId = 133;

  //ArduPlane/Attitude.cpp
  //else if(text == F("Demo Servos!"))                                                            textId = 134;
  else if(text.startsWith(F("Throttle unsuppressed - altitude")))                                                   textId = 135;
  else if(text.startsWith(F("Throttle unsuppressed - speed")))                                                   textId = 136;


  //ArduPlane/GCS_Mavlink.cpp
  else if(text == F("Fence floor disabled."))                                                            textId = 137;
  else if(text == F("Go around command accepted."))                                                            textId = 138;
  else if(text == F("Rejected go around command."))                                                            textId = 139;
  else if(text == F("bad fence point"))                                                            textId = 140;
  else if(text == F("fencing must be disabled"))                                                            textId = 141;
//  else if(text.startsWith(F("set home to")))                                                   textId = 142;
/*
  //ArduPlane/Log.cpp
  else if(text == F("Erasing logs"))                                                            textId = 143;
  else if(text == F("Log erase complete"))                                                            textId = 144;
*/
  //ArduPlane/arming_checks.cpp
  else if(text == F("PreArm: LIM_PITCH_MAX too small"))                                                            textId = 145;
  else if(text == F("PreArm: LIM_PITCH_MIN too large"))                                                            textId = 146;
  else if(text == F("PreArm: LIM_ROLL_CD too small"))                                                            textId = 147;
  else if(text == F("PreArm: invalid THR_FS_VALUE for rev throttle"))                                                            textId = 148;

  //ArduPlane/commands.cpp
  else if(text.startsWith(F("gps alt:")))                                                   textId = 149;

  //ArduPlane/commands_logic.cpp
  else if(text == F("Enable fence failed (cannot autoenable"))                                                            textId = 150;
//  else if(text == F("Fence enabled. (autoenabled)"))                                                            textId = 151;
  else if(text == F("verify_conditon: Invalid or no current Condition cmd"))                                                            textId = 152;
  else if(text == F("verify_nav: Invalid or no current Nav cmd"))                                                            textId = 153;
//  else if(text == F("Reached altitude"))                                                            textId = 154;
//  else if(text == F("Reached home"))                                                            textId = 155;
//  else if(text == F("verify_nav: LOITER orbits complete"))                                                            textId = 156;
//  else if(text == F("verify_nav: LOITER time complete"))                                                            textId = 157;
//  else if(text.startsWith(F("Executing nav command ID")))                                                   textId = 158;
//  else if(text.startsWith(F("Fence floor disabled.")))                                                   textId = 159;
//  else if(text.startsWith(F("Holding course")))                                                   textId = 160;
//  else if(text.startsWith(F("Reached descent rate")))                                                   textId = 161;
  else if(text == F("Returning to Home"))                                                            textId = 162;
//  else if(text.startsWith(F("Set airspeed")))                                                   textId = 163;
//  else if(text.startsWith(F("Set fence enabled state to")))                                                   textId = 164;
//  else if(text.startsWith(F("Set groundspeed")))                                                   textId = 165;
//  else if(text.startsWith(F("Set inverted")))                                                   textId = 166;
//  else if(text.startsWith(F("Set throttle")))                                                   textId = 167;
//  else if(text.startsWith(F("Takeoff complete at")))                                                   textId = 168;
  else if(text.startsWith(F("Unable to set fence enabled state to")))                                                   textId = 169;
  else if(text.startsWith(F("Unabled to disable fence floor.")))                                                   textId = 170;
/*
  //ArduPlane/control_modes.cpp
  else if(text == F("PX4IO Override disabled"))                                                            textId = 171;
  else if(text == F("PX4IO Override enable failed"))                                                            textId = 172;
  else if(text == F("PX4IO Override enabled"))                                                            textId = 173;
*/
  //ArduPlane/events.cpp
  else if(text == F("No GCS heartbeat."))                                                            textId = 174;
  else if(text == F("Failsafe - Long event on"))                                                            textId = 175;
  else if(text == F("Failsafe - Short event off"))                                                            textId = 176;
  else if(text == F("Failsafe - Short event on"))                                                            textId = 177;
//  else if(text.startsWith(F("Low Battery")))                                                   textId = 178;

/*
  //ArduPlane/geofence.cpp
  else if(text == F("geo-fence setup error"))                                                            textId = 179;
  else if(text == F("geo-fence OK"))                                                            textId = 180;
  else if(text == F("geo-fence loaded"))                                                            textId = 181;
  else if(text == F("geo-fence triggered"))                                                            textId = 182;
*/
  //ArduPlane/landing.cpp
  else if(text == F("Unable to start landing sequence."))                                                            textId = 183;
  else if(text == F("Auto-Disarmed"))                                                            textId = 184;
  else if(text == F("Landing sequence begun."))                                                            textId = 185;  
  else if(text.startsWith(F("Distance from LAND point")))                                                   textId = 186;
/*
  else if(text.startsWith(F("Flare crash detected:")))                                                   textId = 187;
  else if(text.startsWith(F("Flare")))                                                   textId = 188;

  //ArduPlane/radio.cpp
  else if(text.startsWith(F("MSG FS OFF")))                                                   textId = 189;
  else if(text.startsWith(F("MSG FS ON")))                                                   textId = 190;

  //ArduPlane/sensors.cpp
  else if(text == F("zero airspeed calibrated"))                                                            textId = 191;

  //ArduPlane/system.cpp
  else if(text == F("NO airspeed"))                                                            textId = 192;
  else if(text.endsWith(F("Ready to FLY.")))                                                   textId = 193;
  else if(text == F("Beginning INS calibration"))                                                            textId = 194;

  //ArduPlane/takeoff.cpp
  else if(text == F("FBWA tdrag off"))                                                            textId = 195;
  else if(text.startsWith(F("Armed AUTO")))                                                   textId = 196;
  else if(text == F("Bad Launch AUTO"))                                                            textId = 197;
  else if(text == F("Timeout AUTO"))                                                            textId = 198;
  else if(text == F("Timer Interrupted AUTO"))                                                            textId = 199;

  //libraries/APM_OBC/APM_OBC.cpp
*/
  else if(text == F("Dual loss TERMINATE"))                                                            textId = 200;
//  else if(text == F("Fence TERMINATE"))                                                            textId = 201;
//  else if(text == F("GCS OK"))                                                            textId = 202;
//  else if(text == F("GPS OK"))                                                            textId = 203;

  else if(text == F("RC failure terminate"))                                                            textId = 204;
//  else if(text == F("Starting AFS_AUTO"))                                                            textId = 205;
  else if(text == F("State DATA_LINK_LOSS"))                                                            textId = 206;
  else if(text == F("State GPS_LOSS"))                                                            textId = 207;

  //libraries/AP_Arming/AP_Arming.cpp
  else if(text == F("PreArm: 3D accel cal needed"))                                                            textId = 208;
  else if(text == F("PreArm: AHRS not healthy!"))                                                            textId = 209;
  else if(text == F("PreArm: Bad GPS Position"))                                                            textId = 210;
  else if(text == F("PreArm: Barometer not healthy!"))                                                            textId = 211;
  else if(text == F("PreArm: Battery failsafe on."))                                                            textId = 212;
  else if(text == F("PreArm: Compass not healthy!"))                                                            textId = 213;
  else if(text == F("PreArm: Hardware Safety Switch"))                                                            textId = 214;
  else if(text == F("PreArm: Radio failsafe on"))                                                            textId = 215;
  else if(text == F("PreArm: accels not healthy!"))                                                            textId = 216;
  else if(text == F("PreArm: airspeed not healthy"))                                                            textId = 217;
  else if(text == F("PreArm: gyros not calibrated!"))                                                            textId = 218;
  else if(text == F("PreArm: gyros not healthy!"))                                                            textId = 219;
  else if(text == F("PreArm: inconsistent gyros"))                                                            textId = 220;
  else if(text == F("PreArm: logging not available"))                                                            textId = 221;
  else if(text == F("Throttle armed!"))                                                            textId = 222;
  else if(text == F("Throttle disarmed!"))                                                            textId = 223;
/*
  //libraries/GCS_MAVLink/GCS_Common.cpp
  else if(text == F("flight plan received"))                                                            textId = 224;
  else if(text == F("flight plan update rejected"))                                                            textId = 225;

  //modules/PX4Firmware/src/modules/mavlink/mavlink_ftp.cpp
  else if(text == F("FTP: can't open path (file system corrupted?)"))                                                            textId = 226;
  else if(text == F("FTP: list readdir_r failure"))                                                            textId = 227;

  //modules/PX4Firmware/src/modules/mavlink/mavlink_main.cpp
  else if(text == F("Save params and reboot to change COMPID"))                                                            textId = 228;
  else if(text == F("Save params and reboot to change SYSID"))                                                            textId = 229;

  //modules/PX4Firmware/src/modules/mavlink/mavlink_mission.cpp
  else if(text == F("ERROR: Waypoint index exceeds list capacity"))                                                            textId = 230;
  else if(text == F("ERROR: can't save mission state"))                                                            textId = 231;
  else if(text == F("ERROR: wp index out of bounds"))                                                            textId = 232;
  else if(text == F("IGN MISSION_ITEM: Busy"))                                                            textId = 233;
  else if(text == F("IGN MISSION_ITEM: No transfer"))                                                            textId = 234;
  else if(text == F("IGN MISSION_ITEM_REQUEST: No active transfer"))                                                            textId = 235;
  else if(text == F("IGN REQUEST LIST: Busy"))                                                            textId = 236;
  else if(text == F("Operation timeout"))                                                            textId = 237;
  else if(text == F("REJ. WP CMD: partner id mismatch"))                                                            textId = 238;
  else if(text == F("Unable to read from micro SD"))                                                            textId = 239;
  else if(text == F("Unable to write on micro SD"))                                                            textId = 240;
  else if(text == F("WPM: ERR: not all items sent -> IDLE"))                                                            textId = 241;
  else if(text == F("WPM: IGN CLEAR CMD: Busy"))                                                            textId = 242;
  else if(text == F("WPM: IGN MISSION_COUNT: Busy"))                                                            textId = 243;
  else if(text == F("WPM: IGN WP CURR CMD: Busy"))                                                            textId = 244;
  else if(text == F("WPM: REJ. CMD: Busy"))                                                            textId = 245;
  else if(text == F("WPM: REJ. CMD: Req. WP was unexpected"))                                                            textId = 246;
  else if(text == F("WPM: REJ. CMD: partner id mismatch"))                                                            textId = 247;
  else if(text == F("WPM: WP CURR CMD: Error setting ID"))                                                            textId = 248;
  else if(text == F("WPM: WP CURR CMD: Not in list"))                                                            textId = 249;
  else if(text == F("WP CMD OK TRY AGAIN"))                                                            textId = 250;
  else if(text == F("WPM: COUNT 0: CLEAR MISSION"))                                                            textId = 251;
  else if(text == F("WPM: Transfer complete."))                                                            textId = 252;
  else if(text == F("WPM: mission is empty"))                                                            textId = 253;
*/
  //ArduCopter/motors.cpp
  else if(text.startsWith(F("PreArm:")))                                                   textId = 254;

  // Unknown text (textId = 1023)
  else                                                                    textId = 1023;
  
    ap_status_text_id = textId;

#ifdef DEBUG_PARSE_STATUS_TEXT
  //debugSerial.print(millis());
  //debugSerial.print("\tparseStatusText. severity: ");
  //debugSerial.print(severity);
  //debugSerial.print(", text: \"");
  //debugSerial.print(text);
  //debugSerial.print("\" textId: ");
  //debugSerial.print(textId);
  /debugSerial.println();
#endif
}



