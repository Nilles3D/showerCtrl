// turning shower knobs based on temperature and time
long showerTime = 3.5 * 60 * 1000; //milliseconds
int goalTemp = 33; //deg. C


// ------------------------------------------------------
// Pin Definitions
// ------------------------------------------------------
// pins for Temperature sensor
const int T_IN = A0;  // analog read
// pins for Motor H
const int H_IN1 = 6;   // D6, white
const int H_IN2 = 9;   // D9, green
// pins for Motor C
const int C_IN1 = 5;   // D5, red
const int C_IN2 = 3;   // D3, blue


// ------------------------------------------------------
// Declarations
// ------------------------------------------------------
// ----
// Testing
// ----
int dir = 1; //test direction
int test_tim = 0; //test time
int test_delt = (255*2)/30; //test change in speed
int test_spd = -255 - test_delt; //test oomph
int test_step = 0; //calc later, test temp over motor change
// ----
// Temperature
// ----
int tempMax = 40; //deg. C, water
int tempTap = 10; //deg. C, water
int tempAmbMin = 15; //deg. C, air
int tempAmbMax = 0.9*tempMax; //deg. C, air
int tempWater = 37; //overridden by calculations later
int tempCorrection = 2; //deg. C to sensor's output
const long dRdT = 132; //millis / degree change of Motor C at full speed. Very approximate
int tempNow = 0; //deg. C
int dT_req = 0; //deg. C
int dR_req = 0; //fake radians / deg.C
// ----
// Motors (Adafruit 3777)
// ----
const int MOTOR_H = 0;
const int MOTOR_C = 1;
const int motor_Max_Spd = 124; //RPM
const int motor_Min_Spd = 95; //RPM
long maxTurn = 0; //fake radians
int fullSpeed = 0; //RPM
int revSpeed = -255; //power
const float gear_ratio = (2.625 / 0.75) * (5.75 / 0.75); // in/in
bool closed = false;
// ----
// Runtime
// ----
unsigned long motorH_time = 0;   // milliseconds
signed long motorH_log = 0;  //pseudo revolutions
unsigned long motorC_time = 0;
signed long motorC_log = 0;
long time1 = 0; //millis


// ------------------------------------------------------
// Sub routines
// ------------------------------------------------------

// Temperature reading
int getTemp () {
  int mV = 0;
  const int tempCount = 10;
  int tempRead = 0;
  int tempSum = 0;
  int sigFigs = 100;
    
  //Serial.println("Reading temperature sensor multiple times and getting an average...");
  //https://www.ti.com/lit/ds/symlink/lmt86.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1774490934045
  //page 9, Eq. 2
  for (int i=0; i < tempCount; i++){
    //read analog temperature sensor LMT86
    mV = analogRead(T_IN)*3300/1023;
    tempRead = int((-(10.888-sqrt(pow(10.888,2)+4*0.00347*(1777.3-mV)))/(2*0.00347)+30+tempCorrection)*sigFigs);
    tempSum += tempRead;
    /*
    Serial.print("  read ");
    Serial.print(mV);
    Serial.print(" and got temp = ");
    Serial.println(tempRead);
    */
    delay(15);
  }
  tempRead = (tempSum/tempCount*10+5)/10; //rounded
  Serial.print("  temperature = ");
  Serial.print(tempRead);
  Serial.print(" (");
  tempRead /= sigFigs;
  Serial.print(tempRead);
  Serial.println("C)");

  return tempRead;
}

// Motor timing 1
int bit2spd (int bitVolt) {
  //want RPM for motor given pin "voltage"
  if (bitVolt == 0) return 0;
  
  bitVolt = max(-255, min(bitVolt, 255));
  int bitVoltSign = bitVolt/abs(bitVolt);
  
  bitVolt = abs(bitVolt);
  int omega = motor_Min_Spd + (motor_Max_Spd - motor_Min_Spd)*(bitVolt - 160) / (255 - 160);
  
  Serial.println();
  Serial.print("bit2spd given ");
  Serial.print(bitVolt);
  Serial.print(" bits produces ");
  Serial.print(omega);
  Serial.println(" RPM");
  return omega*bitVoltSign;
}
// Motor timing 2
int spd2bit (float RPM) { 
  //have RPM on motor, get voltage-ish value
  
  if (RPM == 0) return 0;

  int spdSign = RPM / abs(RPM);
  RPM = abs(RPM);
  
  int vout = 160 + (255 - 160)*(RPM - motor_Min_Spd) / (motor_Max_Spd - motor_Min_Spd);
  
  Serial.println();
  Serial.print("spd2bit given ");
  Serial.print(RPM);
  Serial.print(" RPM makes ");
  Serial.print(vout);
  Serial.println(" bits");
  return vout*spdSign;
}
// Motor timing 3
int ang2time (int angle, int speed) { //deg_rack, RPM_pinion = milliseconds
  //want time to spin at speed to get an angle on big gear
  float ang = abs(angle/360.0); //revs
  speed = abs(speed);
  float rps = speed/60.0; //rev/s

  int t = 1000.0 * ang * gear_ratio / rps; //millis
  
  Serial.println();
  Serial.print("ang2time given ");
  Serial.print((String)angle+" degrees ("+ang+" revs) and ");
  Serial.print((String)speed+" RPM ("+rps+" rps) makes ");
  Serial.print(t);
  Serial.println(" milliseconds");
  return t;
}

// Unified Motor Control
void runMotor (int motorID, int direction, int motorTime = 0) {
  //inputs:
  //  motorID = which motor to turn
  //  direction = signed "velocity" (/255)
  //  motorTime = millis to run at speed

  Serial.println();
  Serial.print("runMotor given motor ");
  Serial.print(motorID);
  Serial.print(", direction ");
  Serial.print(direction);
  Serial.print(", and time ");
  Serial.println(motorTime);

  //dead range
  int min_react = 160; //90RPM
  if (abs(direction) < min_react){
    //time correction
    motorTime = direction * motorTime / min_react;
    //speed correction
    direction = int(round(direction/min_react)*min_react);
    Serial.print(" Direction corrected to ");
    Serial.print(direction);
    Serial.print(", and Time corrected to ");
    Serial.println(motorTime);
  }
  
  //safe range
  direction = max(-255, min(direction, 255));

  //ramp up time
  int rampTime = abs(150.0 * (float)direction / 255.0); //very loose definition
  if (rampTime > motorTime) rampTime *= 0.2;
  if (motorTime == 0) rampTime = 0;
  Serial.println((String)" Ramp time approximated as "+rampTime);

  //direction = radial velocity (i.e., speed and direction)
  if (motorID == MOTOR_H) {
    if (direction > 0) {          // positive = forward/clockwise
      Serial.println(" H fwd");
      analogWrite(H_IN1, direction);
      analogWrite(H_IN2, 0);
    } else if (direction < 0) {    // negative = reverse/counterclockwise
      Serial.println(" H rev");
      analogWrite(H_IN1, 0);
      analogWrite(H_IN2, abs(direction));
    } else {
      Serial.println(" H stop");
      analogWrite(H_IN1, 0);
      analogWrite(H_IN2, 0);
    }
    motorH_log += motorTime*direction; //psuedo radians
    Serial.print(" motorH_log now ");
    Serial.println(motorH_log);
    
    delay(motorTime + rampTime);

    analogWrite(H_IN1, 0); //static control
    analogWrite(H_IN2, 0);
  }

  else if (motorID == MOTOR_C) {
    if (direction > 0) {          // + = forward/clockwise
      Serial.println(" C fwd");
      analogWrite(C_IN1, direction);
      analogWrite(C_IN2, 0);
    } else if (direction < 0) {   // - = reverse/counterclockwise
      Serial.println(" C rev");
      analogWrite(C_IN1, 0);
      analogWrite(C_IN2, abs(direction));
    } else {
      Serial.println(" C stop");
      analogWrite(C_IN1, 0);
      analogWrite(C_IN2, 0);
    }
    motorC_log += motorTime*direction;
    Serial.print(" motorC_log now ");
    Serial.println(motorC_log);

    delay(motorTime + rampTime);

    analogWrite(C_IN1, 0);
    analogWrite(C_IN2, 0);
  }
}
// Motor return and stop
void closeAll () {
  //run motors back whatever they're logged for

  Serial.println("Closing all valves");
  int revTime_H = abs(int(motorH_log / revSpeed));
  
  runMotor(MOTOR_H, revSpeed, revTime_H + 1);
  runMotor(MOTOR_H, -200, 90);//extra rxtra

  int revTime_C = abs(int(motorC_log / revSpeed));
  runMotor(MOTOR_C, -revSpeed, revTime_C + 1); 
}



void setup() {

  // comms start
  Serial.begin(9600);
  int bypass = millis() + 2000;
  while (!Serial && millis() <= bypass) { }  // Wait for Serial on USB
  Serial.println("Starting system...");
  
  // Initialize pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(H_IN1, OUTPUT);
  pinMode(H_IN2, OUTPUT);
  pinMode(C_IN1, OUTPUT);
  pinMode(C_IN2, OUTPUT);

  // Get ambient temperature to set desired water temp
  int tempAmb = getTemp()-2; //startup causes some heat
  tempWater = (tempTap - tempMax)/(tempAmbMax - tempAmbMin)*(tempAmb - tempAmbMin) + tempMax;
  Serial.print("Water temp set at ");
  Serial.println(tempWater);
  
  // Motor check
  Serial.println();
  Serial.print("Using gear ratio ");
  Serial.println(gear_ratio);
  Serial.print("Full speed:");
  fullSpeed = bit2spd(255);
  
  // variable definitions
  Serial.print("For basic motor full revolution check: ");
  test_tim = ang2time(360, fullSpeed);
  Serial.print("For charting temperature per positoin increment: ");
  test_step = ang2time(30, 255);
  Serial.print("For either motor limit: ");
  maxTurn = ang2time(90, 255);

  // ------------------------------------------------------
  // Step 1: Motor H clockwise
  // ------------------------------------------------------
  Serial.println();
  Serial.print("Initial Motor H turn:");
  int h0 = ang2time(80, fullSpeed);
  runMotor(MOTOR_H, 255, h0);
  
  // ------------------------------------------------------
  // Step 2: Motor C counterclockwise
  // ------------------------------------------------------
  Serial.println();
  Serial.print("Initial Motor C turn:");
  int c0 = ang2time(30, fullSpeed);
  runMotor(MOTOR_C, -255, c0);
  
  // ------------------------------------------------------
  // Step 3: Wait for water to stabilize
  // ------------------------------------------------------
  Serial.println();
  Serial.println("Waiting for predetermined time ... ");
  delay(20 * 1000);

  Serial.println();
  Serial.println("Setup done");
  Serial.println();

  time1 = millis() + showerTime;
}

void loop() {
  // ----
  // Main functionality
  // ----
  while (millis()<time1){
    // Temperature control
    tempNow = getTemp();
    dT_req = goalTemp - tempNow;
    dR_req = dT_req*dRdT*255; //fake radians

    if (abs(dR_req) < 200*255) {
      //too small of an adjustment
      //do nothing
    }
    else if ((dR_req >= 0) && (motorC_log + dR_req >= 0) && (motorC_log <= 0)) {
      //lower displacement limit, cannot close that far
      Serial.print("+");
      runMotor(MOTOR_C, 255, abs(int(motorC_log / 255)));
    }
    else if ((dR_req <= 0) && (motorC_log + dR_req <= -maxTurn) && (motorC_log >= -maxTurn)){
      //upper displacement limit -- assumes fullspeed moves up until this point
      Serial.print("-");
      runMotor(MOTOR_C, -200, abs(int(((-maxTurn - motorC_log) / revSpeed))));
    }
    else {
      // fine adjustment
      Serial.print("=");
      runMotor(MOTOR_C, (dR_req/abs(dR_req))*255, abs(dR_req/255));
    }

    delay(5000); // wait for water to catch up
  }
  if (!closed){
    closeAll();
    closed = !closed;
  }

  // Testing
  
  // ----
  //motors on
  // runMotor(MOTOR_H, -test_spd, 500);
  // runMotor(MOTOR_C, -test_spd, 500);

  /*
  // ----
  //temperature check
  Serial.print("While MOTOR_C is at ");
  Serial.println(motorC_log);
  for (int r = 0; r <= 10000; r+=2500){
    // Serial.print(" The temperature at milliseconds ");
    // Serial.println(r);
    tempNow = getTemp();
    delay(r);
  }
  //adjust cold's position
  if (0 <= motorC_log) { //stop when returned to zero
    Serial.println();
    Serial.println("TEST Motor has returned beyond home position");
    if (dir <=0) { 
      Serial.println(" and will now stop");
      dir = 0;
      closeAll();
      while (true) {
        //end
      }
    }
    else {
      Serial.println(" and is now reversing");
      dir *= -1;
    }
  } 
  else if (motorC_log < -4*test_step*255) { //120 deg max
    Serial.println();
    Serial.println("TEST Motor is now reversing");
    dir *= -1;}
  runMotor(MOTOR_C, -dir*255, test_step);
  */
  
  /*
  // ----
  //speed and direction
  test_spd += dir*test_delt;
  runMotor(MOTOR_H, test_spd, 500);
  runMotor(MOTOR_C, test_spd, 500);
  if ((test_spd > 255) || (test_spd < -255)) dir *= -1;
  */
  
  // ----
  //basic time step
  //runMotor(MOTOR_H, 255, test_tim);
  //runMotor(MOTOR_C, 255, test_tim);

  /*
  // ----
  //led check
  analogWrite(H_IN1, 160);
  delay(250);
  analogWrite(H_IN1, 0);
  analogWrite(H_IN2, 160);
  delay(250);
  analogWrite(H_IN2, 0);
  analogWrite(C_IN1, 160);
  delay(250);
  analogWrite(C_IN1, 0);
  analogWrite(C_IN2, 160);
  delay(250);
  analogWrite(C_IN2, 0);
   */

  // ----
  // bootloader troubleshooting
  // digitalWrite(LED_BUILTIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
  // delay(500);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
  // delay(500);                      // wait for a second
  //Serial.println(5);
  
  // ----
}