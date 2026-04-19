// 6-servo pick-and-place sorter (IK Dynamic)
// Pins: Base=9, Shoulder=8, Elbow=10, WristPitch=6, WristRotate=7, Gripper=11

const int BASE_PIN     = 9;
const int SHOULDER_PIN = 8;
const int ELBOW_PIN    = 10;
const int WRIST_P_PIN  = 6;
const int WRIST_R_PIN  = 7;
const int GRIPPER_PIN  = 11;

const unsigned long FRAME_US = 20000; 
const unsigned long GRIP_HOLD_MS = 1000;
const int STEP_DELAY_MS = 35; // slower->faster

// robot dimensions in mm
const float L1 = 112.75;  // shoulder to elbow
const float L2 = 219.75; // elbow to tip of gripper (94.75 + 125)
const float Z_TABLE = -102.28; 
const float Z_SAFE = -50.0; 

// calibrated servo offsets
const int S_VERT_SERVO = 85;// shoulder is vertical at 85
const int E_VERT_SERVO = 110; // elbow is straight at 110
const int E_HORIZ_SERVO = 170;// elbow is at 90deg at 25
const int P_ALIGN_SERVO = 103; // wrist pitch aligned with forearm
const int G_OPEN = 180;
const int G_CLOSE = 130;

// current positions
int curB = 90, curS = 85, curE = 110, curWP = 103, curWR = 90, curG = 180;

// default home position
const int HOME_B = 90, HOME_S = 85, HOME_E = 110, HOME_WP = 103, HOME_WR = 90, HOME_G = 180;

// bin coordinates (x, y, z)
const float BIN1_XYZ[] = {150.0, 100.0, -70.0};  // leds
const float BIN2_XYZ[] = {150.0, -100.0, -70.0}; // resistors 

struct RobotPose { int b, s, e, wp; bool reachable; };

int angleToPulseUs(int angle) {
  angle = constrain(angle, 0, 180);
  return 600 + (angle * 1800) / 180;
}

void sendFrame(int b, int s, int e, int wp, int wr, int g) {
  int pulses[] = {angleToPulseUs(b), angleToPulseUs(s), angleToPulseUs(e), angleToPulseUs(wp), angleToPulseUs(wr), angleToPulseUs(g)};
  int pins[] = {BASE_PIN, SHOULDER_PIN, ELBOW_PIN, WRIST_P_PIN, WRIST_R_PIN, GRIPPER_PIN};

  unsigned long start = micros();
  for(int i=0; i<6; i++) {
    digitalWrite(pins[i], HIGH);
    delayMicroseconds(pulses[i]);
    digitalWrite(pins[i], LOW);
  }
  while (micros() - start < FRAME_US);
}

// moves the servos step-by-step
void holdPose(int b, int s, int e, int wp, int wr, int g, unsigned long ms) {
  bool arrived = false;
  while (!arrived) {
    arrived = true;
    if (curB != b) { curB += (b > curB) ? 1 : -1; arrived = false; }
    if (curS != s) { curS += (s > curS) ? 1 : -1; arrived = false; }
    if (curE != e) { curE += (e > curE) ? 1 : -1; arrived = false; }
    if (curWP != wp) { curWP += (wp > curWP) ? 1 : -1; arrived = false; }
    if (curWR != wr) { curWR += (wr > curWR) ? 1 : -1; arrived = false; }
    if (curG != g) { curG += (g > curG) ? 1 : -1; arrived = false; }

    sendFrame(curB, curS, curE, curWP, curWR, curG);
    delay(STEP_DELAY_MS);
  }
  unsigned long t0 = millis();
  while (millis() - t0 < ms) sendFrame(curB, curS, curE, curWP, curWR, curG);
}

// calculate inverse kinematics using physical offsets
RobotPose getAnglesIK(float x, float y, float z) {
  RobotPose p;
  float R = sqrt(sq(x) + sq(y));
  float D = sqrt(sq(R) + sq(z));

  // check if the target is out of bounds
  if (D > (L1 + L2 - 5) || D < 60) {
    p.reachable = false;
    return p;
  }

  float a1 = atan2(z, R);
  float a2 = acos(constrain((sq(L1) + sq(D) - sq(L2)) / (2 * L1 * D), -1, 1));
  float a3 = acos(constrain((sq(L1) + sq(L2) - sq(D)) / (2 * L1 * L2), -1, 1));

  float mathS = (a1 + a2) * 180.0 / PI; // angle from horizontal
  float mathE = a3 * 180.0 / PI; // internal angle between links

  // apply the physical servo offsets and inversions
  p.b = 90 + (atan2(y, x) * 180.0 / PI);
  p.s = 175 - mathS; // map the math 90(vert) to our servo 85

  float elbow_ratio = (mathE - 90.0) / (180.0 - 90.0);
  p.e = E_HORIZ_SERVO + elbow_ratio * (E_VERT_SERVO - E_HORIZ_SERVO);

  // keep the gripper pointing straight down
  p.wp = 180 - (p.s + p.e); 
  p.wp = constrain(p.wp, 20, 160);
  
  p.reachable = true;
  return p;
}

// run the actual pick and place sequence
void doCycleDynamic(RobotPose hover, RobotPose pick, RobotPose bin) {
  holdPose(hover.b, hover.s, hover.e, hover.wp, 90, G_OPEN, 500);   // hover safely over the part
  holdPose(pick.b,  pick.s,  pick.e,  pick.wp,  90, G_OPEN, 400);   // drop down to it
  holdPose(pick.b,  pick.s,  pick.e,  pick.wp,  90, G_CLOSE, GRIP_HOLD_MS); // grab it
  holdPose(hover.b, hover.s, hover.e, hover.wp, 90, G_CLOSE, 500);  // lift it back up
  holdPose(bin.b,   bin.s,   bin.e,   bin.wp,   90, G_CLOSE, 800);  // move over to the bin
  holdPose(bin.b,   bin.s,   bin.e,   bin.wp,   90, G_OPEN, GRIP_HOLD_MS);  // let go
  holdPose(HOME_B, HOME_S, HOME_E, HOME_WP, HOME_WR, HOME_G, 800);  // go back home
}

void setup() {
  int pins[] = {9, 8, 10, 6, 7, 11};
  for(int i=0; i<6; i++) pinMode(pins[i], OUTPUT);
  Serial.begin(9600);
  
  // start up slowly and go to home position
  holdPose(HOME_B, HOME_S, HOME_E, HOME_WP, HOME_WR, HOME_G, 1000);
  Serial.println("READY");
}

/* // manual debugger
void loop() {
  if (Serial.available() > 0) {
    // reduce timeout to 10ms instead of 1000ms so it doesn't lag
    Serial.setTimeout(10); 
    char motor = Serial.read(); 
    int angle = Serial.parseInt();
    
    if (motor == 'B') curB = angle;
    else if (motor == 'S') curS = angle;
    else if (motor == 'E') curE = angle;
    else if (motor == 'P') curWP = angle;
    else if (motor == 'R') curWR = angle;
    else if (motor == 'G') curG = angle;

    Serial.print("Target: "); Serial.print(motor); 
    Serial.println(angle);
  }
  sendFrame(curB, curS, curE, curWP, curWR, curG);
}
*/

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    char type = data.charAt(0);
    int firstComma = data.indexOf(','), secondComma = data.lastIndexOf(',');
    float tx = data.substring(firstComma + 1, secondComma).toFloat();
    float ty = data.substring(secondComma + 1).toFloat();

    RobotPose pick = getAnglesIK(tx, ty, Z_TABLE);
    Serial.print("Target X: "); Serial.print(tx);
    Serial.print(" Y: "); Serial.print(ty);
    Serial.print(" | Angles B:"); Serial.print(pick.b);
    Serial.print(" S:"); Serial.print(pick.s);
    Serial.print(" E:"); Serial.print(pick.e);
    Serial.print(" WP:"); Serial.println(pick.wp);
    
    const float* bXYZ = (type == 'L') ? BIN1_XYZ : BIN2_XYZ;
    RobotPose bin = getAnglesIK(bXYZ[0], bXYZ[1], bXYZ[2]);
    RobotPose hover = getAnglesIK(tx, ty, Z_SAFE);
    
    if (pick.reachable && bin.reachable && hover.reachable) {
      doCycleDynamic(hover, pick, bin);
    } else {
      Serial.println("ERROR: Unreachable");
    }
    
    Serial.println("READY");
  }
}