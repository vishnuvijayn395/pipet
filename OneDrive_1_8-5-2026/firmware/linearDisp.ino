#include <AccelStepper.h>
#include <EEPROM.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSoftSpi.h"

#define OLED_CS    18
#define OLED_DC    5
#define OLED_RST   17
#define OLED_MOSI  16
#define OLED_CLK   4

SSD1306AsciiSoftSpi display;

// Encoder Pins 
const int ENC_CLK_PIN = 23;
const int ENC_DT_PIN = 22;
const int ENC_SW_PIN = 21;

int lastClkState;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 200;

bool displayNeedsUpdate = true;

enum UIMode { UI_MAIN_MENU, UI_VAR_SETUP, UI_ALIQUOT_ASP_SETUP, UI_ALIQUOT_DISP_SETUP, UI_RUNNING };
UIMode uiMode = UI_MAIN_MENU;
int mainMenuIndex = 0; 

enum PipetteState { STATE_READY, STATE_ASPIRATED, STATE_EMPTY, STATE_DISPENSED };
PipetteState pipetteState = STATE_READY;

float customDispenseVol = 10.0;
float aliquotTotalVol = 100.0;
float aliquotDispVol = 10.0;
float aliquotRemainingVol = 0.0;

unsigned long btnPressTime = 0;
bool btnIsPressed = false;
bool longPressHandled = false;

String motorAction = "";
unsigned long lastDisplayUpdate = 0;

const int STEP_PIN = 14;
const int DIR_PIN = 27;
const int ENABLE_PIN = 26; 
const int M0_PIN = 25;
const int M1_PIN = 33;
const int M2_PIN = 32;
const int LIMIT_SW_PIN = 13;  // Blowout limit switch

#define STEPDIR 1
AccelStepper pipetteStepper(STEPDIR, STEP_PIN, DIR_PIN);

String appCommand = "";



// === DYNAMIC CALIBRATION VARIABLES ===
long homePosition = 0;
long firstStopPosition = 0;  
long blowoutPosition = 0;    
float stepsPerUL = 4.5669; // Default fallback
float maxCapacity = 100.0;  // Default max volume in uL
float deceleration = 5000.0; // Acceleration/deceleration in steps/s^2

// EEPROM Addresses
const int ADDR_FIRST_STOP = 0;    // 4 bytes (long)
const int ADDR_BLOWOUT = 4;       // 4 bytes (long)
const int ADDR_STEPS_PER_UL = 8;  // 4 bytes (float)
const int ADDR_MAX_CAPACITY = 12; // 4 bytes (float)
const int ADDR_DECEL = 20;        // 4 bytes (float)
const int ADDR_CAL_NUM_POINTS = 24; // 1 byte
const int ADDR_CAL_ENABLED = 25;    // 1 byte
const int ADDR_CAL_POINTS = 26;     // 20 * 8 = 160 bytes
const int ADDR_SPEED = 186;         // 4 bytes (float)

// Multipoint Calibration Struct
struct CalPoint {
  float target;
  float measured;
};
byte calNumPoints = 0;
bool calEnabled = false;
CalPoint calPoints[20];


// State tracking for DONE signal
bool wasRunning = false;

// Limit switch â€” set by ISR, cleared in loop after handling
volatile bool limitSwitchTriggered = false;
volatile bool ignoreLimitSwitch = false;
volatile long switchHitPosition = 0;
int blowoutOffset = 80;   // Steps to travel past switch before saving blowout position

// We will use inline polling instead of an ISR to track the switch hit position
// accurately without worrying about interrupt edges and bounce.


void performAutoHome(bool isCalibration) {
  float savedSpeed = pipetteStepper.maxSpeed();
  float savedDecel = deceleration;
  
  if (savedSpeed < 100.0) savedSpeed = 10000.0;
  
  pipetteStepper.enableOutputs();
  pipetteStepper.setAcceleration(5000.0);
  ignoreLimitSwitch = true;
  
  // === DIRECTIONS (confirmed from hardware): ===
  // POSITIVE = DOWN (push/dispense) — toward the limit switch at bottom
  // NEGATIVE = UP (draw/retract) — away from the limit switch

  // 1. Move DOWN (POSITIVE) FAST until switch is pressed (HIGH)
  if (digitalRead(LIMIT_SW_PIN) == LOW) {
    pipetteStepper.setMaxSpeed(8000.0);
    pipetteStepper.moveTo(pipetteStepper.currentPosition() + 100000);
    while (digitalRead(LIMIT_SW_PIN) == LOW) {
      pipetteStepper.run();
      yield();
    }
    pipetteStepper.stop();
    pipetteStepper.runToPosition();
  }

  // 2. Back off UP (NEGATIVE) until switch releases (LOW)
  if (digitalRead(LIMIT_SW_PIN) == HIGH) {
    pipetteStepper.setMaxSpeed(5000.0);
    pipetteStepper.moveTo(pipetteStepper.currentPosition() - 100000);
    while (digitalRead(LIMIT_SW_PIN) == HIGH) {
      pipetteStepper.run();
      yield();
    }
    pipetteStepper.stop();
    pipetteStepper.runToPosition();
  }

  // 3. Move a bit further UP (NEGATIVE) to fully clear the switch zone
  pipetteStepper.move(-800);
  pipetteStepper.runToPosition();
  delay(80);

  // 4. Move DOWN (POSITIVE) SLOWLY to find exact switch edge
  if (digitalRead(LIMIT_SW_PIN) == LOW) {
    pipetteStepper.setMaxSpeed(200.0);
    pipetteStepper.moveTo(pipetteStepper.currentPosition() + 100000);
    while (digitalRead(LIMIT_SW_PIN) == LOW) {
      pipetteStepper.run();
      yield();
    }
    pipetteStepper.stop();
    pipetteStepper.runToPosition();
  }

  // 5. Move DOWN (POSITIVE) 130 steps past actuation = blowout point
  pipetteStepper.setMaxSpeed(200.0);
  pipetteStepper.move(130);
  pipetteStepper.runToPosition();


  // 6. Save or sync blowout position, then return to open
  if (isCalibration) {
    blowoutPosition = pipetteStepper.currentPosition();
    EEPROM.put(ADDR_BLOWOUT, blowoutPosition);
    EEPROM.commit();
  } else {
    pipetteStepper.setCurrentPosition(blowoutPosition);
    delay(500); 
    
    pipetteStepper.setMaxSpeed(savedSpeed);
    pipetteStepper.setAcceleration(savedDecel);
    
    // Go back to open position (0)
    pipetteStepper.moveTo(0);
    while(pipetteStepper.distanceToGo() != 0) {
      pipetteStepper.run();
      yield();
    }
  }

  pipetteStepper.setMaxSpeed(savedSpeed);
  pipetteStepper.setAcceleration(savedDecel);
  pipetteStepper.disableOutputs();
  
  limitSwitchTriggered = false;
  ignoreLimitSwitch = false;
}

void performDriftCheck() {
  float savedSpeed = pipetteStepper.maxSpeed();
  float savedDecel = deceleration;
  pipetteStepper.enableOutputs();
  ignoreLimitSwitch = true;
  
  for (int i = 1; i <= 20; i++) {
    pipetteStepper.setMaxSpeed(savedSpeed);
    pipetteStepper.setAcceleration(savedDecel);
    pipetteStepper.moveTo(0);
    while (pipetteStepper.distanceToGo() != 0) { pipetteStepper.run(); yield(); }
    delay(300); 
    pipetteStepper.disableOutputs(); 
    delay(10000); 
    pipetteStepper.enableOutputs();
    
    pipetteStepper.setMaxSpeed(savedSpeed);
    moveToTarget(blowoutPosition);
    
    int startState = digitalRead(LIMIT_SW_PIN);
    bool hitRecorded = false;
    
    while (pipetteStepper.distanceToGo() != 0) {
      pipetteStepper.run();
      yield();
      if (!hitRecorded && digitalRead(LIMIT_SW_PIN) != startState) {
        delayMicroseconds(500);
        if (digitalRead(LIMIT_SW_PIN) != startState) {
          hitRecorded = true;
          Serial.print("SWITCH_HIT:");
          Serial.println(pipetteStepper.currentPosition());
        }
      }
    }
    delay(300);
    
    moveToTarget(firstStopPosition);
    while (pipetteStepper.distanceToGo() != 0) { pipetteStepper.run(); yield(); }
    delay(300);
  }

  pipetteStepper.setMaxSpeed(savedSpeed);
  pipetteStepper.setAcceleration(savedDecel);
  pipetteStepper.moveTo(0);
  while(pipetteStepper.distanceToGo() != 0) { pipetteStepper.run(); yield(); } 
  pipetteStepper.disableOutputs();
  ignoreLimitSwitch = false;
}

// --- MICROSTEPPING CONTROL ---
void setMicrostepping(int divisor) {
  // DRV8825 Truth Table
  // 1 = Full, 2 = Half, 4 = Quarter, 8 = Eighth, 16 = Sixteenth, 32 = Thirty-second
  switch(divisor) {
    case 1:
      digitalWrite(M0_PIN, LOW); digitalWrite(M1_PIN, LOW); digitalWrite(M2_PIN, LOW);
      break;
    case 2:
      digitalWrite(M0_PIN, HIGH); digitalWrite(M1_PIN, LOW); digitalWrite(M2_PIN, LOW);
      break;
    case 4:
      digitalWrite(M0_PIN, LOW); digitalWrite(M1_PIN, HIGH); digitalWrite(M2_PIN, LOW);
      break;
    case 8:
      digitalWrite(M0_PIN, HIGH); digitalWrite(M1_PIN, HIGH); digitalWrite(M2_PIN, LOW);
      break;
    case 16:
      digitalWrite(M0_PIN, LOW); digitalWrite(M1_PIN, LOW); digitalWrite(M2_PIN, HIGH);
      break;
    case 32:
    default:
      digitalWrite(M0_PIN, HIGH); digitalWrite(M1_PIN, HIGH); digitalWrite(M2_PIN, HIGH);
      break;
  }
}

void setup() {
  Serial.begin(115200); 
  EEPROM.begin(512);
  
  pipetteStepper.setEnablePin(ENABLE_PIN);
  pipetteStepper.setMinPulseWidth(20);
  pipetteStepper.setPinsInverted(false, false, true); 
  pipetteStepper.disableOutputs(); 
  
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  // Set to 1/16 Microstepping on boot
  setMicrostepping(16);
  
  // --- MANUAL OLED RESET ---
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  delay(50);
  
  // 1. Read Positions from Memory
  EEPROM.get(ADDR_FIRST_STOP, firstStopPosition);
  EEPROM.get(ADDR_BLOWOUT, blowoutPosition);
  
  if (firstStopPosition == -1 || blowoutPosition == -1) {
    firstStopPosition = 0;
    blowoutPosition = 0;
  }

  // 2. Read Volume Factor from Memory
  EEPROM.get(ADDR_STEPS_PER_UL, stepsPerUL);
  if (isnan(stepsPerUL) || stepsPerUL <= 0.0 || stepsPerUL > 1000.0) {
    stepsPerUL = 4.5669;
  }

  // 3. Read Max Capacity
  EEPROM.get(ADDR_MAX_CAPACITY, maxCapacity);
  if (isnan(maxCapacity) || maxCapacity <= 0.0 || maxCapacity > 10000.0) {
    maxCapacity = 100.0;
  }

  // 5. Read Deceleration and Speed
  EEPROM.get(ADDR_DECEL, deceleration);
  if (isnan(deceleration) || deceleration <= 0.0 || deceleration > 50000.0) {
    deceleration = 5000.0;
  }
  
  float savedMaxSpeed;
  EEPROM.get(ADDR_SPEED, savedMaxSpeed);
  if (isnan(savedMaxSpeed) || savedMaxSpeed <= 0.0 || savedMaxSpeed > 50000.0) {
    savedMaxSpeed = 800.0;
  }
  pipetteStepper.setMaxSpeed(savedMaxSpeed);
  
  // 6. Read Multipoint Calibration
  calNumPoints = EEPROM.read(ADDR_CAL_NUM_POINTS);
  
  // Check if we need to initialize default curve
  bool needsInit = false;
  if (calNumPoints > 20 || calNumPoints == 0) {
    needsInit = true;
  } else {
    EEPROM.get(ADDR_CAL_POINTS, calPoints);
    // If the first element has a higher target than the second, it's the old corrupted descending array!
    if (calNumPoints >= 2 && calPoints[0].target > calPoints[1].target) {
      needsInit = true;
    }
  }

  if (needsInit) {
    // Preload default curve
    calNumPoints = 14;
    calPoints[0] = {1.0, 0.8};
    calPoints[1] = {2.5, 1.8};
    calPoints[2] = {5.0, 4.0};
    calPoints[3] = {7.5, 6.8};
    calPoints[4] = {10.0, 8.5};
    calPoints[5] = {20.0, 18.5};
    calPoints[6] = {30.0, 27.9};
    calPoints[7] = {40.0, 39.2};
    calPoints[8] = {50.0, 48.7};
    calPoints[9] = {60.0, 58.8};
    calPoints[10] = {70.0, 68.9};
    calPoints[11] = {80.0, 78.7};
    calPoints[12] = {90.0, 89.1};
    calPoints[13] = {100.0, 99.0};
    calEnabled = true;
      
    EEPROM.write(ADDR_CAL_NUM_POINTS, calNumPoints);
    EEPROM.write(ADDR_CAL_ENABLED, 1);
    EEPROM.put(ADDR_CAL_POINTS, calPoints);
    EEPROM.commit();
  } else {
    byte en = EEPROM.read(ADDR_CAL_ENABLED);
    calEnabled = (en == 1);
    EEPROM.get(ADDR_CAL_POINTS, calPoints);
  }
  
  // Set up stepper parameters
  pipetteStepper.setMaxSpeed(savedMaxSpeed); 
  pipetteStepper.setAcceleration(deceleration);

  // Limit switch: INPUT_PULLUP reads HIGH normally, LOW when switch closes to GND
  // Limit switch
  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  lastClkState = digitalRead(ENC_CLK_PIN);

  display.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_CLK, OLED_MOSI, OLED_RST);
  display.setFont(Arial14);
  display.clear();
  display.setCursor(0, 0);
  display.println(F("FluiDrop V4"));
  delay(1000);

  Serial.println("V4 Protocol Online. Awaiting commands.");
}

// --- MULTIPOINT CALIBRATION INTERPOLATION ---
float applyCalCorrection(float vol) {
  if (!calEnabled || calNumPoints == 0) return vol;

  // Since calPoints are sorted by target (we enforce this during ADD_CAL in a later step)
  // Check bounds
  if (vol <= calPoints[0].target) {
    return vol * (calPoints[0].target / calPoints[0].measured);
  }
  
  if (vol >= calPoints[calNumPoints - 1].target) {
    const float lastTarget = calPoints[calNumPoints - 1].target;
    const float lastMeasured = calPoints[calNumPoints - 1].measured;
    return vol * (lastTarget / lastMeasured);
  }
  
  // Interpolate
  for (int i = 0; i < calNumPoints - 1; i++) {
    if (vol >= calPoints[i].target && vol <= calPoints[i + 1].target) {
      float t = (vol - calPoints[i].target) / (calPoints[i + 1].target - calPoints[i].target);
      float corrA = calPoints[i].target / calPoints[i].measured;
      float corrB = calPoints[i + 1].target / calPoints[i + 1].measured;
      return vol * (corrA + t * (corrB - corrA));
    }
  }
  return vol;
}

// --- STANDARD MICRO DISPENSE (blocking, used for aliquots) ---

// --- VIGOROUS MIXING VIA 1/2 MICROSTEPPING SPEED BOOST ---
void mixVigorous(float targetVol, int times) {
  if (times <= 0) {
    Serial.println("DONE");
    return;
  }
  
  float origSpeed = pipetteStepper.maxSpeed();
  float origDecel = deceleration;
  float origStepsPerUL = stepsPerUL;
  long origCurrentPos = pipetteStepper.currentPosition();
  long origFirstStop = firstStopPosition;
  long origBlowout = blowoutPosition;
  
  // Switch stepper driver to 1/2 microstepping
  setMicrostepping(2);
  
  // Scale steps parameters down by 8 (from 1/16 to 1/2 ustepping)
  stepsPerUL = origStepsPerUL / 8.0;
  firstStopPosition = origFirstStop / 8;
  blowoutPosition = origBlowout / 8;
  pipetteStepper.setCurrentPosition(origCurrentPos / 8);
  
  // Boost max speed to 4000 steps/sec at 1/2 ustepping (which is physically 8x faster)
  // Boost acceleration to 20000 steps/sec^2 for fast reversals
  pipetteStepper.setMaxSpeed(4000.0);
  pipetteStepper.setAcceleration(20000.0);
  pipetteStepper.enableOutputs();
  
  float compVol = applyCalCorrection(targetVol);
  long stepsToMove = round(compVol * stepsPerUL);
  long aspPos = firstStopPosition - stepsToMove;
  
  // Update screen
  display.clear();
  display.setFont(Arial14);
  display.setCursor(0, 0);
  display.println("Mixing...");
  
  for (int i = 0; i < times; i++) {
    // 1. Aspirate
    pipetteStepper.moveTo(aspPos);
    while (pipetteStepper.distanceToGo() != 0) {
      pipetteStepper.run();
      yield();
    }
    delay(50);
    
    // 2. Dispense
    pipetteStepper.moveTo(firstStopPosition);
    while (pipetteStepper.distanceToGo() != 0) {
      pipetteStepper.run();
      yield();
    }
    delay(50);
  }
  
  // Restore original 1/16 microstepping
  setMicrostepping(16);
  stepsPerUL = origStepsPerUL;
  firstStopPosition = origFirstStop;
  blowoutPosition = origBlowout;
  pipetteStepper.setCurrentPosition(pipetteStepper.currentPosition() * 8);
  
  pipetteStepper.setMaxSpeed(origSpeed);
  pipetteStepper.setAcceleration(origDecel);
  pipetteStepper.disableOutputs();
  
  Serial.println("DONE");
  sendState();
}

void dispenseMicroVolume(float targetVolumeUL) {
  float compVol = applyCalCorrection(targetVolumeUL);
  long targetSteps = round(compVol * stepsPerUL);

  
  pipetteStepper.enableOutputs();
  pipetteStepper.move(targetSteps);
  pipetteStepper.runToPosition(); 
  Serial.println("DONE");
}

// --- SEND STATE PACKET ---
void sendState() {
  Serial.print("STATE:");
  Serial.print(pipetteStepper.currentPosition()); Serial.print(",");
  Serial.print(firstStopPosition); Serial.print(",");
  Serial.print(blowoutPosition); Serial.print(",");
  Serial.print(stepsPerUL, 4); Serial.print(",");
  Serial.print(maxCapacity, 1); Serial.print(",");
  Serial.print(pipetteStepper.maxSpeed(), 0); Serial.print(",");
  Serial.print(deceleration, 0); Serial.println();
}



void moveToTarget(long target) {
  pipetteStepper.moveTo(target);
  wasRunning = true;
}

void updateOLED() {
  static bool lastWasRunning = false;
  
  if (wasRunning) {
    if (!lastWasRunning) {
      lastWasRunning = true;
      display.clear();
      display.setCursor(0, 0);
      display.setFont(Arial14); 
      display.println(motorAction != "" ? motorAction : "Moving...");
      display.setFont(System5x7);
      display.println();
      display.println(F("Working..."));
    }
    return;
  }
  
  if (lastWasRunning) {
    lastWasRunning = false;
    displayNeedsUpdate = true;
  }
  
  if (!displayNeedsUpdate) {
    return;
  }
  displayNeedsUpdate = false;

  static UIMode lastUiMode = (UIMode)-1;
  static PipetteState lastState = (PipetteState)-1;

  if (uiMode != lastUiMode || pipetteState != lastState) {
    lastUiMode = uiMode;
    lastState = pipetteState;
  }
  
  display.setCursor(0, 0);
  
  if (uiMode == UI_MAIN_MENU) {
    // Row 0: Title
    display.setFont(System5x7);
    display.setCursor(0, 0);
    
    // Fill the row with inverted spaces to create a solid blue bar
    display.setInvertMode(true);
    for (int i = 0; i < 21; i++) {
      display.print(F(" "));
    }
    
    const char* title = "FLUIDROP";
    display.setCursor((128 - display.strWidth(title)) / 2, 0);
    display.print(title);
    display.setInvertMode(false);
    
    // Row 1: Blank gap
    display.setCursor(0, 1);
    display.clearToEOL();
    
    // Rows 2-7: Items
    display.setFont(Arial14);
    
    display.setCursor(0, 2);
    display.clearToEOL();
    const char* txt0 = (mainMenuIndex == 0) ? "> Variable" : "  Variable";
    display.setCursor((128 - display.strWidth(txt0)) / 2, 2);
    if (mainMenuIndex == 0) display.setInvertMode(true);
    display.print(txt0);
    display.setInvertMode(false);
    
    display.setCursor(0, 4);
    display.clearToEOL();
    const char* txt1 = (mainMenuIndex == 1) ? "> Aliquot" : "  Aliquot";
    display.setCursor((128 - display.strWidth(txt1)) / 2, 4);
    if (mainMenuIndex == 1) display.setInvertMode(true);
    display.print(txt1);
    display.setInvertMode(false);

    display.setCursor(0, 6);
    display.clearToEOL();
    const char* txt2 = (mainMenuIndex == 2) ? "> Calibrate" : "  Calibrate";
    display.setCursor((128 - display.strWidth(txt2)) / 2, 6);
    if (mainMenuIndex == 2) display.setInvertMode(true);
    display.print(txt2);
    display.setInvertMode(false);
  } 
  else if (uiMode == UI_VAR_SETUP) {
    display.setFont(Arial14);
    display.setCursor(0, 0);
    display.clearToEOL();
    const char* title = "Variable Mode";
    display.setCursor((128 - display.strWidth(title)) / 2, 0);
    display.print(title);

    if (pipetteState == STATE_READY) {
      display.setFont(System5x7);
      display.setCursor(0, 2);
      display.clearToEOL();
      const char* sub = "Set Volume (uL):";
      display.setCursor((128 - display.strWidth(sub)) / 2, 2);
      display.print(sub);
      
      display.setFont(Arial14);
      display.set2X();
      display.setCursor(0, 3);
      display.clearToEOL();
      String volStr = String(customDispenseVol, 1);
      display.setCursor((128 - (display.strWidth(volStr.c_str()) * 2)) / 2, 3);
      display.print(volStr);
      display.set1X();
    } else if (pipetteState == STATE_ASPIRATED) {
      display.setFont(System5x7);
      display.setCursor(0, 2);
      display.clearToEOL();

      display.setFont(Arial14);
      display.set2X();
      display.setCursor(0, 3);
      display.clearToEOL();
      const char* msg = "FULL";
      display.setCursor((128 - (display.strWidth(msg) * 2)) / 2, 3);
      display.print(msg);
      display.set1X();
      
      display.setCursor(0, 7);
      display.setFont(System5x7);
      display.clearToEOL();
      const char* sub = "Click to Dispense";
      display.setCursor((128 - display.strWidth(sub)) / 2, 7);
      display.print(sub);
    } else if (pipetteState == STATE_DISPENSED) {
      display.setFont(System5x7);
      display.setCursor(0, 2);
      display.clearToEOL();

      display.setFont(Arial14);
      display.set2X();
      display.setCursor(0, 3);
      display.clearToEOL();
      const char* msg = "EMPTY";
      display.setCursor((128 - (display.strWidth(msg) * 2)) / 2, 3);
      display.print(msg);
      display.set1X();
      
      display.setCursor(0, 7);
      display.setFont(System5x7);
      display.clearToEOL();
      const char* sub = "Click to Return";
      display.setCursor((128 - display.strWidth(sub)) / 2, 7);
      display.print(sub);
    }
  }
  else if (uiMode == UI_ALIQUOT_ASP_SETUP) {
    display.setFont(Arial14);
    display.setCursor(0, 0);
    display.clearToEOL();
    const char* title = "Aliquot Mode";
    display.setCursor((128 - display.strWidth(title)) / 2, 0);
    display.print(title);

    display.setFont(System5x7);
    display.setCursor(0, 2);
    display.clearToEOL();
    const char* sub = "Total Asp Vol:";
    display.setCursor((128 - display.strWidth(sub)) / 2, 2);
    display.print(sub);

    display.setFont(Arial14);
    display.set2X();
    display.setCursor(0, 3);
    display.clearToEOL();
    String volStr = String(aliquotTotalVol, 1);
    display.setCursor((128 - (display.strWidth(volStr.c_str()) * 2)) / 2, 3);
    display.print(volStr);
    display.set1X();
  }
  else if (uiMode == UI_ALIQUOT_DISP_SETUP) {
    display.setFont(Arial14);
    display.setCursor(0, 0);
    display.clearToEOL();
    const char* title = "Aliquot Mode";
    display.setCursor((128 - display.strWidth(title)) / 2, 0);
    display.print(title);

    display.setFont(System5x7);
    display.setCursor(0, 2);
    display.clearToEOL();
    const char* sub = "Dispense Vol:";
    display.setCursor((128 - display.strWidth(sub)) / 2, 2);
    display.print(sub);

    display.setFont(Arial14);
    display.set2X();
    display.setCursor(0, 3);
    display.clearToEOL();
    String volStr = String(aliquotDispVol, 1);
    display.setCursor((128 - (display.strWidth(volStr.c_str()) * 2)) / 2, 3);
    display.print(volStr);
    display.set1X();
  }
  else if (uiMode == UI_RUNNING) {
    display.setFont(Arial14);
    display.setCursor(0, 0);
    display.clearToEOL();
    const char* title = "Aliquot Mode";
    display.setCursor((128 - display.strWidth(title)) / 2, 0);
    display.print(title);

    if (pipetteState == STATE_READY) {
      display.setFont(System5x7);
      display.setCursor(0, 2);
      display.clearToEOL();
      const char* sub = "Ready. Total:";
      display.setCursor((128 - display.strWidth(sub)) / 2, 2);
      display.print(sub);

      display.setFont(Arial14);
      display.set2X();
      display.setCursor(0, 3);
      display.clearToEOL();
      String volStr = String(aliquotTotalVol, 1);
      display.setCursor((128 - (display.strWidth(volStr.c_str()) * 2)) / 2, 3);
      display.print(volStr);
      display.set1X();
    } else if (pipetteState == STATE_ASPIRATED) {
      display.setFont(System5x7);
      display.setCursor(0, 2);
      display.clearToEOL();
      const char* sub = "Remaining (uL):";
      display.setCursor((128 - display.strWidth(sub)) / 2, 2);
      display.print(sub);

      display.setFont(Arial14);
      display.set2X();
      display.setCursor(0, 3);
      display.clearToEOL();
      String volStr = String(aliquotRemainingVol, 1);
      display.setCursor((128 - (display.strWidth(volStr.c_str()) * 2)) / 2, 3);
      display.print(volStr);
      display.set1X();
      
      display.setCursor(0, 7);
      display.setFont(System5x7);
      display.clearToEOL();
      const char* btn = "Click to Aliquot";
      display.setCursor((128 - display.strWidth(btn)) / 2, 7);
      display.print(btn);
    } else if (pipetteState == STATE_EMPTY) {
      display.setFont(System5x7);
      display.setCursor(0, 2);
      display.clearToEOL();

      display.setFont(Arial14);
      display.set2X();
      display.setCursor(0, 3);
      display.clearToEOL();
      const char* msg = "EMPTY";
      display.setCursor((128 - (display.strWidth(msg) * 2)) / 2, 3);
      display.print(msg);
      display.set1X();
      
      display.setCursor(0, 7);
      display.setFont(System5x7);
      display.clearToEOL();
      const char* btn = "Click to Blowout";
      display.setCursor((128 - display.strWidth(btn)) / 2, 7);
      display.print(btn);
    } else if (pipetteState == STATE_DISPENSED) {
      display.setFont(System5x7);
      display.setCursor(0, 2);
      display.clearToEOL();

      display.setFont(Arial14);
      display.set2X();
      display.setCursor(0, 3);
      display.clearToEOL();
      const char* msg = "PURGED";
      display.setCursor((128 - (display.strWidth(msg) * 2)) / 2, 3);
      display.print(msg);
      display.set1X();
      
      display.setCursor(0, 7);
      display.setFont(System5x7);
      display.clearToEOL();
      const char* btn = "Click to Return";
      display.setCursor((128 - display.strWidth(btn)) / 2, 7);
      display.print(btn);
    }
  }
  
  if (uiMode != UI_MAIN_MENU) {
    if (pipetteState == STATE_READY || uiMode == UI_ALIQUOT_ASP_SETUP || uiMode == UI_ALIQUOT_DISP_SETUP) {
      display.setCursor(0, 7);
      display.setFont(System5x7);
      display.clearToEOL();
    }
  }
}

void handleEncoder() {
  int clkState = digitalRead(ENC_CLK_PIN);
  if (clkState != lastClkState && clkState == LOW) { 
    bool isUp = (digitalRead(ENC_DT_PIN) != clkState);
    if (uiMode == UI_MAIN_MENU) {
      if (isUp) {
        mainMenuIndex--;
        if (mainMenuIndex < 0) mainMenuIndex = 2;
      } else {
        mainMenuIndex++;
        if (mainMenuIndex > 2) mainMenuIndex = 0;
      }
    } else if (uiMode == UI_VAR_SETUP) {
      if (!isUp) {
        customDispenseVol -= 1.0;
        if (customDispenseVol < 0.1) customDispenseVol = 0.1;
      } else {
        customDispenseVol += 1.0;
        if (customDispenseVol > maxCapacity) customDispenseVol = maxCapacity;
      }
    } else if (uiMode == UI_ALIQUOT_ASP_SETUP) {
      if (!isUp) aliquotTotalVol -= 1.0;
      else aliquotTotalVol += 1.0;
      if (aliquotTotalVol < 1.0) aliquotTotalVol = 1.0;
      if (aliquotTotalVol > maxCapacity) aliquotTotalVol = maxCapacity;
    } else if (uiMode == UI_ALIQUOT_DISP_SETUP) {
      if (!isUp) aliquotDispVol -= 1.0;
      else aliquotDispVol += 1.0;
      if (aliquotDispVol < 0.1) aliquotDispVol = 0.1;
      if (aliquotDispVol > aliquotTotalVol) aliquotDispVol = aliquotTotalVol;
    }
    displayNeedsUpdate = true;
  }
  lastClkState = clkState;
  
  // Long press logic
  int swState = digitalRead(ENC_SW_PIN);
  if (swState == LOW) {
    if (!btnIsPressed && (millis() - lastButtonPress > debounceDelay)) {
      btnIsPressed = true;
      btnPressTime = millis();
      longPressHandled = false;
    }
    if (btnIsPressed && !longPressHandled && (millis() - btnPressTime > 800)) {
      longPressHandled = true;
      if (uiMode != UI_MAIN_MENU && !wasRunning) {
        uiMode = UI_MAIN_MENU; // Back to main menu
        pipetteState = STATE_READY;
        displayNeedsUpdate = true;
      }
    }
  } else {
    if (btnIsPressed) {
      btnIsPressed = false;
      lastButtonPress = millis();
      if (!longPressHandled && !wasRunning) { // Short click
        if (uiMode == UI_MAIN_MENU) {
          if (mainMenuIndex == 0) {
            uiMode = UI_VAR_SETUP;
            pipetteState = STATE_READY;
          } else if (mainMenuIndex == 1) {
            uiMode = UI_ALIQUOT_ASP_SETUP;
            pipetteState = STATE_READY;
          } else if (mainMenuIndex == 2) {
            display.clear();
            display.setFont(System5x7);
            display.set2X();
            display.setCursor(0, 0);
            const char* title = "CALIBRATE";
            display.setCursor((128 - (display.strWidth(title) * 2)) / 2, display.row());
            display.print(title);
            display.set1X();
            
            display.setCursor(0, 4);
            const char* sub = "(AUTO HOMING)";
            display.setCursor((128 - display.strWidth(sub)) / 2, display.row());
            display.print(sub);
            
            performAutoHome(true);
            displayNeedsUpdate = true;
          }
        } else if (uiMode == UI_ALIQUOT_ASP_SETUP) {
          uiMode = UI_ALIQUOT_DISP_SETUP;
        } else if (uiMode == UI_ALIQUOT_DISP_SETUP) {
          uiMode = UI_RUNNING;
          pipetteState = STATE_READY;
        } else if (uiMode == UI_VAR_SETUP) {
          if (pipetteState == STATE_READY) {
            float compVol = applyCalCorrection(customDispenseVol);
            long stepsToMove = round(compVol * stepsPerUL);
            moveToTarget(firstStopPosition - stepsToMove);
            wasRunning = true;
            motorAction = "Aspirating...";
            pipetteState = STATE_ASPIRATED;
          } else if (pipetteState == STATE_ASPIRATED) {
            pipetteStepper.enableOutputs();
            moveToTarget(blowoutPosition);
            wasRunning = true;
            motorAction = "Dispensing...";
            pipetteState = STATE_DISPENSED;
          } else if (pipetteState == STATE_DISPENSED) {
            moveToTarget(firstStopPosition);
            wasRunning = true;
            motorAction = "Returning...";
            pipetteState = STATE_READY;
          }
        } else if (uiMode == UI_RUNNING) {
          if (pipetteState == STATE_READY) {
            float compVol = applyCalCorrection(aliquotTotalVol);
            long stepsToMove = round(compVol * stepsPerUL);
            moveToTarget(firstStopPosition - stepsToMove);
            wasRunning = true;
            motorAction = "Aspirating...";
            pipetteState = STATE_ASPIRATED;
            aliquotRemainingVol = aliquotTotalVol;
          } else if (pipetteState == STATE_ASPIRATED) {
            float compVol = applyCalCorrection(aliquotDispVol);
            long targetSteps = round(compVol * stepsPerUL);
            pipetteStepper.enableOutputs();
            pipetteStepper.move(targetSteps);
            wasRunning = true;
            motorAction = "Dispensing...";
            aliquotRemainingVol -= aliquotDispVol;
            if (aliquotRemainingVol < (aliquotDispVol - 0.05)) { 
               pipetteState = STATE_EMPTY;
            }
          } else if (pipetteState == STATE_EMPTY) {
            pipetteStepper.enableOutputs();
            moveToTarget(blowoutPosition);
            wasRunning = true;
            motorAction = "Purging...";
            pipetteState = STATE_DISPENSED;
          } else if (pipetteState == STATE_DISPENSED) {
            moveToTarget(firstStopPosition);
            wasRunning = true;
            motorAction = "Returning...";
            pipetteState = STATE_READY;
            uiMode = UI_ALIQUOT_ASP_SETUP; 
          }
        }
        displayNeedsUpdate = true;
      }
    }
  }
}

unsigned long lastSwitchLog = 0;

void loop() {
  // Log limit switch state every 500ms for debugging
  // if (millis() - lastSwitchLog > 500) {
  //   lastSwitchLog = millis();
  //   Serial.print("SW:");
  //   Serial.println(digitalRead(LIMIT_SW_PIN) == HIGH ? "PRESSED" : "OPEN");
  // }
  handleEncoder();
  updateOLED();
  if (Serial.available() > 0) {
    appCommand = Serial.readStringUntil('\n'); 
    appCommand.trim(); 
    
    Serial.print("Executing: ");
    Serial.println(appCommand);

    // ==========================================
    // 1. DIAGNOSTIC & SETTINGS QUERIES
    // ==========================================
    if (appCommand.startsWith("SET_MICRO:")) {
      int ms = appCommand.substring(10).toInt();
      setMicrostepping(ms);
      Serial.print("MICROSTEPPING_SET:");
      Serial.println(ms);
    }
    else if (appCommand.startsWith("SPEED:")) {
      float newSpeed = appCommand.substring(6).toFloat();
      pipetteStepper.setMaxSpeed(newSpeed);
      EEPROM.put(ADDR_SPEED, newSpeed);
      EEPROM.commit();
      Serial.print("MAX_SPEED_SET:");
      Serial.println(newSpeed);
    }
    else if (appCommand == "GET_SPEED") {
      Serial.print("MAX_SPEED:");
      Serial.println(pipetteStepper.maxSpeed());
    }
    else if (appCommand.startsWith("DECEL:")) {
      deceleration = appCommand.substring(6).toFloat();
      pipetteStepper.setAcceleration(deceleration);
      EEPROM.put(ADDR_DECEL, deceleration);
      EEPROM.commit();
      Serial.print("DECEL_SET:");
      Serial.println(deceleration);
    }
    else if (appCommand.startsWith("SET_FACTOR:")) {
      stepsPerUL = appCommand.substring(11).toFloat();
      EEPROM.put(ADDR_STEPS_PER_UL, stepsPerUL);
      EEPROM.commit();
      Serial.print("FACTOR_SAVED:");
      Serial.println(stepsPerUL);
    }
    else if (appCommand == "GET_FACTOR") {
      Serial.print("FACTOR:");
      Serial.println(stepsPerUL);
    }
    else if (appCommand.startsWith("SET_CAPACITY:")) {
      float cap = appCommand.substring(13).toFloat();
      if (cap > 0) {
        maxCapacity = cap;
        // Recalculate stepsPerUL from the new capacity and current stroke
        long totalSteps = abs(firstStopPosition - homePosition);
        if (totalSteps > 1000 && maxCapacity > 0) {
          stepsPerUL = (float)totalSteps / maxCapacity;
          EEPROM.put(ADDR_STEPS_PER_UL, stepsPerUL);
        }
        EEPROM.put(ADDR_MAX_CAPACITY, maxCapacity);
        EEPROM.commit();
        Serial.print("CAPACITY_SAVED:");
        Serial.println(maxCapacity);
        Serial.print("FACTOR_SAVED:");
        Serial.println(stepsPerUL);
      }
    }
    else if (appCommand == "GET_STATE") {
      sendState();
    }

    // ==========================================
    // 2. JOG COMMANDS 
    // ==========================================
    else if (appCommand == "FWD") {
      
      pipetteStepper.move(1000000); 
      wasRunning = true;
    } 
    else if (appCommand == "REV") {
      
      pipetteStepper.move(-1000000); 
      wasRunning = true;
    } 
    else if (appCommand == "STOP") {
      pipetteStepper.stop();
      wasRunning = false;
      pipetteStepper.disableOutputs();
      
      // Emit DONE so UI unlocks after an emergency stop
      Serial.println("DONE");
      sendState();
    }

    // ==========================================
    // 3. PIPETTE OPERATIONS (async - DONE emitted by loop)
    // ==========================================
    else if (appCommand == "PREP") {
      moveToTarget(firstStopPosition);
      wasRunning = true;
    }
    else if (appCommand == "ASP") {
      moveToTarget(homePosition);
      wasRunning = true;
    }
    else if (appCommand == "BLOWOUT") {
      moveToTarget(blowoutPosition);
      wasRunning = true;
    }
    else if (appCommand == "DISP") {
      pipetteStepper.enableOutputs();
      
      pipetteStepper.moveTo(blowoutPosition);
      
      int startState = digitalRead(LIMIT_SW_PIN);
      bool hitRecorded = false;
      
      while (pipetteStepper.distanceToGo() != 0) {
        pipetteStepper.run();
        yield();
        if (!hitRecorded && digitalRead(LIMIT_SW_PIN) != startState) {
          switchHitPosition = pipetteStepper.currentPosition();
          hitRecorded = true;
          Serial.print("SWITCH_HIT:");
          Serial.println(switchHitPosition);
        }
      }
      
      if (hitRecorded && blowoutPosition != 0) {
        long expectedHit = blowoutPosition - blowoutOffset;
        long drift = switchHitPosition - expectedHit;
        if (drift != 0) {
          pipetteStepper.setCurrentPosition(pipetteStepper.currentPosition() - drift);
          Serial.print("DRIFT_CORRECTED:");
          Serial.println(drift);
        }
      }
      
      delay(300);
      pipetteStepper.disableOutputs();
      Serial.println("DONE");
      sendState();
    }
    else if (appCommand == "PREWET") {
      pipetteStepper.enableOutputs();
      for (int i=0; i<3; i++) {
        moveToTarget(firstStopPosition);
        pipetteStepper.runToPosition();
        delay(200);
        moveToTarget(homePosition);
        pipetteStepper.runToPosition();
        delay(200);
      }
      
      // Blowout
      moveToTarget(blowoutPosition);
      
      int startState = digitalRead(LIMIT_SW_PIN);
      bool hitRecorded = false;
      
      while (pipetteStepper.distanceToGo() != 0) {
        pipetteStepper.run();
        yield();
        if (!hitRecorded && digitalRead(LIMIT_SW_PIN) != startState) {
          switchHitPosition = pipetteStepper.currentPosition();
          hitRecorded = true;
          Serial.print("SWITCH_HIT:");
          Serial.println(switchHitPosition);
        }
      }
      
      if (hitRecorded && blowoutPosition != 0) {
        long expectedHit = blowoutPosition - blowoutOffset;
        long drift = switchHitPosition - expectedHit;
        if (drift != 0) {
          pipetteStepper.setCurrentPosition(pipetteStepper.currentPosition() - drift);
          Serial.print("DRIFT_CORRECTED:");
          Serial.println(drift);
        }
      }
      
      delay(300);
      moveToTarget(firstStopPosition);
      pipetteStepper.runToPosition();
      delay(200);
      
      pipetteStepper.disableOutputs();
      Serial.println("DONE");
      sendState();
    }
    else if (appCommand == "MICRO_1UL") {
      dispenseMicroVolume(1.0);
    }
    else if (appCommand.startsWith("MIX_VIGOROUS:")) {
      int commaIdx = appCommand.indexOf(',');
      if (commaIdx != -1) {
        float vol = appCommand.substring(13, commaIdx).toFloat();
        int times = appCommand.substring(commaIdx + 1).toInt();
        mixVigorous(vol, times);
      }
    }
    
    // CUSTOM ALIQUOT DISPENSE (Relative Move, single-value format)
    else if (appCommand.startsWith("DISP_VOL:")) {
      float targetVol = appCommand.substring(9).toFloat();
      if (targetVol > 0) {
        dispenseMicroVolume(targetVol);
      }
    }
    
    // CUSTOM ABSOLUTE ASPIRATE (Moves to absolute coordinate)
    else if (appCommand.startsWith("VOL:")) {
      float targetVol = appCommand.substring(4).toFloat();
      if (targetVol >= 0) {
        float compVol = applyCalCorrection(targetVol);
        long stepsToMove = round(compVol * stepsPerUL);
        long targetCoordinate = firstStopPosition - stepsToMove; 
        moveToTarget(targetCoordinate);
        wasRunning = true;
      }
    }

    // ==========================================
    // 4. CALIBRATION COMMANDS 
    // ==========================================
    else if (appCommand.startsWith("SET_CAL_ENABLE:")) {
      int val = appCommand.substring(15).toInt();
      calEnabled = (val == 1);
      EEPROM.write(ADDR_CAL_ENABLED, calEnabled ? 1 : 0);
      // Save the full array and count when they hit Save (which always sends this command)
      EEPROM.write(ADDR_CAL_NUM_POINTS, calNumPoints);
      EEPROM.put(ADDR_CAL_POINTS, calPoints);
      EEPROM.commit();
      Serial.print("CAL_ENABLE_SET:");
      Serial.println(calEnabled);
    }
    else if (appCommand == "CLEAR_CAL") {
      calNumPoints = 0;
      EEPROM.write(ADDR_CAL_NUM_POINTS, calNumPoints);
      EEPROM.commit();
      Serial.println("CAL_CLEARED");
    }
    else if (appCommand.startsWith("ADD_CAL:")) {
      String data = appCommand.substring(8);
      int commaIdx = data.indexOf(',');
      if (commaIdx > 0 && calNumPoints < 20) {
        float t = data.substring(0, commaIdx).toFloat();
        float m = data.substring(commaIdx + 1).toFloat();
        
        calPoints[calNumPoints].target = t;
        calPoints[calNumPoints].measured = m;
        calNumPoints++;
        
        // Sort the array by target volume (bubble sort is fine for 20 elements)
        for (int i = 0; i < calNumPoints - 1; i++) {
          for (int j = 0; j < calNumPoints - i - 1; j++) {
            if (calPoints[j].target > calPoints[j + 1].target) {
              CalPoint temp = calPoints[j];
              calPoints[j] = calPoints[j + 1];
              calPoints[j + 1] = temp;
            }
          }
        }
        
        // Removed EEPROM writes here to prevent Serial RX buffer overflow from slow writes
        Serial.println("CAL_ADDED");
      }
    }
    else if (appCommand == "GET_CAL") {
      Serial.print("CAL_DATA:");
      Serial.print(calEnabled ? "ON" : "OFF");
      for (int i = 0; i < calNumPoints; i++) {
        Serial.print(",");
        Serial.print(calPoints[i].target, 2);
        Serial.print(":");
        Serial.print(calPoints[i].measured, 2);
      }
      Serial.println();
    }
    else if (appCommand == "ZERO") {
      long oldPos = pipetteStepper.currentPosition();
      homePosition = 0;
      pipetteStepper.setCurrentPosition(0);
      
      firstStopPosition -= oldPos;
      blowoutPosition -= oldPos;
      
      if (maxCapacity > 0) {
        long totalSteps = abs(firstStopPosition - homePosition);
        if (totalSteps > 1000) {
          stepsPerUL = (float)totalSteps / maxCapacity;
          EEPROM.put(ADDR_STEPS_PER_UL, stepsPerUL);
        }
      }
      
      EEPROM.put(ADDR_FIRST_STOP, firstStopPosition);
      EEPROM.put(ADDR_BLOWOUT, blowoutPosition);
      EEPROM.commit();
      sendState();
    }
    else if (appCommand == "SET1") {
      firstStopPosition = pipetteStepper.currentPosition();
      
      if (maxCapacity > 0) {
        long totalSteps = abs(firstStopPosition - homePosition);
        if (totalSteps > 1000) {
          stepsPerUL = (float)totalSteps / maxCapacity;
          EEPROM.put(ADDR_STEPS_PER_UL, stepsPerUL);
        }
      }
      
      EEPROM.put(ADDR_FIRST_STOP, firstStopPosition); 
      EEPROM.commit();
      sendState();
    }
    else if (appCommand == "SET2") {
      blowoutPosition = pipetteStepper.currentPosition();
      EEPROM.put(ADDR_BLOWOUT, blowoutPosition); 
      EEPROM.commit();
      sendState();
    }
    // Set how many steps past the limit switch to treat as blowout
    else if (appCommand.startsWith("SET2_OFFSET:")) {
      blowoutOffset = appCommand.substring(12).toInt();
      Serial.print("Blowout offset set to: ");
      Serial.println(blowoutOffset);
    }
    // Auto-home the blowout position using limit switch on D2
    // 3D-printer style: fast bump â†’ back off â†’ slow precise creep â†’ offset
    else if (appCommand == "AUTO_BLOWOUT" || appCommand == "AUTO_HOME") {
      bool isCalibration = (appCommand == "AUTO_BLOWOUT");
      performAutoHome(isCalibration);
      Serial.println("DONE");
      sendState();
    }
    else if (appCommand == "DRIFT_CHECK") {
      performDriftCheck();
      Serial.println("DONE");
      sendState();
    }
    else if (appCommand == "RET_ZERO") {
      moveToTarget(homePosition);
      wasRunning = true;
    }
    else if (appCommand.startsWith("SYNC_POS:")) {
      int idx = appCommand.substring(9).toInt();
      if (idx == 0) {
        pipetteStepper.setCurrentPosition(homePosition);
      } else if (idx == 1) {
        pipetteStepper.setCurrentPosition(firstStopPosition);
      } else if (idx == 2) {
        pipetteStepper.setCurrentPosition(blowoutPosition);
      }
    }
  }

  // --- ASYNC MOVEMENT LOOP ---
  if (pipetteStepper.distanceToGo() != 0) {
    pipetteStepper.enableOutputs(); 
    pipetteStepper.run();
    wasRunning = true;
  } else {
    if (wasRunning) {
      wasRunning = false;
      pipetteStepper.disableOutputs();
      Serial.println("DONE");
      sendState();
    } else {
      if (!pipetteStepper.isRunning()) {
        pipetteStepper.disableOutputs(); 
      }
    }
  }
}
