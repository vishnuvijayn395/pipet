#include <AccelStepper.h>
#include <EEPROM.h>

const int STEP_PIN = 5;
const int DIR_PIN = 4;
const int ENABLE_PIN = 11;
const int LIMIT_SW_PIN = 2;   // Blowout limit switch on D2 (INT0)
                               // Wiring: one leg → GND, other leg → D2. No external resistor needed.

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

// Limit switch — set by ISR, cleared in loop after handling
volatile bool limitSwitchTriggered = false;
volatile bool ignoreLimitSwitch = false;
volatile long switchHitPosition = 0;
int blowoutOffset = 80;   // Steps to travel past switch before saving blowout position

// We will use inline polling instead of an ISR to track the switch hit position
// accurately without worrying about interrupt edges and bounce.


void performAutoHome(bool isCalibration) {
  Serial.println(isCalibration ? "AUTO_BLOWOUT: Homing started..." : "AUTO_HOME: Homing started...");
  float savedSpeed = pipetteStepper.maxSpeed();
  float savedDecel = deceleration;
  pipetteStepper.enableOutputs();
  
  pipetteStepper.setMaxSpeed(800.0);
  pipetteStepper.setAcceleration(5000.0);
  
  ignoreLimitSwitch = true; // Disable ISR completely during homing

  // PASS 0: If starting already pressed, back off until released
  if (digitalRead(LIMIT_SW_PIN) == HIGH) {
    Serial.println("Switch already pressed, backing off...");
    pipetteStepper.setMaxSpeed(800.0);
    pipetteStepper.moveTo(pipetteStepper.currentPosition() - 100000);
    while (digitalRead(LIMIT_SW_PIN) == HIGH) {
      pipetteStepper.run();
    }
    pipetteStepper.stop();
    pipetteStepper.runToPosition();
  }

  // PASS 1: Fast approach
  if (digitalRead(LIMIT_SW_PIN) == LOW) {
    Serial.println("Pass 1 - fast approach...");
    pipetteStepper.setMaxSpeed(800.0);
    pipetteStepper.moveTo(pipetteStepper.currentPosition() + 100000);
    while (digitalRead(LIMIT_SW_PIN) == LOW) {
      pipetteStepper.run();
    }
    pipetteStepper.stop();
    pipetteStepper.runToPosition();
  }

  // BACK OFF: Ensure switch is fully released
  if (digitalRead(LIMIT_SW_PIN) == HIGH) {
    Serial.println("Switch hit, releasing...");
    pipetteStepper.setMaxSpeed(800.0);
    pipetteStepper.moveTo(pipetteStepper.currentPosition() - 100000);
    while (digitalRead(LIMIT_SW_PIN) == HIGH) {
      pipetteStepper.run();
    }
    pipetteStepper.stop();
    pipetteStepper.runToPosition();
  }

  // Hysteresis clearance
  pipetteStepper.move(-100);
  pipetteStepper.runToPosition();
  delay(80);

  // PASS 2: Slow precise creep
  if (digitalRead(LIMIT_SW_PIN) == LOW) {
    Serial.println("Pass 2 - slow precise approach...");
    pipetteStepper.setMaxSpeed(60.0);
    pipetteStepper.moveTo(pipetteStepper.currentPosition() + 100000);
    while (digitalRead(LIMIT_SW_PIN) == LOW) {
      pipetteStepper.run();
    }
    pipetteStepper.stop();
    pipetteStepper.runToPosition();
  }

  // OVERSHOOT OFFSET: move N steps past the switch trigger point
  Serial.println("Applying offset...");
  pipetteStepper.setMaxSpeed(60.0);
  pipetteStepper.move(blowoutOffset);
  pipetteStepper.runToPosition();

  if (isCalibration) {
    // CALIBRATION: Save current physical position as blowout position
    blowoutPosition = pipetteStepper.currentPosition();
    EEPROM.put(ADDR_BLOWOUT, blowoutPosition);
    Serial.print("AUTO_BLOWOUT complete. Position saved at step ");
    Serial.println(blowoutPosition);
  } else {
    // NORMAL HOMING: Restore coordinate system from EEPROM
    pipetteStepper.setCurrentPosition(blowoutPosition);
    
    Serial.print("AUTO_HOME complete. Coordinates restored to blowout: ");
    Serial.println(blowoutPosition);
    
    delay(500); // Let inertia settle before reversing direction
    
    Serial.println("AUTO_HOME: Moving to Open Position...");
    pipetteStepper.setMaxSpeed(savedSpeed);
    pipetteStepper.setAcceleration(savedDecel);
    
    pipetteStepper.moveTo(0);
    while(pipetteStepper.distanceToGo() != 0) {
      pipetteStepper.run();
    }
  }

  pipetteStepper.setMaxSpeed(savedSpeed);
  pipetteStepper.setAcceleration(savedDecel);
  pipetteStepper.disableOutputs();
  
  limitSwitchTriggered = false;
  ignoreLimitSwitch = false;
}

void performDriftCheck() {
  Serial.println("DRIFT_CHECK: Starting 20x loop...");
  float savedSpeed = pipetteStepper.maxSpeed();
  float savedDecel = deceleration;
  pipetteStepper.enableOutputs();
  ignoreLimitSwitch = true;
  
  for (int i = 1; i <= 20; i++) {
    // 1. Move to home/open position (0)
    pipetteStepper.setMaxSpeed(savedSpeed);
    pipetteStepper.setAcceleration(savedDecel);
    pipetteStepper.moveTo(0);
    while (pipetteStepper.distanceToGo() != 0) {
      pipetteStepper.run();
      yield();
    }
    delay(300); // Hold at open position for 0.3 seconds with torque engaged
    pipetteStepper.disableOutputs(); // Disable holding torque
    delay(10000); // Wait 10 seconds without holding torque
    pipetteStepper.enableOutputs(); // Re-enable for the next move
    
    // 2. Move to blowout position (simulate normal dispense stroke)
    pipetteStepper.setMaxSpeed(savedSpeed);
    pipetteStepper.moveTo(blowoutPosition);
    
    int startState = digitalRead(LIMIT_SW_PIN);
    bool hitRecorded = false;
    
    while (pipetteStepper.distanceToGo() != 0) {
      pipetteStepper.run();
      if (!hitRecorded && digitalRead(LIMIT_SW_PIN) != startState) {
        // Debounce
        delayMicroseconds(500);
        if (digitalRead(LIMIT_SW_PIN) != startState) {
          long hitPos = pipetteStepper.currentPosition();
          hitRecorded = true;
          Serial.print("DRIFT_CHECK Iteration ");
          Serial.print(i);
          Serial.print(": Switch triggered at step = ");
          Serial.println(hitPos);
        }
      }
      yield();
    }
    
    // Hold at blowout point for 0.3s
    delay(300);
    
    // 3. Return to 1st stop
    pipetteStepper.moveTo(firstStopPosition);
    while (pipetteStepper.distanceToGo() != 0) {
      pipetteStepper.run();
      yield();
    }
    
    // Hold at 1st stop for 0.3s
    delay(300);
  }

  // Restore and return
  pipetteStepper.setMaxSpeed(savedSpeed);
  pipetteStepper.setAcceleration(savedDecel);
  pipetteStepper.moveTo(0);
  while(pipetteStepper.distanceToGo() != 0) { 
    pipetteStepper.run(); 
    yield();
  }
  pipetteStepper.disableOutputs();
  ignoreLimitSwitch = false;
  Serial.println("DRIFT_CHECK: Complete.");
}

void setup() {
  Serial.begin(115200); 
  
  pipetteStepper.setEnablePin(ENABLE_PIN);
  pipetteStepper.setPinsInverted(false, false, true); 
  pipetteStepper.disableOutputs(); 
  
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
      
    EEPROM.update(ADDR_CAL_NUM_POINTS, calNumPoints);
    EEPROM.update(ADDR_CAL_ENABLED, 1);
    EEPROM.put(ADDR_CAL_POINTS, calPoints);
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
void dispenseMicroVolume(float targetVolumeUL) {
  float compVol = applyCalCorrection(targetVolumeUL);
  long targetSteps = round(compVol * stepsPerUL);

  pipetteStepper.enableOutputs();

  // Pure relative forward move
  pipetteStepper.move(targetSteps);
  pipetteStepper.runToPosition(); 
  
  Serial.print("Dispensed ");
  Serial.print(targetVolumeUL);
  Serial.println(" uL.");
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

void loop() {
  if (Serial.available() > 0) {
    appCommand = Serial.readStringUntil('\n'); 
    appCommand.trim(); 
    
    Serial.print("Executing: ");
    Serial.println(appCommand);

    // ==========================================
    // 1. DIAGNOSTIC & SETTINGS QUERIES
    // ==========================================
    if (appCommand.startsWith("SPEED:")) {
      float newSpeed = appCommand.substring(6).toFloat();
      pipetteStepper.setMaxSpeed(newSpeed);
      EEPROM.put(ADDR_SPEED, newSpeed);
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
      Serial.print("DECEL_SET:");
      Serial.println(deceleration);
    }
    else if (appCommand.startsWith("SET_FACTOR:")) {
      stepsPerUL = appCommand.substring(11).toFloat();
      EEPROM.put(ADDR_STEPS_PER_UL, stepsPerUL);
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
        if (totalSteps > 0 && maxCapacity > 0) {
          stepsPerUL = (float)totalSteps / maxCapacity;
          EEPROM.put(ADDR_STEPS_PER_UL, stepsPerUL);
        }
        EEPROM.put(ADDR_MAX_CAPACITY, maxCapacity);
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
    } 
    else if (appCommand == "REV") {
      pipetteStepper.move(-1000000); 
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
      pipetteStepper.moveTo(firstStopPosition);
      wasRunning = true;
    }
    else if (appCommand == "ASP") {
      pipetteStepper.moveTo(homePosition);
      wasRunning = true;
    }
    else if (appCommand == "BLOWOUT") {
      pipetteStepper.moveTo(blowoutPosition);
      wasRunning = true;
    }
    else if (appCommand == "DISP") {
      pipetteStepper.enableOutputs();
      pipetteStepper.moveTo(blowoutPosition);
      
      int startState = digitalRead(LIMIT_SW_PIN);
      bool hitRecorded = false;
      
      while (pipetteStepper.distanceToGo() != 0) {
        pipetteStepper.run();
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
        pipetteStepper.moveTo(firstStopPosition);
        pipetteStepper.runToPosition();
        delay(200);
        pipetteStepper.moveTo(homePosition);
        pipetteStepper.runToPosition();
        delay(200);
      }
      
      // Blowout
      pipetteStepper.moveTo(blowoutPosition);
      
      int startState = digitalRead(LIMIT_SW_PIN);
      bool hitRecorded = false;
      
      while (pipetteStepper.distanceToGo() != 0) {
        pipetteStepper.run();
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
      pipetteStepper.moveTo(firstStopPosition);
      pipetteStepper.runToPosition();
      delay(200);
      
      pipetteStepper.disableOutputs();
      Serial.println("DONE");
      sendState();
    }
    else if (appCommand == "MICRO_1UL") {
      dispenseMicroVolume(1.0);
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
        pipetteStepper.moveTo(targetCoordinate);
        wasRunning = true;
      }
    }

    // ==========================================
    // 4. CALIBRATION COMMANDS 
    // ==========================================
    else if (appCommand.startsWith("SET_CAL_ENABLE:")) {
      int val = appCommand.substring(15).toInt();
      calEnabled = (val == 1);
      EEPROM.update(ADDR_CAL_ENABLED, calEnabled ? 1 : 0);
      // Save the full array and count when they hit Save (which always sends this command)
      EEPROM.update(ADDR_CAL_NUM_POINTS, calNumPoints);
      EEPROM.put(ADDR_CAL_POINTS, calPoints);
      Serial.print("CAL_ENABLE_SET:");
      Serial.println(calEnabled);
    }
    else if (appCommand == "CLEAR_CAL") {
      calNumPoints = 0;
      EEPROM.update(ADDR_CAL_NUM_POINTS, calNumPoints);
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
      
      // Shift all saved coordinates by oldPos so they remain physically accurate!
      firstStopPosition -= oldPos;
      blowoutPosition -= oldPos;
      
      EEPROM.put(ADDR_FIRST_STOP, firstStopPosition);
      EEPROM.put(ADDR_BLOWOUT, blowoutPosition);
      
      Serial.print("Calibration: Open Position (Top) locked at 0. Shifted by ");
      Serial.println(-oldPos);
      sendState();
    }
    else if (appCommand == "SET1") {
      firstStopPosition = pipetteStepper.currentPosition();
      EEPROM.put(ADDR_FIRST_STOP, firstStopPosition); 
      Serial.print("Calibration: 1st Stop SAVED at ");
      Serial.println(firstStopPosition);
    }
    else if (appCommand == "SET2") {
      blowoutPosition = pipetteStepper.currentPosition();
      EEPROM.put(ADDR_BLOWOUT, blowoutPosition); 
      Serial.print("Calibration: 2nd Stop (Blowout) SAVED at ");
      Serial.println(blowoutPosition);
    }
    // Set how many steps past the limit switch to treat as blowout
    else if (appCommand.startsWith("SET2_OFFSET:")) {
      blowoutOffset = appCommand.substring(12).toInt();
      Serial.print("Blowout offset set to: ");
      Serial.println(blowoutOffset);
    }
    // Auto-home the blowout position using limit switch on D2
    // 3D-printer style: fast bump → back off → slow precise creep → offset
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
      pipetteStepper.moveTo(homePosition);
      wasRunning = true;
    }
    else if (appCommand.startsWith("SYNC_POS:")) {
      // UI syncs the firmware's knowledge of where calibrated stops are
      int idx = appCommand.substring(9).toInt();
      if (idx == 0) {
        pipetteStepper.setCurrentPosition(homePosition);
        Serial.println("Synced to Home");
      } else if (idx == 1) {
        pipetteStepper.setCurrentPosition(firstStopPosition);
        Serial.println("Synced to 1st Stop");
      } else if (idx == 2) {
        pipetteStepper.setCurrentPosition(blowoutPosition);
        Serial.println("Synced to Blowout");
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
      // Motor just stopped: emit DONE once so the UI unlocks isMoving
      wasRunning = false;
      pipetteStepper.disableOutputs();
      Serial.println("DONE");
      sendState(); // Send updated position state so UI can sync
    } else {
      if (!pipetteStepper.isRunning()) {
        pipetteStepper.disableOutputs(); 
      }
    }
  }
}
