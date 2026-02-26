#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1_bc.h>

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Pin definitions
const int pinA = 3;
const int pinB = 4;
const int pinC = 5;
const int pinD = 6;

const int limitHome = 11; // homing limit switch
const int limitEnd  = 12; // far end safety limit switch

// Full-step sequence for a 4-wire stepper
const int stepSequence[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1}
};

long position = 0;                    
const unsigned long stepDelay = 1000; 
unsigned long lastStepTime = 0;

bool stepDirection = true;
long stepsToMove = 0;
bool energized = false;

// PID control variables
unsigned long lastRequest = 0;
const unsigned long requestInterval = 15000; // 15 seconds
bool conversionRequested = false;
unsigned long now = 0;

double consigne = 23.5; // target setpoint
double entree = 0;
double sortie = 0;
// tuned PID gains
double Kp = 3000.0, Ki = 1000.0, Kd = 200.0;

PID myPID(&entree, &sortie, &consigne, Kp, Ki, Kd, DIRECT);

// Homing state
bool homed = false;
unsigned long lastHomingTime = 0;  
const unsigned long HOMING_INTERVAL = 172800000UL; 
const long MAX_TRAVEL_STEPS = 17866;               

// Step thresholding
long stepAccumulator = 0;

// Flag to wait for first temperature reading
bool firstValidTemperature = false;

void setup() {
  Serial.begin(9600);

  sensors.begin();
  sensors.setResolution(12); // high resolution
  sensors.setWaitForConversion(false);

  myPID.SetOutputLimits(800, MAX_TRAVEL_STEPS); // min 800 steps = 2 rev
  myPID.SetMode(AUTOMATIC);

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);

  pinMode(limitHome, INPUT); 
  pinMode(limitEnd, INPUT);  

  disableStepper();

  doHoming();
}

void disableStepper() {
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, LOW);
  digitalWrite(pinD, LOW);
  energized = false;
}

void stepOutput(int index) {
  digitalWrite(pinA, stepSequence[index][0]);
  digitalWrite(pinB, stepSequence[index][1]);
  digitalWrite(pinC, stepSequence[index][2]);
  digitalWrite(pinD, stepSequence[index][3]);
  energized = true;
}

void runStepper() {
  if (stepsToMove == 0) {
    if (energized) disableStepper();
    return;
  }

  unsigned long nowMicros = micros();
  if (nowMicros - lastStepTime >= stepDelay) {
    if (stepDirection) {
      position++;
    } else {
      position--;
    }

    int idx = (int)((position % 4 + 4) % 4);
    stepOutput(idx);

    stepsToMove--;
    lastStepTime = nowMicros;
  }
}

void moveStepper(long steps, bool direction) {
  stepDirection = direction;
  stepsToMove = steps;
}

// Homing routine
void doHoming() {
  Serial.println("Starting homing...");

  unsigned long startTime = millis();
  moveStepper(9999999, false); 

  while (true) {
    runStepper();

    if (digitalRead(limitHome) == HIGH) {
      stepsToMove = 0;
      position = 0;
      Serial.println("Home switch reached");
      break;
    }

    if (millis() - startTime > 20000) {
      stepsToMove = 0;
      position = 0;
      Serial.println("Homing timeout, assuming position 0");
      break;
    }
  }

  disableStepper();

  moveStepper(16000, true);
  while (stepsToMove > 0) {
    runStepper();
  }

  Serial.println("Homing finished");
  homed = true;
  lastHomingTime = millis();
  firstValidTemperature = false; // reset for next cycle
}

// Initialize PID output and internal states to current position
void initializePIDtoCurrentPosition() {
  sortie = position;                // PID output = current position
  myPID.SetMode(MANUAL);            // temporarily manual
  myPID.SetOutputLimits(800, MAX_TRAVEL_STEPS);
  myPID.SetTunings(Kp, Ki, Kd);     // reapply tunings
  myPID.SetMode(AUTOMATIC);         // back to automatic
}

void loop() {
  if (!homed) return;

  runStepper();

  now = millis();

  if (now - lastHomingTime >= HOMING_INTERVAL) {
    homed = false;
    doHoming();
  }

  if (!conversionRequested && now - lastRequest >= requestInterval) {
    sensors.requestTemperatures();
    lastRequest = now;
    conversionRequested = true;
  }

  if (conversionRequested && now - lastRequest >= 800) {
    float tempC = sensors.getTempCByIndex(0);
    conversionRequested = false;

    if (tempC != DEVICE_DISCONNECTED_C) {
      entree = tempC;

      if (!firstValidTemperature) {
        initializePIDtoCurrentPosition(); // ensure PID starts at current position
        firstValidTemperature = true;
        Serial.println("First valid temperature received → PID initialized");
      }

      // Deadzone check
      if (entree >= 23 && entree <= 24) {
        Serial.println("Within deadzone → no PID update");
      } else {
        myPID.Compute();

        long targetSteps = constrain(round(sortie), 400, MAX_TRAVEL_STEPS);
        long deltaSteps = targetSteps - position;

        if (abs(deltaSteps) < 400) {
          stepAccumulator += deltaSteps;
          if ((targetSteps >= 17866) && (position == 17866)) {
            stepAccumulator = 0;
          }
          if ((targetSteps <= 400) && (position == 400)) {
            stepAccumulator = 0;
          }
          if (abs(stepAccumulator) >= 400) {
            moveStepper(abs(stepAccumulator), stepAccumulator > 0);
            stepAccumulator = 0;
          }
        } else {
          moveStepper(abs(deltaSteps), deltaSteps > 0);
          stepAccumulator = 0;
        }
      }

      Serial.print("Temp: "); Serial.print(entree);
      Serial.print("  PID Out: "); Serial.print(sortie);
      Serial.print("  Position: "); Serial.println(position);
    }
  }

  if (digitalRead(limitEnd) == HIGH) {
    homed = false;
    doHoming();
  }
}