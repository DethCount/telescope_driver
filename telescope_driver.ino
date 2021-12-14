  #define X_STEP_PIN         54
  #define X_DIR_PIN          55
  #define X_ENABLE_PIN       38
  
  #define Y_STEP_PIN         60
  #define Y_DIR_PIN          61
  #define Y_ENABLE_PIN       56
  
  #define Z_STEP_PIN         46
  #define Z_DIR_PIN          48
  #define Z_ENABLE_PIN       62
  
  #define E_STEP_PIN        26
  #define E_DIR_PIN         28
  #define E_ENABLE_PIN      24

  #define LED_PIN            13 
  #define PS_ON_PIN          12 // killer pin
  #define ONE_LED_PIN        65 // power switch led, LOW = ON
  #define ONE_BUTTON_PIN     18 // power switch button

  #define MAX_STEP_FREQUENCY 40000
  #define PS_ON_ASLEEP       HIGH
  #define PS_ON_AWAKE        LOW
  #define ONE_BUTTON_DELAY   50 // ms

unsigned long lastBtnDebounceTime;
int lastBtnRead;
bool lastBtnState;
bool btnState;
int stepDelayMin = 0;
int stepDelayMax = 15;
int stepDelay = stepDelayMax;
int stepDelayInc = 0;
int microSteps = 1;
int stepsPerRotation = 3200;
unsigned long maxNbSteps = 1000 * stepsPerRotation;
int nbSteps = 0;

void setup() {
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  
  pinMode(E_STEP_PIN, OUTPUT);
  pinMode(E_DIR_PIN, OUTPUT);
  pinMode(E_ENABLE_PIN, OUTPUT);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(ONE_LED_PIN, OUTPUT);
  pinMode(ONE_BUTTON_PIN, INPUT);
  
  digitalWrite(ONE_LED_PIN, HIGH);

  lastBtnRead = HIGH;
  lastBtnState = false;
  btnState = false;
  lastBtnDebounceTime = 0;

  // initialize serial communication:
  Serial.begin(9600);
}

void blink(int d = 1000, int pin = LED_PIN) {
  digitalWrite(pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(d);                       // wait for a second
  digitalWrite(pin, LOW);    // turn the LED off by making the voltage LOW
  delay(d);                       // wait for a second
}

bool readSwitch(int pin, unsigned long debounceDelay) {
  int currentBtnRead = digitalRead(pin);
  if (currentBtnRead == LOW and currentBtnRead != lastBtnRead) {
    unsigned long now = millis();
    Serial.println(now - lastBtnDebounceTime);
    if (now - lastBtnDebounceTime > debounceDelay) {
      btnState = !btnState;
      lastBtnDebounceTime = now;
    }
  }
  
  lastBtnRead = currentBtnRead;

  return btnState;
}

void doStep(int stepDelay) {
  digitalWrite(X_STEP_PIN, LOW); 
  delayMicroseconds(stepDelay);
  digitalWrite(X_STEP_PIN, HIGH);
  delayMicroseconds(stepDelay);
  nbSteps += 1;
}

void loop() {
  readSwitch(ONE_BUTTON_PIN, ONE_BUTTON_DELAY);
  if (btnState != lastBtnState) {
    lastBtnState = btnState;
    digitalWrite(ONE_LED_PIN, btnState ? LOW : HIGH);
    digitalWrite(X_ENABLE_PIN, btnState ? LOW : HIGH);
    // digitalWrite(Y_ENABLE_PIN, btnState ? HIGH : LOW);
    // digitalWrite(Z_ENABLE_PIN, btnState ? HIGH : LOW);
    // digitalWrite(E_ENABLE_PIN, btnState ? HIGH : LOW);

    // digitalWrite(X_DIR_PIN, HIGH);
    // digitalWrite(X_STEP_PIN, HIGH);
    nbSteps = 0;
    Serial.println("wut");
  }
  if (btnState && (maxNbSteps < 0 || nbSteps <= maxNbSteps)) {
    digitalWrite(X_DIR_PIN, LOW);

    for (int i = 0; i < microSteps; i++) {
      doStep(stepDelay);
    }
    
    stepDelay += stepDelayInc;
    if (stepDelay <= stepDelayMin || stepDelay >= stepDelayMax) {
      stepDelayInc *= -1;
    }
  }
}
