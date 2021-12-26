#define MY_ENABLE_SPI 1
#define MY_ENABLE_SERIAL 1

#if MY_ENABLE_SPI
  #include <string.h>
  #include <SPI.h>
#endif

#define MY_USE_TIMERS 1

#if MY_USE_TIMERS
  #include <avr/io.h>
  #include <math.h>
  
  #define DIO54_PIN   PINF0
  #define DIO54_RPORT PINF
  #define DIO54_WPORT PORTF
  #define DIO54_DDR   DDRF
  #define DIO54_PWM   NULL
#endif

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define LED_PIN            13 
#define PS_ON_PIN          12 // killer pin
#define ONE_LED_PIN        65 // power switch led, LOW = ON
#define ONE_BUTTON_PIN     18 // power switch button
#define ONE_BUTTON_DELAY   50 // ms

#define MISO_PIN          50  // B3
#define MOSI_PIN          51  // B2
#define SCK_PIN           52   // B1
#define SS_PIN            53    // B0

#if MY_USE_TIMERS
  // #define F_CPU              16000000UL
  #define PS_ON_ASLEEP       HIGH
  #define PS_ON_AWAKE        LOW
   
  // MarlinSerial.h
  #ifndef CRITICAL_SECTION_START
    #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
    #define CRITICAL_SECTION_END    SREG = _sreg;
  #endif
  
  // Marlin fastio.h
  #define _READ(IO) ((bool)(DIO ## IO ## _RPORT & bit(DIO ## IO ## _PIN)))
  #define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= bit(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~bit(DIO ## IO ## _PIN); }; } while (0)
  #define _WRITE_C(IO, v)   do { if (v) { \
       CRITICAL_SECTION_START; \
       {DIO ##  IO ## _WPORT |= bit(DIO ## IO ## _PIN); } \
       CRITICAL_SECTION_END; \
     } \
     else { \
       CRITICAL_SECTION_START; \
       {DIO ##  IO ## _WPORT &= ~bit(DIO ## IO ## _PIN); } \
       CRITICAL_SECTION_END; \
     } \
   } \
   while (0)
  #define _WRITE(IO, v)  do {  if (&(DIO ##  IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)
  #define _SET_INPUT(IO) do {DIO ##  IO ## _DDR &= ~bit(DIO ## IO ## _PIN); } while (0)
  #define _SET_OUTPUT(IO) do {DIO ##  IO ## _DDR |=  bit(DIO ## IO ## _PIN); } while (0)
#endif


const unsigned long HALF_STEP_DELAY_MIN = 0;
const unsigned long HALF_STEP_DELAY_MAX = 9000;
const float RPM_MAX = 493.0;
const float RPM_MIN = -RPM_MAX;
const float RPM_INC_MAX = 100.0;
const int RPM_DEBOUNCE_DELAY = 50;
const int MICROSTEPPING = 16;
const int STEPS_PER_ROTATION_NOMINAL = 200;
const unsigned long STEPS_PER_ROTATION = STEPS_PER_ROTATION_NOMINAL * MICROSTEPPING;

unsigned long lastBtnDebounceTime;
int lastBtnRead;
bool lastBtnState;
bool btnState;

float rpm = 0;
float targetRpm = 0;
unsigned long lastRpmDebounceTime;

#if MY_USE_TIMERS
  int timer2_prescale;
  int timer2_top;
  int timer2_stepState;

  bool isTimer2CompAEnabled() {
    return 1 == bitRead(TIMSK2, 1);
  }

  // disable ISC(TIMER2_COMPA_vect) call
  void disableTimer2CompA() {
    bitClear(TIMSK2, 1);
  }
  
  // enable ISC(TIMER2_COMPA_vect) call
  void enableTimer2CompA() {
    bitSet(TIMSK2, 1);
  }
  
  float getCountingStepDuration(int prescale) {
    return ((float)prescale / (float)F_CPU) * 1e+6; // microseconds
  }
  
  /**
   * Check if given prescaler value is valid to count to given value allowing approximation
   * 
   * @param unsigned long d Delay time in microseconds
   * @param int prescale Prescaler value (8, 64, 128, 256 or 1024)
   * @param bool strict Keep precision of counter or approximate (smaller prescale is more precise)
   * 
   * @return bool Valid prescale ?
   */
  float isValidCounterPrescale(unsigned long d, int timerSize, int prescale, bool strict = true) {
    Serial.println("isValidCounterPrescale");
    Serial.println(d);
    Serial.println(timerSize);
    Serial.println(prescale);
    Serial.println(strict);
    double maxTimerValue = pow(2, timerSize);
    
    float countingStepDuration = getCountingStepDuration(prescale);
    Serial.println("countingStepDuration");
    Serial.println(countingStepDuration);

    int top = round(d / countingStepDuration);
    
    if (top <= 0 || top > maxTimerValue) {
      Serial.println("return 1");
      return false;
    }
    
    /*
    if (countingStepDuration > 1e+6 / maxTimerValue) {
      Serial.println("return 2");
      return false;
    }
    */
  
    if (true == strict && countingStepDuration > d) {
      // counting step duration must be greater than delay time
      Serial.println("return 3");
      return false;
    }

    if (false == strict) {
      return true;
    }
    
    Serial.println("return 4");
    // counting step duration must be a divider of delay time and lesser than timer max value
    Serial.println(fmodf((float) d, countingStepDuration));
    return fmodf((float) d, countingStepDuration) == 0;
  }
    
  int getPrescaler(unsigned long d, int timerSize, bool strict = true, bool nonStrictFallback = true) {
    if (
      isValidCounterPrescale(d, timerSize, timer2_prescale, strict)
      || (
        strict && nonStrictFallback && isValidCounterPrescale(d, timerSize, timer2_prescale, false)
      )
    ) {
      return timer2_prescale;
    }
    
    if (1024 != timer2_prescale && isValidCounterPrescale(d, timerSize, 1024, strict)) {
      return 1024;
    }
    
    if (256 != timer2_prescale && isValidCounterPrescale(d, timerSize, 256, strict)) {
      return 256;
    }
    
    if (64 != timer2_prescale && isValidCounterPrescale(d, timerSize, 64, strict)) {
      return 64;
    }
    
    if (8 != timer2_prescale && isValidCounterPrescale(d, timerSize, 8, strict)) {
      return 8;
    }
    
    if (1 != timer2_prescale && isValidCounterPrescale(d, timerSize, 1, strict)) {
      return 1;
    }
  
    if (strict && nonStrictFallback) {
      return getPrescaler(d, timerSize, false, false);
    }
  
    return 0; // disable counter
  }
  
  void prepareMotorRun(float newRpm, bool strict = true) {
    Serial.println("Prepare motor");

    unsigned long newHalfStepDelay = rpmToHalfStepDelay(newRpm);
    int prescale = getPrescaler(newHalfStepDelay, 8, true);
    if (prescale == 0) {
      targetRpm = 0;
      rpm = 0;
      stopMotorRun();
      return;
    }
    
    float countingStepDuration = getCountingStepDuration(prescale);
    Serial.println("+++");
    Serial.println(prescale);
    Serial.println(newHalfStepDelay);
    Serial.println(countingStepDuration);
    Serial.println(newHalfStepDelay / countingStepDuration);
    Serial.println(round(newHalfStepDelay / countingStepDuration));
    Serial.println("+-+");
    int top = round(newHalfStepDelay / countingStepDuration);

    ///*
    Serial.print("Prescaler:");
    Serial.println(prescale);
    Serial.println("Top:");
    Serial.println(top);
    Serial.println("RPM:");
    Serial.println(newRpm);
    Serial.println("Delay:");
    Serial.println(newHalfStepDelay);
    Serial.println("Counting duration:");
    Serial.println(countingStepDuration);
    Serial.println(!isTimer2CompAEnabled());
    //*/
    if (top <= 0) {
      disableTimer2CompA();
      return;
    }
  
    if (!isTimer2CompAEnabled() || prescale != timer2_prescale) {
      cli();
      
      // Serial.println("Timer configuration");
      timer2_stepState = LOW;
    
      // enable async
      // bitSet(ASSR, EXCLK);
      // bitSet(ASSR, AS2);
    
      // Compare Output Mode
      bitClear(TCCR2A, COM2A1); // On compare match: clear/set OC2A
      bitClear(TCCR2A, COM2A0); // On compare match: set OC2A + if WGM22 is active: toggle OC2A
      bitClear(TCCR2A, COM2B1); // On compare match: clear/set OC2B
      bitClear(TCCR2A, COM2B0); // On compare match: set OC2B + if WGM22 is active: toggle OC2B
      
      // ensure Output Compare value is in output mode
      // _SET_OUTPUT(TIMER2A); // aka PWM9 aka OC2A
      // _SET_OUTPUT(TIMER2B); // aka PWM10 aka OC2B
      
      // Waveform Generation Mode
      bitSet(TCCR2A, WGM21); // CTC Mode: Clear Timer on Compare match with OCR2A - Fast PWM (MAX/TOP)
      bitClear(TCCR2A, WGM20); // Fast PWM (MAX/TOP) - Phase Correct PWM (MAX/TOP) counting from BOTTOM to (MAX/TOP) then from (MAX/TOP) to BOTTOM
      bitClear(TCCR2B, WGM22); // Fast PWM (TOP) - Phase Correct PWM (TOP)
    
      bitClear(TCCR2A, FOC2A); // Force Output Compare: While in non-PWM, immediate comparison with OCR2A
      bitClear(TCCR2B, FOC2B); // Force Output Compare: While in non-PWM, immediate comparison with OCR2B
    
      // Clock Select prescaler value
      switch (prescale) {
        case 1:
          bitClear(TCCR2B, CS22);
          bitClear(TCCR2B, CS21);
          bitSet(TCCR2B, CS20);
          break;
        case 8:
          bitClear(TCCR2B, CS22);
          bitSet(TCCR2B, CS21);
          bitClear(TCCR2B, CS20);
          break;
        case 32:
          bitClear(TCCR2B, CS22);
          bitSet(TCCR2B, CS21);
          bitSet(TCCR2B, CS20);
          break;
        case 64:
          bitSet(TCCR2B, CS22);
          bitClear(TCCR2B, CS21);
          bitClear(TCCR2B, CS20);
          break;
        case 128:
          bitSet(TCCR2B, CS22);
          bitClear(TCCR2B, CS21);
          bitSet(TCCR2B, CS20);
          break;
        case 256:
          bitSet(TCCR2B, CS22);
          bitSet(TCCR2B, CS21);
          bitClear(TCCR2B, CS20);
          break;
        case 1024:
          bitSet(TCCR2B, CS22);
          bitSet(TCCR2B, CS21);
          bitSet(TCCR2B, CS20);
          break;
      }
  
      TCNT2 = 0; // reset counter value
    } else {
      disableTimer2CompA();
    }

    digitalWrite(X_DIR_PIN, newRpm > 0 ? LOW : HIGH);
    
    rpm = newRpm;
    timer2_prescale = prescale;
    timer2_top = top;
    
    OCR2A = timer2_top;
    
    enableTimer2CompA();
    sei();
  }

  void updateSmoothMotorRun(bool strict = true) {
    if (targetRpm == 0 && rpm == 0 && isTimer2CompAEnabled()) {
      stopMotorRun();
      return;
    }
    
    float rpmInc = targetRpm - rpm;
    
    if (rpmInc == 0) {
      return;
    }

    unsigned long now = millis();
    if (now - lastRpmDebounceTime < RPM_DEBOUNCE_DELAY) {
      return;
    }

    lastRpmDebounceTime = now;
    
    int rpmIncSign = rpmInc > 0 ? 1 : -1;

    prepareMotorRun(
      rpm + rpmIncSign * min(abs(rpmInc), RPM_INC_MAX),
      strict
    );
  }

  void changeMotorSpeed(char axisName, float speedFactor, bool strictTimer = true) {
    Serial.print("MOVE ");
    Serial.print(axisName);
    Serial.print(" ");
    Serial.println(speedFactor);

    float halfRpmLength = 0.5 * abs(RPM_MAX - RPM_MIN);
    targetRpm = RPM_MIN + halfRpmLength + speedFactor * halfRpmLength;
    Serial.println(RPM_MIN);
    Serial.println(speedFactor);
    Serial.println(RPM_MIN + halfRpmLength + speedFactor * halfRpmLength);
    Serial.println("Target rpm: ");
    Serial.println(targetRpm);

    if (!isTimer2CompAEnabled()) {
      updateSmoothMotorRun(strictTimer);
    }
  }
  
  void stopMotorRun() {
    Serial.println("Stop motor");

    disableTimer2CompA();

    rpm = 0;
    targetRpm = 0;
    bitClear(TIMSK2, 1); // disable ISC(TIMER2_COMPA_vect) call
    TCNT2 = 0; // reset counter value
  }
  
  ISR(TIMER2_COMPA_vect) {
    if (rpm == 0) {
      return;
    }

    // Serial.println("---");
    // Serial.println(ASSR);
    _WRITE(54, timer2_stepState);
    timer2_stepState = timer2_stepState == HIGH ? LOW : HIGH;
  }
#endif

#if MY_ENABLE_SPI
  String spiBuffer;
  String spiProcessBuffer;
  volatile bool spiProcess;
  
  ISR (SPI_STC_vect) {
    char c = SPDR;  // grab byte from SPI Data Register
    
    if (c == '\n') {
      spiProcessBuffer = "";
      spiProcessBuffer.concat(spiBuffer);
      spiBuffer = "";
      spiProcess = true;
      return;
    }

    spiBuffer.concat(c);
  }
#endif

void setup() {
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  /*
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  
  pinMode(E_STEP_PIN, OUTPUT);
  pinMode(E_DIR_PIN, OUTPUT);
  pinMode(E_ENABLE_PIN, OUTPUT);
  */
  pinMode(LED_PIN, OUTPUT);
  pinMode(ONE_LED_PIN, OUTPUT);
  pinMode(ONE_BUTTON_PIN, INPUT);
  
  digitalWrite(ONE_LED_PIN, HIGH);
  
  digitalWrite(X_DIR_PIN, LOW);

  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);

  lastBtnRead = HIGH;
  lastBtnState = false;
  btnState = false;
  lastBtnDebounceTime = 0;

  #if MY_USE_TIMERS
    timer2_prescale = 1;
    timer2_top = 0;
    timer2_stepState = LOW;
  #endif
  
  // initialize serial communication:
  Serial.begin(9600);
  #if MY_USE_TIMERS
  Serial.println("Using timers");
  Serial.println(AS2);
  Serial.println(EXCLK);
  #endif

  #if MY_ENABLE_SPI
    // turn on SPI in slave mode
    SPCR |= bit(SPE);
    SPCR |= bit(SPIE);/*
    SPCR |= bit(CPOL);
    
    pinMode(MOSI, INPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SCK, INPUT);*/
    pinMode(SS, INPUT);

    // get ready for an interrupt
    spiBuffer = "";
    spiProcess = false;
    SPI.attachInterrupt();
  #endif
}
/*
void blink(int d = 1000, int pin = LED_PIN) {
  digitalWrite(pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(d);                       // wait for a second
  digitalWrite(pin, LOW);    // turn the LED off by making the voltage LOW
  delay(d);                       // wait for a second
}
*/

bool readSwitch(int pin, unsigned long debounceDelay) {
  int currentBtnRead = digitalRead(pin);
  if (currentBtnRead == LOW and currentBtnRead != lastBtnRead) {
    unsigned long now = millis();
    // Serial.println(now - lastBtnDebounceTime);
    if (now - lastBtnDebounceTime > debounceDelay) {
      btnState = !btnState;
      lastBtnDebounceTime = now;
    }
  }
  
  lastBtnRead = currentBtnRead;

  return btnState;
}

void doStep(unsigned long halfStepDelay) {
  digitalWrite(X_STEP_PIN, LOW); 
  delayMicroseconds(halfStepDelay);
  digitalWrite(X_STEP_PIN, HIGH);
  delayMicroseconds(halfStepDelay);
}

unsigned long doSteps(unsigned long nb, unsigned long halfStepDelay) {
  for (int i = 0; i < nb; i++) {
    doStep(halfStepDelay);
  }

  return nb;
}

float getMinRPM() {
  return HALF_STEP_DELAY_MAX <= 0
    ? -1 
    : 60000000.0 / (2 * HALF_STEP_DELAY_MAX * STEPS_PER_ROTATION);
}

float getMaxRPM() {
  return HALF_STEP_DELAY_MIN <= 0 
    ? -1 
    : 60000000.0 / (2 * HALF_STEP_DELAY_MIN * STEPS_PER_ROTATION);
}

float getBoundedRPM(float rpm) {
  float minRPM = getMinRPM();
  if (minRPM >= rpm) {
     return minRPM;
  }
  
  float maxRPM = getMaxRPM();
  if (maxRPM <= rpm) {
     return maxRPM;
  }

  return rpm;
}

unsigned long rpmToHalfStepDelay(float rpm) {
  return 0.5 * 60e+6 / (abs(rpm) * STEPS_PER_ROTATION);
}

unsigned long doFullRotations(unsigned long nb, float rpm, bool bound = true) {
  unsigned long nbSteps = nb * STEPS_PER_ROTATION * MICROSTEPPING;
  
  if (bound) {
    rpm = getBoundedRPM(rpm);
  }
  
  doSteps(nbSteps, rpmToHalfStepDelay(rpm));
  
  return nbSteps;
}

unsigned long angleToSteps(float angle) {
   return round(0.5 * angle * STEPS_PER_ROTATION);
}

unsigned long doRotate(float angle, float rpm, bool bound = true) {
  unsigned long nbSteps = angleToSteps(angle);

  if (bound) {
    rpm = getBoundedRPM(rpm);
  }
  
  doSteps(nbSteps, rpmToHalfStepDelay(rpm));
  
  return nbSteps;
}

#if MY_ENABLE_SPI
  void execCommand(String cmd) {
    Serial.print("cmd:");
    Serial.println(cmd);
    Serial.println("--");

    String cmdMove = cmd.substring(0, 7);
    if (cmdMove == "MOVE X "
      || cmdMove == "MOVE Y "
      || cmdMove == "MOVE Z "
      || cmdMove == "MOVE E "
    ) {
      float speedFactor = cmd.substring(7, 20).toFloat();
      if (speedFactor < -1 || speedFactor > 1) {
        return;
      }
      changeMotorSpeed(cmdMove[5], speedFactor);
    } else if(cmd == "STOP") {
      stopMotorRun();
    }
  }
#endif

void onBtnClick() {
  Serial.println("btn click");
  Serial.println(btnState);
  
  lastBtnState = btnState;
  digitalWrite(ONE_LED_PIN, btnState ? LOW : HIGH);
  digitalWrite(X_ENABLE_PIN, btnState ? LOW : HIGH);
  // digitalWrite(Y_ENABLE_PIN, btnState ? HIGH : LOW);
  // digitalWrite(Z_ENABLE_PIN, btnState ? HIGH : LOW);
  // digitalWrite(E_ENABLE_PIN, btnState ? HIGH : LOW);

  #if MY_USE_TIMERS
  if (btnState) {
    changeMotorSpeed('X', HALF_STEP_DELAY_MAX);
  } else {
    changeMotorSpeed('X', 0);
  }
  #endif
}

void whileBtnActive() {
  // Serial.println("btn active");
  // nbSteps += doSteps(microStepping, halfStepDelay);
  // nbSteps += doFullRotations(1, 493.0, false);
  // nbSteps += doRotate(1.0, 60.0);
  // Serial.println(nbSteps);
}

void loop() {
  // Serial.println("loop");
  readSwitch(ONE_BUTTON_PIN, ONE_BUTTON_DELAY);
  updateSmoothMotorRun();
  
  if (btnState != lastBtnState) {
    onBtnClick();
  }

  #if MY_ENABLE_SERIAL
    if (Serial.available()) {
      Serial.println("Reading serial");
      String msg = "";
      int inByte;
      do {
        inByte = Serial.read();
        if (inByte != -1 && inByte != '\n') {
          msg += char(inByte);
        }
      } while(inByte != '\n');
      
      execCommand(msg);
    }
  #endif

  #if MY_ENABLE_SPI
  String cmd = "";
  
  noInterrupts();
  if (spiProcess) {
    cmd.concat(spiProcessBuffer);
    spiProcessBuffer = "";
    spiProcess = false;
  }
  interrupts();

  if (cmd != "") {
    execCommand(cmd);
  }
  #endif

  /*
  if (btnState) {
    whileBtnActive();
    Serial.println("loop end");
  }
  */
}
