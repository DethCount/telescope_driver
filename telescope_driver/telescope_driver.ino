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
/*
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24
*/
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
  
  #ifndef MASK
    #define MASK(PIN)  (1 << PIN)
  #endif
  
  #define _BV(bit) (1 << (bit))
  #define SBI(n,b) (n |= _BV(b))
  #define CBI(n,b) (n &= ~_BV(b))
  
  // MarlinSerial.h
  #ifndef CRITICAL_SECTION_START
    #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
    #define CRITICAL_SECTION_END    SREG = _sreg;
  #endif
  
  // Marlin fastio.h
  #define _READ(IO) ((bool)(DIO ## IO ## _RPORT & MASK(DIO ## IO ## _PIN)))
  #define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }; } while (0)
  #define _WRITE_C(IO, v)   do { if (v) { \
       CRITICAL_SECTION_START; \
       {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } \
       CRITICAL_SECTION_END; \
     } \
     else { \
       CRITICAL_SECTION_START; \
       {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); } \
       CRITICAL_SECTION_END; \
     } \
   } \
   while (0)
  #define _WRITE(IO, v)  do {  if (&(DIO ##  IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)
  #define _SET_INPUT(IO) do {DIO ##  IO ## _DDR &= ~MASK(DIO ## IO ## _PIN); } while (0)
  #define _SET_OUTPUT(IO) do {DIO ##  IO ## _DDR |=  MASK(DIO ## IO ## _PIN); } while (0)
#endif

unsigned long lastBtnDebounceTime;
int lastBtnRead;
bool lastBtnState;
bool btnState;
int halfStepDelayMin = 18;
unsigned long halfStepDelayMax = 9000;
unsigned long halfStepDelay = halfStepDelayMin;
unsigned int maxRpm = 493;
int halfStepDelayInc = 0;
int microStepping = 16;
int stepsPerRotationNominal = 200;
unsigned long stepsPerRotation = stepsPerRotationNominal * microStepping;
unsigned long maxNbSteps = 10 * stepsPerRotation;
unsigned long nbSteps;

int timer2_prescale;
int timer2_top;
int timer2_stepState;

#if MY_USE_TIMERS
  float getCountingStepDuration(int prescale) {
    Serial.println("getCountingStepDuration");
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
    if (d / prescale >= pow(2, timerSize)) {
      Serial.println("return 1");
      return false;
    }
    
    float countingStepDuration = getCountingStepDuration(prescale);
    Serial.println("countingStepDuration");
    Serial.println(countingStepDuration);
    if (countingStepDuration > 1e+6 / 0x10000) {
      Serial.println("return 2");
      return false;
    }
  
    if (true == strict && countingStepDuration > d) {
      // counting step duration must be greater than delay time
      Serial.println("return 3");
      return false;
    }
    
    Serial.println("return 4");
    // counting step duration must be a divider of delay time and lesser than timer max value
    Serial.println(fmodf((float) d, countingStepDuration));
    return fmodf((float) d, countingStepDuration) == 0;
  }
  
  int getPrescalerValue(unsigned long d, int timerSize, bool strict = true, bool nonStrictFallback = true) {
    if (isValidCounterPrescale(d, 16, 1024, strict)) {
      return 0b101;
    }
    
    if (isValidCounterPrescale(d, 16, 256, strict)) {
      return 0b100;
    }
    
    if (isValidCounterPrescale(d, 16, 64, strict)) {
      return 0b011;
    }
    
    if (isValidCounterPrescale(d, 16, 8, strict)) {
      return 0b010;
    }
    
    if (isValidCounterPrescale(d, 16, 1, strict)) {
      return 0b001;
    }
  
    if (strict && nonStrictFallback) {
      return getPrescalerValue(d, timerSize, false, false);
    }
  
    return 0b000; // disable counter
  }
  
  void prepareMotorRun(unsigned long halfStepDelay, bool strict = true) {
    Serial.println("Prepare motor");
    cli(); // disable interrupts
    
    timer2_prescale = getPrescalerValue(halfStepDelay, 8, true);
    Serial.print("Prescaler:");
    Serial.println(timer2_prescale);
    
    float countingStepDuration = getCountingStepDuration(timer2_prescale);
    timer2_top = round(halfStepDelay / countingStepDuration);
    
    Serial.println("Top:");
    Serial.println(timer2_top);
    Serial.println("Delay:");
    Serial.println(halfStepDelay);
    Serial.println("Counting duration:");
    Serial.println(countingStepDuration);
    if (timer2_top <= 0) {
      sei();
      return;
    }
    
    Serial.println("Timer configuration");
    timer2_stepState = LOW;
  
    // enable async
    // SBI(ASSR, EXCLK);
    // SBI(ASSR, AS2);
  
    // Compare Output Mode
    CBI(TCCR2A, COM2A1); // On compare match: clear/set OC2A
    CBI(TCCR2A, COM2A0); // On compare match: set OC2A + if WGM22 is active: toggle OC2A
    CBI(TCCR2A, COM2B1); // On compare match: clear/set OC2B
    CBI(TCCR2A, COM2B0); // On compare match: set OC2B + if WGM22 is active: toggle OC2B
    
    // ensure Output Compare value is in output mode
    // _SET_OUTPUT(TIMER2A); // aka PWM9 aka OC2A
    // _SET_OUTPUT(TIMER2B); // aka PWM10 aka OC2B
    
    // Waveform Generation Mode
    SBI(TCCR2A, WGM21); // CTC Mode: Clear Timer on Compare match with OCR2A - Fast PWM (MAX/TOP)
    CBI(TCCR2A, WGM20); // Fast PWM (MAX/TOP) - Phase Correct PWM (MAX/TOP) counting from BOTTOM to (MAX/TOP) then from (MAX/TOP) to BOTTOM
    CBI(TCCR2B, WGM22); // Fast PWM (TOP) - Phase Correct PWM (TOP)
  
    CBI(TCCR2A, FOC2A); // Force Output Compare: While in non-PWM, immediate comparison with OCR2A
    CBI(TCCR2B, FOC2B); // Force Output Compare: While in non-PWM, immediate comparison with OCR2B
  
    // Clock Select prescaler value
    switch (timer2_prescale) {
      case 0b001: // 1
        CBI(TCCR2B, CS22);
        CBI(TCCR2B, CS21);
        SBI(TCCR2B, CS20);
        break;
      case 0b010: // 8
        CBI(TCCR2B, CS22);
        SBI(TCCR2B, CS21);
        CBI(TCCR2B, CS20);
        break;
      case 0b011: // 32
        CBI(TCCR2B, CS22);
        SBI(TCCR2B, CS21);
        SBI(TCCR2B, CS20);
        break;
      case 0b100: // 64
        SBI(TCCR2B, CS22);
        CBI(TCCR2B, CS21);
        CBI(TCCR2B, CS20);
        break;
      case 0b101: // 128
        SBI(TCCR2B, CS22);
        CBI(TCCR2B, CS21);
        SBI(TCCR2B, CS20);
        break;
      case 0b110: // 256
        SBI(TCCR2B, CS22);
        SBI(TCCR2B, CS21);
        CBI(TCCR2B, CS20);
        break;
      case 0b111: // 1024
        SBI(TCCR2B, CS22);
        SBI(TCCR2B, CS21);
        SBI(TCCR2B, CS20);
        break;
    }

    TCNT2 = 0; // reset counter value
    
    sei(); // enable interrupts
  }
  
  void startMotorRun() {
    Serial.println("Start motor");
    Serial.println(nbSteps);
    cli();
    
    SBI(TIMSK2, 1); // enable ISC(TIMER2_COMPA_vect) call
    OCR2A = timer2_top;
    if (nbSteps >= maxNbSteps) {
      nbSteps = 0;
    }

    sei();
  }
  
  void pauseMotorRun() {
    Serial.println("Pause motor");
    Serial.println(nbSteps);
    cli(); // disable interrupts
    CBI(TIMSK2, 1); // disable ISC(TIMER2_COMPA_vect) call
    sei(); // enable interrupts
  }
  
  void stopMotorRun() {
    Serial.println("Stop motor");
    Serial.println(nbSteps);
    cli(); // disable interrupts
    CBI(TIMSK2, 1); // disable ISC(TIMER2_COMPA_vect) call
    TCNT2 = 0; // reset counter value
    sei(); // enable interrupts
  }
  
  ISR(TIMER2_COMPA_vect) {
    if (nbSteps >= maxNbSteps) {
      stopMotorRun();
      return;
    }

    // Serial.println("---");
    // Serial.println(ASSR);
    _WRITE(54, timer2_stepState);
    timer2_stepState = timer2_stepState == HIGH ? LOW : HIGH;
    nbSteps++;
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
  
  nbSteps = 0;

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
  
  prepareMotorRun(halfStepDelay);
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
  return halfStepDelayMax <= 0
    ? -1 
    : 60000000.0 / (2 * halfStepDelayMax * stepsPerRotation);
}

float getMaxRPM() {
  return halfStepDelayMin <= 0 
    ? -1 
    : 60000000.0 / (2 * halfStepDelayMin * stepsPerRotation);
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
  return 0.5 * 60e+6 / (rpm * stepsPerRotation);
}

unsigned long doFullRotations(unsigned long nb, float rpm, bool bound = true) {
  unsigned long nbSteps = nb * stepsPerRotation * microStepping;
  
  if (bound) {
    rpm = getBoundedRPM(rpm);
  }
  
  doSteps(nbSteps, rpmToHalfStepDelay(rpm));
  
  return nbSteps;
}

unsigned long angleToSteps(float angle) {
   return round(0.5 * angle * stepsPerRotation);
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
    
    if (cmd == "startMotorRun") {
      startMotorRun();
    } else if (cmd == "pauseMotorRun") {
        pauseMotorRun();
    } else if(cmd == "stopMotorRun") {
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
    startMotorRun();
  } else {
    pauseMotorRun();
  }
  #endif
}

void whileBtnActive() {
  // Serial.println("btn active");
  // nbSteps += doSteps(microStepping, halfStepDelay);
  // nbSteps += doFullRotations(1, 493.0, false);
  // nbSteps += doRotate(1.0, 60.0);
  // Serial.println(nbSteps);
  
  /*
  halfStepDelay += halfStepDelayInc;
  // Serial.println(stepDelay);
  if ((halfStepDelayMin > 0 && halfStepDelay <= halfStepDelayMin) 
    || (halfStepDelayMax > 0 && halfStepDelay >= halfStepDelayMax)
  ) {
    halfStepDelayInc *= -1;
  }
  */
}

void loop() {
  // Serial.println("loop");
  readSwitch(ONE_BUTTON_PIN, ONE_BUTTON_DELAY);
  
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
  
  if (btnState && (maxNbSteps < 0 || nbSteps < maxNbSteps)) {
    whileBtnActive();
    // Serial.println("loop end");
  }
}
