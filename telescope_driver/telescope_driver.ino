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
  
  #define DIO26_PIN   PINA4
  #define DIO26_RPORT PINA
  #define DIO26_WPORT PORTA
  #define DIO26_DDR   DDRA
  #define DIO26_PWM   NULL
    
  #define DIO46_PIN   PINL3
  #define DIO46_RPORT PINL
  #define DIO46_WPORT PORTL
  #define DIO46_DDR   DDRL
  #define DIO46_PWM   &OCR5AL
  
  #define DIO54_PIN   PINF0
  #define DIO54_RPORT PINF
  #define DIO54_WPORT PORTF
  #define DIO54_DDR   DDRF
  #define DIO54_PWM   NULL
  
  #define DIO60_PIN   PINF6
  #define DIO60_RPORT PINF
  #define DIO60_WPORT PORTF
  #define DIO60_DDR   DDRF
  #define DIO60_PWM   NULL
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
const float RPM_INC_MAX = 20.0;
const int RPM_DEBOUNCE_DELAY = 10;
const int MICROSTEPPING = 16;
const int STEPS_PER_ROTATION_NOMINAL = 200;
const unsigned long STEPS_PER_ROTATION = STEPS_PER_ROTATION_NOMINAL * MICROSTEPPING;

unsigned long lastBtnDebounceTime;
int lastBtnRead;
bool lastBtnState;
bool btnState;

float rpmX = 0;
float rpmY = 0;
float rpmZ = 0;
float rpmE = 0;

float targetRpmX = 0;
float targetRpmY = 0;
float targetRpmZ = 0;
float targetRpmE = 0;

unsigned long lastRpmDebounceTime;

#if MY_USE_TIMERS
  int timerX_prescale;
  int timerX_top;
  int timerX_stepState;
  
  int timerY_prescale;
  int timerY_top;
  int timerY_stepState;
  
  int timerZ_prescale;
  int timerZ_top;
  int timerZ_stepState;
  
  int timerE_prescale;
  int timerE_top;
  int timerE_stepState;

  int getTimerPrescale(char axis) {
    if (axis == 'X') {
      return timerX_prescale;
    }
    
    if (axis == 'Y') {
      return timerY_prescale;
    }
    
    if (axis == 'Z') {
      return timerZ_prescale;
    }
    
    if (axis == 'E') {
      return timerE_prescale;
    }
  }
  
  void setTimerPrescale(char axis, int prescale) {
    if (axis == 'X') {
      timerX_prescale = prescale;
      return;
    }
    
    if (axis == 'Y') {
      timerY_prescale = prescale;
      return;
    }
    
    if (axis == 'Z') {
      timerZ_prescale = prescale;
      return;
    }
    
    if (axis == 'E') {
      timerE_prescale = prescale;
      return;
    }
  }

  int getTimerTop(char axis) {
    if (axis == 'X') {
      return timerX_top;
    }
    
    if (axis == 'Y') {
      return timerY_top;
    }
    
    if (axis == 'Z') {
      return timerZ_top;
    }
    
    if (axis == 'E') {
      return timerE_top;
    }
  }
  
  void setTimerTop(char axis, int top) {
    if (axis == 'X') {
      timerX_top = top;
      return;
    }
    
    if (axis == 'Y') {
      timerY_top = top;
      return;
    }
    
    if (axis == 'Z') {
      timerZ_top = top;
      return;
    }
    
    if (axis == 'E') {
      timerE_top = top;
      return;
    }
  }

  void setTimerCompare(char axis, int val) {
    if (axis == 'X') {
      OCR1A = val;
      return;
    }
    
    if (axis == 'Y') {
      OCR3A = val;
      return;
    }
    
    if (axis == 'Z') {
      OCR4A = val;
      return;
    }
    
    if (axis == 'E') {
      OCR5A = val;
      return;
    }
  }

  int getTimerStepState(char axis) {
    if (axis == 'X') {
      return timerX_stepState;
    }
    
    if (axis == 'Y') {
      return timerY_stepState;
    }
    
    if (axis == 'Z') {
      return timerZ_stepState;
    }
    
    if (axis == 'E') {
      return timerE_stepState;
    }
  }
  
  void setTimerStepState(char axis, int stepState) {
    if (axis == 'X') {
      timerX_stepState = stepState;
      return;
    }
    
    if (axis == 'Y') {
      timerY_stepState = stepState;
      return;
    }
    
    if (axis == 'Z') {
      timerZ_stepState = stepState;
      return;
    }
    
    if (axis == 'E') {
      timerE_stepState = stepState;
      return;
    }
  }
  
  bool isTimerEnabled(char axis) {
    if (axis == 'X') {
      return 1 == bitRead(TIMSK1, 1);
    }
    
    if (axis == 'Y') {
      return 1 == bitRead(TIMSK3, 1);
    }
    
    if (axis == 'Z') {
      return 1 == bitRead(TIMSK4, 1);
    }
    
    if (axis == 'E') {
      return 1 == bitRead(TIMSK5, 1);
    }
  }

  void disableTimer(char axis) {
    if (axis == 'X') {
      bitClear(TIMSK1, 1);
      return;
    }
    if (axis == 'Y') {
      bitClear(TIMSK3, 1);
      return;
    }
    if (axis == 'Z') {
      bitClear(TIMSK4, 1);
      return;
    }
    if (axis == 'E') {
      bitClear(TIMSK5, 1);
      return;
    }
  }

  void enableTimer(char axis) {
    if (axis == 'X') {
      bitSet(TIMSK1, 1);
      return;
    }
    if (axis == 'Y') {
      bitSet(TIMSK3, 1);
      return;
    }
    if (axis == 'Z') {
      bitSet(TIMSK4, 1);
      return;
    }
    if (axis == 'E') {
      bitSet(TIMSK5, 1);
      return;
    }
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
    
  int getPrescaler(char axis, unsigned long d, int timerSize, bool strict = true, bool nonStrictFallback = true) {
    int oldPrescale = getTimerPrescale(axis);
    if (
      isValidCounterPrescale(d, timerSize, oldPrescale, strict)
      || (
        strict && nonStrictFallback && isValidCounterPrescale(d, timerSize, oldPrescale, false)
      )
    ) {
      return oldPrescale;
    }
    
    if (1024 != oldPrescale && isValidCounterPrescale(d, timerSize, 1024, strict)) {
      return 1024;
    }
    
    if (256 != oldPrescale && isValidCounterPrescale(d, timerSize, 256, strict)) {
      return 256;
    }
    
    if (64 != oldPrescale && isValidCounterPrescale(d, timerSize, 64, strict)) {
      return 64;
    }
    
    if (8 != oldPrescale && isValidCounterPrescale(d, timerSize, 8, strict)) {
      return 8;
    }
    
    if (1 != oldPrescale && isValidCounterPrescale(d, timerSize, 1, strict)) {
      return 1;
    }
  
    if (strict && nonStrictFallback) {
      return getPrescaler(axis, d, timerSize, false, false);
    }
  
    return 0; // disable counter
  }

  void resetTimerCounter(char axis) {
    if (axis == 'X') {
      TCNT1 = 0;
      return;
    }
    
    if (axis == 'Y') {
      TCNT3 = 0;
      return;
    }
    
    if (axis == 'Z') {
      TCNT4 = 0;
      return;
    }
    
    if (axis == 'E') {
      TCNT5 = 0;
      return;
    }
  }

  void setupTimer(char axis, int prescale) {
      // Serial.println("Timer configuration");
      setTimerStepState(axis, LOW);
    
      // Compare Output Mode
      if (axis == 'X') {
        bitClear(TCCR1A, COM1A1); // On compare match: clear/set OC1A
        bitClear(TCCR1A, COM1A0); // On compare match: set OC1A + if WGM12 is active: toggle OC1A
        bitClear(TCCR1A, COM1B1); // On compare match: clear/set OC1B
        bitClear(TCCR1A, COM1B0); // On compare match: set OC1B + if WGM12 is active: toggle OC1B
        bitClear(TCCR1A, COM1C1); // On compare match: clear/set OC1C
        bitClear(TCCR1A, COM1C0); // On compare match: set OC1B + if WGM12 is active: toggle OC1C
        
        // Waveform Generation Mode (CTC)
        bitClear(TCCR1A, WGM10);
        bitClear(TCCR1A, WGM11);
        bitSet(TCCR1B, WGM12);
        bitClear(TCCR1B, WGM13);

        // Force Output Compare
        bitClear(TCCR1C, FOC1A); // While in non-PWM, immediate comparison with OCR1A
        bitClear(TCCR1C, FOC1B); // While in non-PWM, immediate comparison with OCR1B
        bitClear(TCCR1C, FOC1C); // While in non-PWM, immediate comparison with OCR1C
      } else if (axis == 'Y') {
        bitClear(TCCR3A, COM3A1); // On compare match: clear/set OC3A
        bitClear(TCCR3A, COM3A0); // On compare match: set OC3A + if WGM32 is active: toggle OC3A
        bitClear(TCCR3A, COM3B1); // On compare match: clear/set OC3B
        bitClear(TCCR3A, COM3B0); // On compare match: set OC3B + if WGM32 is active: toggle OC3B
        bitClear(TCCR3A, COM3C1); // On compare match: clear/set OC3C
        bitClear(TCCR3A, COM3C0); // On compare match: set OC3C + if WGM32 is active: toggle OC3C
        
        // Waveform Generation Mode (CTC)
        bitClear(TCCR3A, WGM30);
        bitClear(TCCR3A, WGM31);
        bitSet(TCCR3B, WGM32);
        bitClear(TCCR3B, WGM33);

        // Force Output Compare
        bitClear(TCCR3C, FOC3A); // While in non-PWM, immediate comparison with OCR1A
        bitClear(TCCR3C, FOC3B); // While in non-PWM, immediate comparison with OCR1B
        bitClear(TCCR3C, FOC3C); // While in non-PWM, immediate comparison with OCR1C
      } else if (axis == 'Z') {
        bitClear(TCCR4A, COM4A1); // On compare match: clear/set OC4A
        bitClear(TCCR4A, COM4A0); // On compare match: set OC4A + if WGM42 is active: toggle OC4A
        bitClear(TCCR4A, COM4B1); // On compare match: clear/set OC4B
        bitClear(TCCR4A, COM4B0); // On compare match: set OC4B + if WGM42 is active: toggle OC4B
        bitClear(TCCR4A, COM4C1); // On compare match: clear/set OC4C
        bitClear(TCCR4A, COM4C0); // On compare match: set OC4C + if WGM42 is active: toggle OC4C
        
        // Waveform Generation Mode (CTC)
        bitClear(TCCR4A, WGM40);
        bitClear(TCCR4A, WGM41);
        bitSet(TCCR4B, WGM42);
        bitClear(TCCR4B, WGM43);

        // Force Output Compare
        bitClear(TCCR4C, FOC4A); // While in non-PWM, immediate comparison with OCR1A
        bitClear(TCCR4C, FOC4B); // While in non-PWM, immediate comparison with OCR1B
        bitClear(TCCR4C, FOC4C); // While in non-PWM, immediate comparison with OCR1C
      } else if (axis == 'E') {
        bitClear(TCCR5A, COM5A1); // On compare match: clear/set OC5A
        bitClear(TCCR5A, COM5A0); // On compare match: set OC5A + if WGM52 is active: toggle OC5A
        bitClear(TCCR5A, COM5B1); // On compare match: clear/set OC5B
        bitClear(TCCR5A, COM5B0); // On compare match: set OC5B + if WGM52 is active: toggle OC5B
        bitClear(TCCR5A, COM5C1); // On compare match: clear/set OC5C
        bitClear(TCCR5A, COM5C0); // On compare match: set OC5C + if WGM52 is active: toggle OC5C
        
        // Waveform Generation Mode (CTC)
        bitClear(TCCR5A, WGM50);
        bitClear(TCCR5A, WGM51);
        bitSet(TCCR5B, WGM52);
        bitClear(TCCR5B, WGM53);

        // Force Output Compare
        bitClear(TCCR5C, FOC5A); // While in non-PWM, immediate comparison with OCR1A
        bitClear(TCCR5C, FOC5B); // While in non-PWM, immediate comparison with OCR1B
        bitClear(TCCR5C, FOC5C); // While in non-PWM, immediate comparison with OCR1C
      }
    
      // Clock Select prescaler value
      switch (prescale) {
        case 1:
          if (axis == 'X') {
            bitClear(TCCR1B, CS12);
            bitClear(TCCR1B, CS11);
            bitSet(TCCR1B, CS10);
          } else if (axis == 'Y') {
            bitClear(TCCR3B, CS32);
            bitClear(TCCR3B, CS31);
            bitSet(TCCR3B, CS30);
          } else if (axis == 'Z') {
            bitClear(TCCR4B, CS42);
            bitClear(TCCR4B, CS41);
            bitSet(TCCR4B, CS40);
          } else if (axis == 'E') {
            bitClear(TCCR5B, CS52);
            bitClear(TCCR5B, CS51);
            bitSet(TCCR5B, CS50);
          }
          break;
        case 8:
          if (axis == 'X') {
            bitClear(TCCR1B, CS12);
            bitSet(TCCR1B, CS11);
            bitClear(TCCR1B, CS10);
          } else if (axis == 'Y') {
            bitClear(TCCR3B, CS32);
            bitSet(TCCR3B, CS31);
            bitClear(TCCR3B, CS30);
          } else if (axis == 'Z') {
            bitClear(TCCR4B, CS42);
            bitSet(TCCR4B, CS41);
            bitClear(TCCR4B, CS40);
          } else if (axis == 'E') {
            bitClear(TCCR5B, CS52);
            bitSet(TCCR5B, CS51);
            bitClear(TCCR5B, CS50);
          }
          break;
        case 32:
          if (axis == 'X') {
            bitClear(TCCR1B, CS12);
            bitSet(TCCR1B, CS11);
            bitSet(TCCR1B, CS10);
          } else if (axis == 'Y') {
            bitClear(TCCR3B, CS32);
            bitSet(TCCR3B, CS31);
            bitSet(TCCR3B, CS30);
          } else if (axis == 'Z') {
            bitClear(TCCR4B, CS42);
            bitSet(TCCR4B, CS41);
            bitSet(TCCR4B, CS40);
          } else if (axis == 'E') {
            bitClear(TCCR5B, CS52);
            bitSet(TCCR5B, CS51);
            bitSet(TCCR5B, CS50);
          }
          break;
        case 64:
          if (axis == 'X') {
            bitSet(TCCR1B, CS12);
            bitClear(TCCR1B, CS11);
            bitClear(TCCR1B, CS10);
          } else if (axis == 'Y') {
            bitSet(TCCR3B, CS32);
            bitClear(TCCR3B, CS31);
            bitClear(TCCR3B, CS30);
          } else if (axis == 'Z') {
            bitSet(TCCR4B, CS42);
            bitClear(TCCR4B, CS41);
            bitClear(TCCR4B, CS40);
          } else if (axis == 'E') {
            bitSet(TCCR5B, CS52);
            bitClear(TCCR5B, CS51);
            bitClear(TCCR5B, CS50);
          }
          break;
        case 128:
          if (axis == 'X') {
            bitSet(TCCR1B, CS12);
            bitClear(TCCR1B, CS11);
            bitSet(TCCR1B, CS10);
          } else if (axis == 'Y') {
            bitSet(TCCR3B, CS32);
            bitClear(TCCR3B, CS31);
            bitSet(TCCR3B, CS30);
          } else if (axis == 'Z') {
            bitSet(TCCR4B, CS42);
            bitClear(TCCR4B, CS41);
            bitSet(TCCR4B, CS40);
          } else if (axis == 'E') {
            bitSet(TCCR5B, CS52);
            bitClear(TCCR5B, CS51);
            bitSet(TCCR5B, CS50);
          }
          break;
        case 256:
          if (axis == 'X') {
            bitSet(TCCR1B, CS12);
            bitSet(TCCR1B, CS11);
            bitClear(TCCR1B, CS10);
          } else if (axis == 'Y') {
            bitSet(TCCR3B, CS32);
            bitSet(TCCR3B, CS31);
            bitClear(TCCR3B, CS30);
          } else if (axis == 'Z') {
            bitSet(TCCR4B, CS42);
            bitSet(TCCR4B, CS41);
            bitClear(TCCR4B, CS40);
          } else if (axis == 'E') {
            bitSet(TCCR5B, CS52);
            bitSet(TCCR5B, CS51);
            bitClear(TCCR5B, CS50);
          }
          break;
        case 1024:
          if (axis == 'X') {
            bitSet(TCCR1B, CS12);
            bitSet(TCCR1B, CS11);
            bitSet(TCCR1B, CS10);
          } else if (axis == 'Y') {
            bitSet(TCCR3B, CS32);
            bitSet(TCCR3B, CS31);
            bitSet(TCCR3B, CS30);
          } else if (axis == 'Z') {
            bitSet(TCCR4B, CS42);
            bitSet(TCCR4B, CS41);
            bitSet(TCCR4B, CS40);
          } else if (axis == 'E') {
            bitSet(TCCR5B, CS52);
            bitSet(TCCR5B, CS51);
            bitSet(TCCR5B, CS50);
          }
          break;
      }

      // reset counter value
      resetTimerCounter(axis);
  }
  
  void prepareMotorRun(char axis, float newRpm, bool strict = true) {
    Serial.print("Prepare motor ");
    Serial.println(axis);
    
    if (newRpm == 0) {
      stopMotorRun(axis);
      return;
    }

    unsigned long newHalfStepDelay = rpmToHalfStepDelay(newRpm);
    int prescale = getPrescaler(axis, newHalfStepDelay, 16, true);
    if (prescale == 0) {
      stopMotorRun(axis);
      return;
    }
    
    float countingStepDuration = getCountingStepDuration(prescale);
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
    //*/
    
    if (top <= 0) {
      disableTimer(axis);
      return;
    }
  
    if (!isTimerEnabled(axis) || prescale != getTimerPrescale(axis)) {
      cli();
      setupTimer(axis, prescale);
    } else {
      disableTimer(axis);
    }

    if (axis == 'X') {
      digitalWrite(X_DIR_PIN, newRpm > 0 ? LOW : HIGH);
      rpmX = newRpm;
    } else if (axis == 'Y') {
      digitalWrite(Y_DIR_PIN, newRpm > 0 ? LOW : HIGH);
      rpmY = newRpm;
    } else if (axis == 'Z') {
      digitalWrite(Z_DIR_PIN, newRpm > 0 ? LOW : HIGH);
      rpmZ = newRpm;
    } else if (axis == 'E') {
      digitalWrite(E_DIR_PIN, newRpm > 0 ? LOW : HIGH);
      rpmE = newRpm;
    }
    
    setTimerPrescale(axis, prescale);
    setTimerTop(axis, top);
    setTimerCompare(axis, top);
    
    enableTimer(axis);
    sei();
  }

  void updateSmoothMotorRun(bool strict = true) {
    if (targetRpmX == 0 && rpmX == 0 && isTimerEnabled('X')) {
      stopMotorRun('X');
    }
    if (targetRpmY == 0 && rpmY == 0 && isTimerEnabled('Y')) {
      stopMotorRun('Y');
    }
    if (targetRpmZ == 0 && rpmZ == 0 && isTimerEnabled('Z')) {
      stopMotorRun('Z');
    }
    if (targetRpmE == 0 && rpmE == 0 && isTimerEnabled('E')) {
      stopMotorRun('E');
    }

    unsigned long now = millis();
    if (now - lastRpmDebounceTime < RPM_DEBOUNCE_DELAY) {
      return;
    }

    lastRpmDebounceTime = now;

    
    float rpmIncX = targetRpmX - rpmX;
    float rpmIncY = targetRpmY - rpmY;
    float rpmIncZ = targetRpmZ - rpmZ;
    float rpmIncE = targetRpmE - rpmE;
    
    if (rpmIncX != 0) {
      prepareMotorRun(
        'X',
        rpmX + (rpmIncX > 0 ? 1 : -1) * min(abs(rpmIncX), RPM_INC_MAX),
        strict
      );
    }
    
    if (rpmIncY != 0) {
      prepareMotorRun(
        'Y',
        rpmY + (rpmIncY > 0 ? 1 : -1) * min(abs(rpmIncY), RPM_INC_MAX),
        strict
      );
    }
    
    if (rpmIncZ != 0) {
      prepareMotorRun(
        'Z',
        rpmZ + (rpmIncZ > 0 ? 1 : -1) * min(abs(rpmIncZ), RPM_INC_MAX),
        strict
      );
    }
    
    if (rpmIncE != 0) {
      prepareMotorRun(
        'E',
        rpmE + (rpmIncE > 0 ? 1 : -1) * min(abs(rpmIncE), RPM_INC_MAX),
        strict
      );
    }
  }

  void changeMotorSpeed(char axis, float speedFactor, bool strictTimer = true) {
    Serial.print("MOVE ");
    Serial.print(axis);
    Serial.print(" ");
    Serial.println(speedFactor);

    float halfRpmLength = 0.5 * abs(RPM_MAX - RPM_MIN);
    
    float targetRpm = RPM_MIN + halfRpmLength + speedFactor * halfRpmLength;
    if (axis == 'X') {
      targetRpmX = targetRpm;
    } else if (axis == 'Y') {
      targetRpmY = targetRpm;
    } else if (axis == 'Z') {
      targetRpmZ = targetRpm;
    } else if (axis == 'E') {
      targetRpmE = targetRpm;
    }
    
    Serial.println("Target rpm: ");
    Serial.println(targetRpm);

    if (!isTimerEnabled(axis)) {
      updateSmoothMotorRun(strictTimer);
    }
  }

  void stopMotorRun() {
    stopMotorRun('X');
    stopMotorRun('Y');
    stopMotorRun('Z');
    stopMotorRun('E');
  }
  
  void stopMotorRun(char axis) {
    Serial.print("Stop motor ");
    Serial.println(axis);

    disableTimer(axis);

    if (axis == 'X') {
      rpmX = 0;
      targetRpmX = 0;
    } else if (axis == 'Y') {
      rpmY = 0;
      targetRpmY = 0;
    } else if (axis == 'Z') {
      rpmZ = 0;
      targetRpmZ = 0;
    } else if (axis == 'E') {
      rpmE = 0;
      targetRpmE = 0;
    }
    
    resetTimerCounter(axis);
  }
  
  ISR(TIMER1_COMPA_vect) {
    if (rpmX == 0) {
      return;
    }
    
    _WRITE(54, timerX_stepState);
    timerX_stepState = timerX_stepState == HIGH ? LOW : HIGH;
  }
  
  ISR(TIMER3_COMPA_vect) {  
    if (rpmY == 0) {
      return;
    }
    
    _WRITE(60, timerY_stepState);
    timerY_stepState = timerY_stepState == HIGH ? LOW : HIGH;
  }
  
  ISR(TIMER4_COMPA_vect) {  
    if (rpmZ == 0) {
      return;
    }
    
    _WRITE(46, timerZ_stepState);
    timerZ_stepState = timerZ_stepState == HIGH ? LOW : HIGH;
  }
  
  ISR(TIMER5_COMPA_vect) {  
    if (rpmE == 0) {
      return;
    }
    
    _WRITE(26, timerE_stepState);
    timerE_stepState = timerE_stepState == HIGH ? LOW : HIGH;
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
  
  digitalWrite(X_DIR_PIN, LOW);
  digitalWrite(Y_DIR_PIN, LOW);
  digitalWrite(Z_DIR_PIN, LOW);
  digitalWrite(E_DIR_PIN, LOW);

  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(E_ENABLE_PIN, LOW);

  // pinMode(0, OUTPUT);
  // digitalWrite(0, HIGH);

  lastBtnRead = HIGH;
  lastBtnState = false;
  btnState = false;
  lastBtnDebounceTime = 0;

  #if MY_USE_TIMERS
    timerX_prescale = 1;
    timerX_top = 0;
    timerX_stepState = LOW;
    
    timerY_prescale = 1;
    timerY_top = 0;
    timerY_stepState = LOW;
    
    timerZ_prescale = 1;
    timerZ_top = 0;
    timerZ_stepState = LOW;
    
    timerE_prescale = 1;
    timerE_top = 0;
    timerE_stepState = LOW;
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
    SPCR |= bit(SPIE);
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

void doStep(char axis, unsigned long halfStepDelay) {
  if (axis == 'X') {
    digitalWrite(X_STEP_PIN, LOW); 
    delayMicroseconds(halfStepDelay);
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(halfStepDelay);
  } else if (axis == 'Y') {
    digitalWrite(Y_STEP_PIN, LOW); 
    delayMicroseconds(halfStepDelay);
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(halfStepDelay);
  } else if (axis == 'Z') {
    digitalWrite(Z_STEP_PIN, LOW); 
    delayMicroseconds(halfStepDelay);
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(halfStepDelay);
  } else if (axis == 'E') {
    digitalWrite(E_STEP_PIN, LOW); 
    delayMicroseconds(halfStepDelay);
    digitalWrite(E_STEP_PIN, HIGH);
    delayMicroseconds(halfStepDelay);
  }
}

unsigned long doSteps(char axis, unsigned long nb, unsigned long halfStepDelay) {
  for (int i = 0; i < nb; i++) {
    doStep(axis, halfStepDelay);
  }

  return nb;
}

float getBoundedRPM(float rpm) {
  if (RPM_MIN >= rpm) {
     return RPM_MIN;
  }
  
  if (RPM_MAX <= rpm) {
     return RPM_MAX;
  }

  return rpm;
}

unsigned long rpmToHalfStepDelay(float rpm) {
  return 0.5 * 60e+6 / (abs(rpm) * STEPS_PER_ROTATION);
}

unsigned long doFullRotations(char axis, unsigned long nb, float rpm, bool bound = true) {
  unsigned long nbSteps = nb * STEPS_PER_ROTATION * MICROSTEPPING;
  
  if (bound) {
    rpm = getBoundedRPM(rpm);
  }
  
  doSteps(axis, nbSteps, rpmToHalfStepDelay(rpm));
  
  return nbSteps;
}

unsigned long angleToSteps(float angle) {
   return round(0.5 * angle * STEPS_PER_ROTATION);
}

unsigned long doRotate(char axis, float angle, float rpm, bool bound = true) {
  unsigned long nbSteps = angleToSteps(angle);

  if (bound) {
    rpm = getBoundedRPM(rpm);
  }
  
  doSteps(axis, nbSteps, rpmToHalfStepDelay(rpm));
  
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
    } else if(cmd == "STOP X") {
      stopMotorRun('X');
    } else if(cmd == "STOP Y") {
      stopMotorRun('Y');
    } else if(cmd == "STOP Z") {
      stopMotorRun('Z');
    } else if(cmd == "STOP E") {
      stopMotorRun('E');
    }
  }
#endif

void onBtnClick() {
  Serial.println("btn click");
  Serial.println(btnState);
  
  lastBtnState = btnState;
  digitalWrite(ONE_LED_PIN, btnState ? LOW : HIGH);
  digitalWrite(X_ENABLE_PIN, btnState ? LOW : HIGH);
  digitalWrite(Y_ENABLE_PIN, btnState ? HIGH : LOW);
  digitalWrite(Z_ENABLE_PIN, btnState ? HIGH : LOW);
  digitalWrite(E_ENABLE_PIN, btnState ? HIGH : LOW);

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
  // nbSteps += doSteps('X', microStepping, halfStepDelay);
  // nbSteps += doFullRotations('X', 1, 493.0, false);
  // nbSteps += doRotate('X', 1.0, 60.0);
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
