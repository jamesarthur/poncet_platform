#include <LiquidCrystal_I2C.h>
#include <EasyButton.h>
#include <Sequence.h>

#include <AccelStepper.h>
//#include <MultiStepper.h>


#define SERIAL_DEBUG


// MODES

typedef enum {
  MODE_POWER_ON,  // Warm-up and initialise
  MODE_TRACK,     // Tracking
  MODE_PAUSE,     // Pause tracking
  MODE_REWIND,    // Rewind to start
  MODE_DISABLED   // For testing motor
} Mode;
Mode mode;

typedef enum {
  TRACK_SIDEREAL,
  TRACK_LUNAR,
  TRACK_SOLAR,
  TRACK_KING,
  TRACK_FAST      // Max-speed shuttle to start/end
} TrackingMode;
TrackingMode trackingMode;



// PINS
const int DIR_PIN = A0;
const int STEP_PIN = A1;
const int ENABLE_PIN = A7;
const int M0_PIN = A6;
const int M1_PIN = 4;
const int M2_PIN = 5;
const int RST_SLP_PIN = A2;

const int BUTTON_PIN_A = 10;
const int BUTTON_PIN_B = 9;
const int BUTTON_PIN_C = 8;
const int BUTTON_PIN_D = 7;

const int LED_PIN = LED_BUILTIN;

const int LCD_BL_PIN = 6;  // PWM
const int LCD_SDA_PIN = A4;
const int LCD_SDL_PIN = A5;



// HARDWARE

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);


// LED
const long BLINK_FAST_RATE = 150;
const long BLINK_SLOW_RATE = 500;
int ledState = LOW;
unsigned long lastLedEvent = 0;


// BUTTONS
const int HOLD_DURATION = 2000;
const int DEBOUNCE_TIME = 35;
EasyButton buttonA(BUTTON_PIN_A, DEBOUNCE_TIME, false, false);
EasyButton buttonB(BUTTON_PIN_B, DEBOUNCE_TIME, false, false);
EasyButton buttonC(BUTTON_PIN_C, DEBOUNCE_TIME, false, false);
EasyButton buttonD(BUTTON_PIN_D, DEBOUNCE_TIME, false, false);


// MOTOR
//#define _A4988_
//#define _DRV8825_
#define _TMC2209_
const long MOTOR_STEPS_PER_REVOLUTION = 200; 
const long MOTOR_MICRO_STEPS = 8;    // 1 = full step; 2=1/2 step; 4=1/4 step; etc.
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


// THE UNIVERSE
// ASCOM standards - arcsec/sec
const float SPEED_SIDEREAL = 15.041;  
const float SPEED_LUNAR = 14.685;
const float SPEED_SOLAR = 15.0;
const float SPEED_KING = 15.0369;

const float SPEED_MAX = 450.0; // Max speed - about 30sec for a full translation
const float ACCELERATION_TIME = 5.0; // Number of seconds to spend accelerating

// PLATFORM DRIVE
const long  DRIVE_GEAR_RATIO = 40;            // 40 = 40:1 
const float DRIVE_BEARING_DIAMETER = 10.0;    // in mm
const float DRIVE_DISTANCE_PER_ARCSEC = 212.0/(20.0*3600.0); // (distance driven/angle traversed) in mm of travel per arcsec of sky
const float TOTAL_DRIVE_DISTANCE = 220.0; // max amount of distance to travel, in mm

// INTERMEDIATE CALCULATIONS
const float DRIVE_DISTANCE_PER_REVOLUTION = PI*DRIVE_BEARING_DIAMETER;
const long STEPS_PER_REVOLUTION = DRIVE_GEAR_RATIO*MOTOR_STEPS_PER_REVOLUTION*MOTOR_MICRO_STEPS;

const float MAX_ROTATIONS = TOTAL_DRIVE_DISTANCE / DRIVE_DISTANCE_PER_REVOLUTION;
const long MAX_STEPS = (long)(MAX_ROTATIONS*(float)STEPS_PER_REVOLUTION);

//const float STEPS_PER_SECOND = (float)STEPS_PER_REVOLUTION*DRIVE_DISTANCE_PER_DEGREE_OF_SKY*DEGREE_OF_SKY_PER_SECOND/DRIVE_DISTANCE_PER_REVOLUTION;
const float STEPS_PER_ARCSEC = (float)STEPS_PER_REVOLUTION*DRIVE_DISTANCE_PER_ARCSEC/DRIVE_DISTANCE_PER_REVOLUTION;

//const float RUNTIME_IN_SECONDS = (float)MAX_STEPS/STEPS_PER_SECOND;
const long BACKTRACK_AMOUNT_STEPS = MAX_STEPS/10.0;



/*
// DERIVATIVE SPEEDS FOR MODES
const float TRACKING_SPEED = STEPS_PER_SECOND;
const float TRACKING_ACCELERATION = STEPS_PER_SECOND/10.0;  // 10s acceleration/deceleration

const float STOP_DECELERATION = STEPS_PER_SECOND/3.0;
const long STOP_BACKTRACK_AMOUNT = MAX_STEPS/10.0;
const float STOP_BACKTRACK_SPEED = STEPS_PER_SECOND*20.0;

const float REWIND_ACCELERATION = STEPS_PER_SECOND/10.0;
const float REWIND_SPEED = STEPS_PER_SECOND*20.0;
*/

// vars
//long lastLcdUpdate = 0;
float trackingSpeed = 0.0;


////// INIT //////

void setup() {
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(RST_SLP_PIN, OUTPUT);
  digitalWrite(RST_SLP_PIN, LOW);

  buttonA.begin();
  buttonA.onPressedFor(DEBOUNCE_TIME, onButtonAPressed);
  buttonB.begin();
  buttonB.onPressedFor(DEBOUNCE_TIME, onButtonBPressed);
  buttonC.begin();
  buttonC.onPressedFor(DEBOUNCE_TIME, onButtonCPressed);
  buttonD.begin();
  buttonD.onPressedFor(DEBOUNCE_TIME, onButtonDPressed);

  ledOn();
  delay(100);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  analogWrite(LCD_BL_PIN, 92);    // Backlight intensity

  lcd.print("STARTING");

  setMode(MODE_POWER_ON);

  // Print info
  Serial.begin(9600);
  Serial.println("-----------------");
  Serial.println("EQ TRACKING MOUNT");
  Serial.println("-----------------");

  // Press A and D within 5 seconds to enter the calibration mode
  long t = millis();
  while(millis()-t < 5000) {
    buttonA.read();
    buttonD.read();
    if(buttonA.wasPressed() && buttonD.wasPressed()) {
      enterStepstickCalibrationMode();
    }

    delay(100);
  }

  stepper.setEnablePin(ENABLE_PIN);
  #ifdef _A4988_
    Serial.println("A4988");
    stepper.setPinsInverted(false, false, true);
  #endif
  #ifdef _DRV8825_
    Serial.println("DRV8825");
    stepper.setPinsInverted(false, false, true);
  #endif
  #ifdef _TMC2209_
    Serial.println("TMC2209");
    stepper.setPinsInverted(false, false, true);
  #endif
  stepper.enableOutputs();  // Warm up
  setMotorMicrosteps(MOTOR_MICRO_STEPS);
  //stepper.setCurrentPosition(0);
  
  Serial.print("MOTOR - Steps:");
  Serial.print(MOTOR_STEPS_PER_REVOLUTION);
  Serial.print(", Step mode:1/");
  Serial.print(MOTOR_MICRO_STEPS);
  Serial.print(", Gear ratio:");
  Serial.print(DRIVE_GEAR_RATIO);
  Serial.print(":1, Total steps/rev:");
  Serial.println(STEPS_PER_REVOLUTION);

  Serial.print("Runtime: ");
  Serial.print(MAX_STEPS/(getTrackingStepSpeed(TRACK_SIDEREAL)*60.0));
  Serial.println(" minutes");

  ledOff();
  lcd.clear();
  delay(100);

  // Register button presses
  buttonA.onPressedFor(DEBOUNCE_TIME, onButtonAPressed);
  buttonB.onPressedFor(DEBOUNCE_TIME, onButtonBPressed);
  buttonC.onPressedFor(DEBOUNCE_TIME, onButtonCPressed);
  buttonD.onPressedFor(DEBOUNCE_TIME, onButtonDPressed);

  setMode(MODE_TRACK);
  setTrackMode(TRACK_SIDEREAL);
  lcdUpdateTrackingMode();

  //track(MAX_STEPS, TRACKING_SPEED, TRACKING_ACCELERATION);
}


void setStepPins(int m0, int m1, int m2) {
  Serial.print("Pins: ");

  pinMode(M0_PIN, OUTPUT);
  digitalWrite(M0_PIN, m0);
  Serial.print(m0 ? "1" : "0");

  pinMode(M1_PIN, OUTPUT);
  digitalWrite(M1_PIN, m1);
  Serial.print(m1 ? "1" : "0");

  pinMode(M2_PIN, OUTPUT);
  digitalWrite(M2_PIN, m2);
  Serial.print(m2 ? "1" : "0");
  
  //pinMode(RST_SLP_PIN, OUTPUT);
  //digitalWrite(RST_SLP_PIN, LOW);

  Serial.println();
}

// Pinouts for stepper driver
// Steps | drv8825 | tmc2209 | a4988
//   1   |  0 0 0  |  x x x  | 0 0 0
//  1/2  |  1 0 0  |  x x x  | 1 0 0
//  1/4  |  0 1 0  |  x x x  | 0 1 0
//  1/8  |  1 1 0  |  0 0 x  | 1 1 0
//  1/16 |  0 0 1  |  1 1 x  | 1 1 1
//  1/32 |  1 0 1  |  0 1 x  | x x x
//  1/64 |  x x x  |  1 0 x  | x x x
void setMotorMicrosteps(unsigned int stepMode) {

  Serial.print("SETTING STEP MODE TO 1/");
  Serial.print(stepMode);
  Serial.println(" STEP");

  switch(stepMode) {

#ifdef _A4988_
    case 1:
      setStepPins(LOW, LOW, LOW);
      break;
    case 2:
      setStepPins(HIGH, LOW, LOW);
      break;
    case 4:
      setStepPins(LOW, HIGH, LOW);
      break;
    case 8:
      setStepPins(HIGH, HIGH, LOW);
      break;
    case 16:
      setStepPins(HIGH, HIGH, HIGH);
      break;
#endif
#ifdef _DRV8825_
    case 1:
      setStepPins(LOW, LOW, LOW);
      break;
    case 2:
      setStepPins(HIGH, LOW, LOW);
      break;
    case 4:
      setStepPins(LOW, HIGH, LOW);
      break;
    case 8:
      setStepPins(HIGH, HIGH, LOW);
      break;
    case 16:
      setStepPins(LOW, LOW, HIGH);
      break;
    case 32:
      setStepPins(HIGH, LOW, HIGH);
      break;
#endif
#ifdef _TMC2209_
    // Pin 3 low = stealthchop; high = spreadcycle
    case 8:
      setStepPins(LOW, LOW, LOW);
      break;
    case 16:
      setStepPins(HIGH, HIGH, LOW);
      break;
    case 32:
      setStepPins(HIGH, LOW, LOW);
      break;
    case 64:
      setStepPins(LOW, HIGH, LOW);
      break;
#endif

    default:
      setStepPins(LOW, LOW, LOW);
      Serial.print("STEP MODE INVALID (");
      Serial.print(stepMode);
      Serial.println(")");
      return;
  }

}


void enterStepstickCalibrationMode() {
  setMode(MODE_DISABLED);
  lcdUpdateMode();
  setMotorMicrosteps(1); // Full step mode
  stepper.enableOutputs();

  while(true) {
    delay(500);
    ledOn();
    lcd.setCursor(15,1);
    lcd.print(".");
    delay(500);
    ledOff();
    lcd.setCursor(15,1);
    lcd.print(" ");
  }
}



////// MAIN LOOP //////

void loop() {
  // put your main code here, to run repeatedly:
  long time = millis();

  switch(mode) {
    case MODE_POWER_ON:
    case MODE_DISABLED:
      break;
    case MODE_TRACK:
    case MODE_PAUSE:
    case MODE_REWIND:
      //if(time-lastLcdUpdate > 500) {    // 2fps
      //  lcdUpdateSpeed();
      //  lcdUpdateButtons();
      //  lastLcdUpdate = time;
      //}
      break;
  }
  stepper.run();
  buttonA.read();
  buttonB.read();
  buttonC.read();
  buttonD.read();
}



void setMode(Mode new_mode) {
  if(new_mode==mode)
    return;

  Serial.print("SET MODE TO ");
  Serial.println(getModeName(new_mode));

  // Dont reset tracking speed changes when moving between track and pause modes
  if(mode!=MODE_TRACK && mode!=MODE_PAUSE) {
    trackingSpeed = getTrackingSpeed(trackingMode);
  }

  mode = new_mode;
  lcdUpdateMode();

  switch(mode) {
    case MODE_POWER_ON:
    case MODE_TRACK:
      lcdUpdateSpeed();
      track(MAX_STEPS, trackingSpeed*STEPS_PER_ARCSEC, getTrackingStepAcceleration(trackingMode));
      break;
    case MODE_PAUSE:
      stop(false);
      break;
    case MODE_REWIND:
      track(0, SPEED_MAX, SPEED_MAX/ACCELERATION_TIME);
      break;
    case MODE_DISABLED:
      break;
  }
}


void setTrackMode(TrackingMode new_mode) {
  if(new_mode==trackingMode)
    return;

  Serial.print("SET TRACK MODE TO ");
  Serial.println(getTrackingModeName(new_mode));

  trackingMode = new_mode;
  lcdUpdateTrackingMode();

  trackingSpeed = getTrackingSpeed(trackingMode);
  lcdUpdateSpeed();

  switch(trackingMode) {
    case TRACK_SIDEREAL:
    case TRACK_LUNAR:
    case TRACK_SOLAR:
    case TRACK_KING:
    case TRACK_FAST:
    default:
      if(mode==MODE_TRACK) {
        track(MAX_STEPS, trackingSpeed*STEPS_PER_ARCSEC, getTrackingStepAcceleration(trackingMode));
      }
      break;
  }
}


// Track to position
void track(long position, float speed, float acceleration) {
  Serial.print("TRACKING TO ");
  Serial.print(position);
  Serial.print(", speed: ");
  Serial.print(speed);
  Serial.print("steps/sec, acceleration: ");
  Serial.print(acceleration);
  Serial.println("steps/sec2");
  
  stepper.setMaxSpeed(speed);
  stepper.setAcceleration(acceleration);
  stepper.moveTo(position);
}


// Stop as quickly as possible
void stop(bool emergency) {
  Serial.print("STOPPING.");

  if(emergency) {
    // Fastest acceleration
    stepper.setAcceleration(SPEED_MAX*STEPS_PER_ARCSEC/ACCELERATION_TIME);
  }
  stepper.stop();
}



// LED functions

void ledOn() {
  // Set LED on
  ledState = HIGH;
  digitalWrite(LED_PIN, ledState);
}

void ledOff() {
  // Set LED OFF
  ledState = LOW;
  digitalWrite(LED_PIN, ledState);
}

void ledBlink(long rate) {
  if(rate==0) {
    if(ledState==HIGH)
      ledOff();
  }
  else {
    unsigned long t = millis();
    if(lastLedEvent+rate <= t) {
      lastLedEvent = t;
      if(ledState==HIGH)
        ledOff();
      else
        ledOn();
    }
  }
  
}


// Button functions

// Button A sets the tracking mode
void onButtonAPressed() {
  switch(trackingMode) {
    case TRACK_SIDEREAL:
      setTrackMode(TRACK_LUNAR);
      break;
    case TRACK_LUNAR:
      setTrackMode(TRACK_SOLAR);
      break;
    case TRACK_SOLAR:
      setTrackMode(TRACK_KING);
      break;
    case TRACK_KING:
      setTrackMode(TRACK_FAST);
      break;
    case TRACK_FAST:
      setTrackMode(TRACK_SIDEREAL);
    default:
      break;
  }
}


// Buttons B & C increase and decrease speed
void onButtonBPressed() {
  if(trackingMode==TRACK_FAST) {
    trackingSpeed = getTrackingSpeed(trackingMode);
    track(MAX_STEPS, trackingSpeed*STEPS_PER_ARCSEC, getTrackingStepAcceleration(trackingMode));

  } else {
    trackingSpeed = trackingSpeed/1.05;
    if(trackingSpeed < 0.0)
      trackingSpeed = 0.0;

    if(mode==MODE_TRACK) {
      stepper.setMaxSpeed(trackingSpeed*STEPS_PER_ARCSEC);
    }
  }

  lcdUpdateSpeed();
}


void onButtonCPressed() {
  if(trackingMode==TRACK_FAST) {
    trackingSpeed = getTrackingSpeed(trackingMode);
    track(0, trackingSpeed*STEPS_PER_ARCSEC, getTrackingStepAcceleration(trackingMode));
  } else {
    trackingSpeed = trackingSpeed*1.05;
    if(trackingSpeed > SPEED_MAX)
      trackingSpeed = SPEED_MAX;

    if(mode==MODE_TRACK) {
      stepper.setMaxSpeed(trackingSpeed*STEPS_PER_ARCSEC);
    }
  }

  lcdUpdateSpeed();
}


// Putton D pauses, resumes
void onButtonDPressed() {
  switch(mode) {
    case MODE_TRACK:
      setMode(MODE_PAUSE);
      break;
    case MODE_PAUSE:
      setMode(MODE_TRACK);
      break;
    default:
      break;
  }
}

/*
// Long press = start
void onButtonHeld() {
}
*/


// OUTPUT MACROS

void lcdUpdateMode() {
  lcd.setCursor(0, 0);
  switch(mode) {
    case MODE_POWER_ON:
      lcd.print("POWER ON");
      break;
    case MODE_TRACK:
      lcd.print("TRACK   ");
      break;
    case MODE_PAUSE:
      lcd.print("PAUSE   ");
      break;
    case MODE_REWIND:
      lcd.print("REWIND  ");
      break;
    default:
      lcd.print("        ");
      break;
  }
}

void lcdUpdateTrackingMode() {
  lcd.setCursor(0, 1);
  switch(trackingMode) {
    case TRACK_SIDEREAL:
      lcd.print("Sidereal");
      break;
    case TRACK_LUNAR:
      lcd.print("Lunar   ");
      break;
    case TRACK_SOLAR:
      lcd.print("Solar   ");
      break;
    case TRACK_KING:
      lcd.print("King    ");
      break;
    case TRACK_FAST:
      lcd.print("Fast    ");
      break;
    default:
      lcd.print("        ");
      break;
  }
}

void lcdUpdateSpeed() {
  //float speed = stepper.speed()/STEPS_PER_ARCSEC;
  float speed = trackingSpeed;

  lcd.setCursor(9, 1);
  lcd.print(speed, 5);
}



// CALCULATIONS

// Returns tracking speed for the specified mode, in arcsec/sec
float getTrackingSpeed(TrackingMode tm) {
  float ret = 0.0;
  switch(tm) {
    case TRACK_SIDEREAL:
      ret = SPEED_SIDEREAL;
      break;
    case TRACK_LUNAR:
      ret = SPEED_LUNAR;
      break;
    case TRACK_SOLAR:
      ret = SPEED_SOLAR;
      break;
    case TRACK_KING:
      ret = SPEED_KING;
      break;
    
    case TRACK_FAST:
    default: 
      ret = SPEED_MAX;
  }  
  return ret;
}


// Returns acceleration rate to accelerate to target speed over ACCELERATION_TIME seconds, in arcsec/sec^2
float getTrackingAcceleration(TrackingMode tm) {
  float ret = 0.0;
  switch(tm) {
    case TRACK_SIDEREAL:
    case TRACK_LUNAR:
    case TRACK_SOLAR:
    case TRACK_KING:
      ret = getTrackingSpeed(tm)/ACCELERATION_TIME;
      break;
    
    case TRACK_FAST:
    default:
      ret = SPEED_MAX/ACCELERATION_TIME;
      break;
  }  
  return ret;
}



const char* getModeName(Mode m) {
  switch(m) {
    case MODE_POWER_ON:
      return "POWER_ON";
      break;
    case MODE_TRACK:
      return "TRACK";
      break;
    case MODE_PAUSE:
      return "PAUSE";
      break;
    case MODE_REWIND:
      return "REWIND";
      break;
    case MODE_DISABLED:
      return "DISABLED";
      break;
  }

  return "NONE";
}

const char* getTrackingModeName(TrackingMode tm) {
  switch(tm) {
    case TRACK_SIDEREAL:
      return "SIDEREAL";
      break;
    case TRACK_LUNAR:
      return "LUNAR";
      break;
    case TRACK_SOLAR:
      return "SOLAR";
      break;
    case TRACK_KING:
      return "KING";
      break;
    case TRACK_FAST:
      return "FAST";
      break;
  }

  return "NONE";
}


// Returns tracking speed for the specified mode, in steps/sec
float getTrackingStepSpeed(TrackingMode tm) {
  return getTrackingSpeed(tm)*STEPS_PER_ARCSEC;
}


// Returns acceleration rate to accelerate to target speed over ACCELERATION_TIME seconds, in steps/sec^2
float getTrackingStepAcceleration(TrackingMode tm) {
  return getTrackingAcceleration(tm)*STEPS_PER_ARCSEC;
}
