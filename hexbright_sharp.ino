/*
 *
 * Sharp's HexBright Firmware
 * (based on the factory firmware, with some code from hexbright4)
 * Awating a license from upstream. Until then, consider new features
 * I add as beerware.
 *
 * https://en.wikipedia.org/wiki/Beerware
 *
 */


// Basic Configuration
#define CFG_FADE true
#define CFG_ENABLE_GRAVITY_MODE true
#define CFG_DIM_BLINKING_PREVIEW true
#define CFG_LOWEST_MODE 50 // out of 255 where 255 is the same as MEDIUM
#define CFG_DISABLE_MED_MODE false


// Advanced Configuration
#define CFG_STEP 3 // higher is faster between modes

// Expert Configuration
#define CFG_OVERTEMP                340


/* CONFIGURATION ENDS HERE / HAPPY HACKING! */

// #include <math.h>
#include <Wire.h>


// Constants
#define ACC_ADDRESS             0x4C
#define ACC_REG_XOUT            0
#define ACC_REG_YOUT            1
#define ACC_REG_ZOUT            2
#define ACC_REG_TILT            3
#define ACC_REG_INTS            6
#define ACC_REG_MODE            7
// Pin assignments
#define DPIN_RLED_SW            2
#define DPIN_GLED               5
#define DPIN_PWR                8
#define DPIN_DRV_MODE           9
#define DPIN_DRV_EN             10
#define APIN_TEMP               0
#define APIN_CHARGE             3
// Modes
#define MODE_OFF                0
#define MODE_LOW                1
#define MODE_MED                2
#define MODE_HIGH               3
#define MODE_BLINKING           4
#define MODE_BLINKING_PREVIEW   5
#define MODE_GRAVITY            6
#define MODE_GRAVITY_PREVIEW    7

// State
byte mode = 0;
unsigned long btnTime = 0;
boolean btnDown = false;
byte currentOutput = 0;
byte targetOutput = 0;
unsigned long time = millis();
boolean forcefade = true; // when to force fade code
static unsigned long lastTempTime = millis();

byte disable[] = {
  ACC_REG_MODE, 0x00};  // Mode: disable!
byte enable[] = {
  ACC_REG_MODE, 0x01};  // Mode: active!

unsigned long modeChangeTime;

void blinkMode() {
  if(CFG_DIM_BLINKING_PREVIEW && mode == MODE_BLINKING_PREVIEW)
    analogWrite(DPIN_DRV_EN, (time%300)<90?CFG_LOWEST_MODE:0); // blinking preview is dim
  else
    digitalWrite(DPIN_DRV_EN, (time%300)<90); // blinking is bright
}

void gravityMode() {
  char accel[3];
  if(time%5000 < 1000) // periodicly flash green LED to remind user light is on
    if(time%1000 < 500)
      analogWrite(DPIN_GLED,map(time%500,0,500,0,255));
    else
      analogWrite(DPIN_GLED,map(time%500,0,500,255,0));

  readAccel(accel); // get accelerometer
  if (abs(accel[1]) > 18) // off threshhold at 18
      accel[1] = 18;
  targetOutput = map(abs(accel[1]),0,18,255,0);
  if(currentOutput - targetOutput > 20) { // turn off faster
    currentOutput -= 5;
  }
}

void setup() {
  // We just powered on!  That means either we got plugged 
  // into USB, or the user is pressing the power button.
  pinMode(DPIN_PWR,      INPUT);
  digitalWrite(DPIN_PWR, LOW);

  // Initialize GPIO
  pinMode(DPIN_RLED_SW,  INPUT);
  pinMode(DPIN_GLED,     OUTPUT);
  pinMode(DPIN_DRV_MODE, OUTPUT);
  pinMode(DPIN_DRV_EN,   OUTPUT);
  digitalWrite(DPIN_DRV_MODE, LOW);
  digitalWrite(DPIN_DRV_EN,   LOW);

  // Initialize serial busses
  Serial.begin(9600);
  Wire.begin();

  byte config[] = {
    ACC_REG_INTS,  // First register (see next line)
    0xE4,  // Interrupts: shakes, taps
    0x00,  // Mode: not enabled yet
    0x00,  // Sample rate: 120 Hz
    0x0F,  // Tap threshold
    0x10   // Tap debounce samples
  };
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(config, sizeof(config));
  Wire.endTransmission();

  btnTime = millis();
  btnDown = digitalRead(DPIN_RLED_SW);
  mode = MODE_OFF;

  Serial.println("Powered up!");
}

void loop() {
  time = millis();

  // Check the state of the charge controller
  int chargeState = analogRead(APIN_CHARGE);
  if (chargeState < 128 && mode == MODE_OFF) { // low - charging
    //  digitalWrite(DPIN_GLED, (time&0x0100)?LOW:HIGH);
    if(time%4000 < 2000)
      analogWrite(DPIN_GLED,map(time%2000,0,2000,0,255));
    else
      analogWrite(DPIN_GLED,map(time%2000,0,2000,255,0));
  }
  else if (chargeState > 768 && mode == MODE_OFF) { // high - charged
    digitalWrite(DPIN_GLED, HIGH);
  }
  else if (mode != MODE_GRAVITY) { // hi-z - shutdown
    digitalWrite(DPIN_GLED, LOW);    
  }

  // overheat protection
  if (time-lastTempTime > 1000) {
    lastTempTime = time;
    int temperature = analogRead(APIN_TEMP);
    Serial.print("Temp: ");
    Serial.println(temperature);
    if (temperature > CFG_OVERTEMP && mode != MODE_OFF) {
      Serial.println("Overheating!");

      for (int i = 0; i < 6; i++) {
        digitalWrite(DPIN_DRV_MODE, LOW);
        delay(100);
        digitalWrite(DPIN_DRV_MODE, HIGH);
        delay(100);
      }
      digitalWrite(DPIN_DRV_MODE, LOW);
      targetOutput = CFG_LOWEST_MODE;
      mode = MODE_LOW;
    }
  }

  // "dynamic" modes
  switch (mode) {
  case MODE_BLINKING:
  case MODE_BLINKING_PREVIEW:
    blinkMode();
    break;
  case MODE_GRAVITY:
  case MODE_GRAVITY_PREVIEW:
    gravityMode();
    break;
  }

  // Periodically pull down the button's pin, since
  // in certain hardware revisions it can float.
  pinMode(DPIN_RLED_SW, OUTPUT);
  pinMode(DPIN_RLED_SW, INPUT);

  // Check for user input to change modes
  byte newMode = mode;
  byte newBtnDown = digitalRead(DPIN_RLED_SW);
  switch (mode) {
  case MODE_OFF:
    if (btnDown && !newBtnDown && (time-btnTime)>20)
      newMode = MODE_LOW;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_BLINKING_PREVIEW;
    break;
  case MODE_LOW:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      if(CFG_DISABLE_MED_MODE) {
        newMode = MODE_HIGH;
      } else {
        newMode = MODE_MED;
      }
    break;
  case MODE_MED:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_HIGH;
    break;
  case MODE_HIGH:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_OFF;
    break;
  case MODE_BLINKING_PREVIEW:
    // This mode exists just to ignore this button release
    // or to launch into "gravity" mode
    if (CFG_ENABLE_GRAVITY_MODE && btnDown && newBtnDown && (time-btnTime) > 3000)
      newMode = MODE_GRAVITY_PREVIEW;
    if (btnDown && !newBtnDown)
      newMode = MODE_BLINKING;
    break;
  case MODE_BLINKING:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_OFF;
    break;
  case MODE_GRAVITY_PREVIEW:
    // to ignore button release
    if (btnDown && !newBtnDown)
      newMode = MODE_GRAVITY;
    break;
  case MODE_GRAVITY:
    if (btnDown && !newBtnDown && (time-btnTime)>50) 
      newMode = MODE_OFF;
    break;
  }

  // Mode transitions
  if (newMode != mode)
  {
    switch (newMode)
    {
    case MODE_OFF:
      Serial.println("Mode = off");
      digitalWrite(DPIN_GLED,LOW);
      pinMode(DPIN_PWR, OUTPUT);
      // Disable accelerometer
      Wire.beginTransmission(ACC_ADDRESS);
      Wire.write(disable, sizeof(disable));
      Wire.endTransmission();
      digitalWrite(DPIN_DRV_MODE, LOW);
      targetOutput = 0;
      if(mode == MODE_GRAVITY)// to let user know the light is going off,
        // just in case light is pointed down.
        currentOutput = 80;
      else if(!CFG_FADE)
        currentOutput = 1;
      forcefade = true;
      break;
    case MODE_LOW:
      Serial.println("Mode = low");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, LOW);
      targetOutput = CFG_LOWEST_MODE;
      forcefade = false;     
      break;
    case MODE_MED:
      Serial.println("Mode = medium");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, LOW);
      targetOutput = 255;
      forcefade = false;
      break;
    case MODE_HIGH:
      Serial.println("Mode = high");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      currentOutput = map(currentOutput,0,255,0,50); // scale back the 
      analogWrite(DPIN_DRV_EN, currentOutput);
      targetOutput = 255;
      forcefade = false;
      break;
//    case MODE_BLINKING:
    case MODE_BLINKING_PREVIEW:
      Serial.println("Mode = blinking");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      targetOutput = currentOutput = 1;
      forcefade = true;
      break;
//    case MODE_GRAVITY:
    case MODE_GRAVITY_PREVIEW:
      Serial.println("Mode = gravity");
      // Enable accelerometer
      Wire.beginTransmission(ACC_ADDRESS);
      Wire.write(enable, sizeof(enable));
      Wire.endTransmission();
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      analogWrite(DPIN_DRV_EN, 1);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      targetOutput = currentOutput = 1;
      forcefade = true;
      break;
    }      

    mode = newMode;
  }

  // Remember button state so we can detect transitions
  if (newBtnDown != btnDown) {
    btnTime = time;
    btnDown = newBtnDown;
    delay(50);
  }

  if(forcefade || CFG_FADE) {
    // some modes need to be forced into fade mode to work
    // or else the DPIN_DRV_EN will be overridden by the
    // else clause
    if(mode!=MODE_GRAVITY && mode!=MODE_GRAVITY_PREVIEW &&
      abs(currentOutput - targetOutput) < CFG_STEP && currentOutput != targetOutput) {
      currentOutput = targetOutput;
      analogWrite(DPIN_DRV_EN, currentOutput);
    } 
    else if(currentOutput < targetOutput) {
      // fading code, currentOutput catches up with targetOutput
      currentOutput+=(mode==MODE_GRAVITY||mode==MODE_GRAVITY_PREVIEW)?1:CFG_STEP;
      analogWrite(DPIN_DRV_EN, currentOutput);
      delay(3);
    } 
    else if(currentOutput > targetOutput) {
      currentOutput-=(mode==MODE_GRAVITY||mode==MODE_GRAVITY_PREVIEW)?1:CFG_STEP;
      analogWrite(DPIN_DRV_EN, currentOutput);
      delay(3);
    }
  } 
  else {
    currentOutput = targetOutput;
    analogWrite(DPIN_DRV_EN, currentOutput);
  }

  if(mode == MODE_OFF && currentOutput == 0)
    digitalWrite(DPIN_PWR, LOW); // turn power off when we're done.
}


void readAccel(char *acc)
{
  while (1)
  {
    Wire.beginTransmission(ACC_ADDRESS);
    Wire.write(ACC_REG_XOUT);
    Wire.endTransmission(false);       // End, but do not stop!
    Wire.requestFrom(ACC_ADDRESS, 3);  // This one stops.

    for (int i = 0; i < 3; i++)
    {
      if (!Wire.available())
        continue;
      acc[i] = Wire.read();
      if (acc[i] & 0x40)  // Indicates failed read; redo!
        continue;
      if (acc[i] & 0x20)  // Sign-extend
        acc[i] |= 0xC0;
    }
    break;
  }
}
/*
float readAccelAngle()
{
  char acc[3];
  readAccel(acc);
  return atan2(acc[0], acc[2]);
}
*/





