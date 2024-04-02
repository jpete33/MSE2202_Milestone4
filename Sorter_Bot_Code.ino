// 
//  MSE 2202 Sorter Bot 
// 
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//  Author:   Jordan Peters
//  Date:     2024 03 31 
//

#define PRINT_COLOUR                                

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);     
void ARDUINO_ISR_ATTR encoderISR(void* arg);

// Encoder structure
struct Encoder {
   const int chanA;                                                            // GPIO pin for encoder channel A
   const int chanB;                                                            // GPIO pin for encoder channel B
   long pos;                                                                   // current encoder position
};

// Port pin constants
#define   CONVEYOR_MOTOR_A              35                                     // GPIO35 pin 28 (J35) Conveyor Motor A
#define   CONVEYOR_MOTOR_B              36                                     // GPIO36 pin 29 (J36) Conveyor Motor B
#define   DISC_MOTOR_A                  37                                     // GPIO35 pin 30 (J37) Disc Motor A
#define   DISC_MOTOR_B                  38                                     // GPIO36 pin 31 (J38) Disc Motor B
#define   CONVEYOR_ENCODER_A            15                                     // Conveyor Encoder A signal is connected to pin 8 GPIO15 (J15)
#define   CONVEYOR_ENCODER_B            16                                     // ConveyorEncoder B signal is connected to pin 9 GPIO15 (J16)
#define   DISC_ENCODER_A                11                                     // Disc Encoder A signal is connected to pin 19 GPIO15 (J11)
#define   DISC_ENCODER_B                12                                     // Disc Encoder B signal is connected to pin 20 GPIO15 (J12)

// Constants
const int cHeartbeatInterval = 75;                                             // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                                             // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                                              // number of Smart LEDs in use
const int cSDA               = 47;                                             // GPIO pin for I2C data
const int cSCL               = 48;                                             // GPIO pin for I2C clock
const int cTCSLED            = 14;                                             // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                                             // DIP switch S1-2 controls LED on TCS32725    
const int cServoPin          = 41;                                             // GPIO pin for servo motor
const int cServoChannel      = 5;                                              // PWM channel used for the RC servo motor
const int cNumMotors = 2;                                                      // number of DC motors
const int cIN1Pin[] = {CONVEYOR_MOTOR_A, DISC_MOTOR_A};                        // GPIO pin(s) for INT1
const int cIN1Chan[] = {0, 1};                                                 // PWM channe(s) for INT1
const int c2IN2Pin[] = {CONVEYOR_MOTOR_B, DISC_MOTOR_B};                       // GPIO pin(s) for INT2
const int cIN2Chan[] = {2, 3};                                                 // PWM channel(s) for INT2
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed
const int cPWMFreq = 20000;                                                    // frequency of PWM signal


// Variables
boolean heartbeatState       = true;                                           // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                                              // time of last heartbeat state change
unsigned long curMillis      = 0;                                              // current time, in milliseconds
unsigned long prevMillis     = 0;                                              // start time for delay cycle, in milliseconds
int servoPos;                                                                  // servo position

// Declare SK6812 SMART LED object
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                                              // TCS34725 flag: 1 = connected; 0 = not found

Encoder encoder[] = {{CONVEYOR_ENCODER_A, CONVEYOR_ENCODER_B, 0},              // left encoder, 0 position 
                     {DISC_ENCODER_A, DISC_ENCODER_B, 0}};                     // right encoder, 0 position   


void setup() {
  Serial.begin(115200);                                                        // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                                           // initialize smart LEDs object
  SmartLEDs.clear();                                                           // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0));                          // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                                                  // set brightness [0-255]
  SmartLEDs.show();                                                            // update LED
  
  // Set up servo
  pinMode(cServoPin, OUTPUT);                                                  // configure servo GPIO for output
  ledcSetup(cServoChannel, 50, 14);                                            // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(cServoPin, cServoChannel);                                     // assign servo pin to servo channel
  
  // Set up motors and encoders
  for (int k = 0; k < cNumMotors; k++) {
      ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);                                  // attach INT1 GPIO to PWM channel
      ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);                               // configure PWM channel frequency and resolution
      ledcAttachPin(c2IN2Pin[k], cIN2Chan[k]);                                 // attach INT2 GPIO to PWM channel
      ledcSetup(cIN2Chan[k], cPWMFreq, cPWMRes);                               // configure PWM channel frequency and resolution
      pinMode(encoder[k].chanA, INPUT);                                        // configure GPIO for encoder channel A input
      pinMode(encoder[k].chanB, INPUT);                                        // configure GPIO for encoder channel B input

      // configure encoder to trigger interrupt with each rising edge on channel A
      attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }

  Wire.setPins(cSDA, cSCL);                                                    // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                                                    // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                                           // configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}

void loop() {
  uint16_t r, g, b, c;                                                         // RGBC values from TCS34725
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));                             // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                                               // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                                            // get raw RGBC values
#ifdef PRINT_COLOUR            
      Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif
  }
   tcs.getRawData(&r, &g, &b, &c);

  // Set desired servo position based colour
  if (g > r && g > b) {
    servoPos = 180;                                                              // if green marble detected, set to desired servo angle
  } else {
    servoPos = 30;                                                               // if non-green marble detected, set servo to resting angle
  }

// Update servo position
ledcWrite(cServoChannel, degreesToDutyCycle(servoPos));
  doHeartbeat();                                                                 // update heartbeat LED
  setMotor(1, 255, cIN1Chan[0], cIN2Chan[0]);                                    // spin conveyor belt
  setMotor(1, 255, cIN1Chan[1], cIN2Chan[1]);                                    // spin disc
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                                                          // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                                                   // update the heartbeat time for the next update
    LEDBrightnessIndex++;                                                        // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {                      // if all defined levels have been used
      LEDBrightnessIndex = 0;                                                    // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);            // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(255, 0, 127));                    // set pixel colours to pink 
    SmartLEDs.show();                                                            // update LED
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
   if (dir == 1) {                                                               // forward
      ledcWrite(in1, pwm);
      ledcWrite(in2, 0);
   }
   else if (dir == -1) {                                                         // reverse
      ledcWrite(in1, 0);
      ledcWrite(in2, pwm);
   }
   else {                                                                        // stop
      ledcWrite(in1, 0);
      ledcWrite(in2, 0);
   }
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
   Encoder* s = static_cast<Encoder*>(arg);                                      // cast pointer to static structure
  
   int b = digitalRead(s->chanB);                                                // read state of channel B
   if (b > 0) {                                                                  // high, leading channel A
      s->pos++;                                                                  // increase position
   }
   else {                                                                        // low, lagging channel A
      s->pos--;                                                                  // decrease position
   }
}

long degreesToDutyCycle(int deg) {
  const long cMinDutyCycle = 400;                                                // duty cycle for 0 degrees
  const long cMaxDutyCycle = 2100;                                               // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);               // convert to duty cycle

#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0061039;                                         // (dutyCycle / 16383) * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif

  return dutyCycle;
}