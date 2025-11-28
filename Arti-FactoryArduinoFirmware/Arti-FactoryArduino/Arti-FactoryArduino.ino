#include <Arduino.h>

//Servo
#define USE_LIGHTWEIGHT_SERVO_LIBRARY // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
#define MAX_EASING_SERVOS 1

#define ENABLE_EASE_PRECISION
#include "ServoEasing.hpp"
#include "PinDefinitionsAndMore.h"
ServoEasing Servo1;
#define START_DEGREE_VALUE  45 // The degree value written to the servo at time of attach.


//Stepper Motor
const uint8_t STEP_PIN = 2;
const uint8_t DIRECTION_PIN = 3;
const uint8_t ENABLE_PIN = 4;
const uint8_t MS1_PIN = 5;
const uint8_t MS2_PIN = 6;
const uint16_t WAIT_TIME_MICROSECONDS = 100;
const uint16_t STOP_TIME = 1000;
const uint8_t SERVO_ENABLE_PIN = 42;

void blinkLED();

void MoveServo(float newPosition)
{
  Servo1.easeTo(newPosition);
  while(Servo1.isMoving())
  {
    delay(10);
  }
}

void setupServo()
{
  pinMode(SERVO_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_ENABLE_PIN, 1);
  
  
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));
#endif
    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
    if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
    //Can't do this until after we have the transitor to turn on and off power to the servo
    //if (Servo1.attach(SERVO1_PIN) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            blinkLED();
        }
    }
    // Wait for servo to reach start position.
    delay(500);
    Servo1.setSpeed(25);  // This speed is taken if no further speed argument is given.
}

void setupStepperMotor()
{
  //Setup Stepper Motor
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);

  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  //https://bttwiki.com/TMC2209.html#pinoutfunction
  //MS1 | MS2 | micro steps per step
  // 0  |  0  | 8 
  // 1  |  1  | 16
  // 1  |  0  | 32
  // 0  |  1  | 64
  digitalWrite(MS1_PIN, 0);
  digitalWrite(MS2_PIN, 1);

  digitalWrite(ENABLE_PIN, 0);
}

void setup() 
{
  pinMode(SERVO_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_ENABLE_PIN, 0);
  
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
    
  setupServo();
  //setup serial communication for command input
  
  setupStepperMotor();
  delay(1000);
  Serial.println("Setup Complete");
}
void loop() 
{
  bool exit=false;
  if (Serial.available() > 0) 
  {
    String command = Serial.readStringUntil('\n');

    if(command.charAt(0)=='S'||command.charAt(0)=='s')
    {
      String positionToMoveTo = command.substring(1);
      float pos = positionToMoveTo.toFloat();
      MoveServo(pos);
      Serial.print("Servo:");
      Serial.print("Completed Move To:");
      Serial.println(pos);
    }
    else if(command.charAt(0)=='M'||command.charAt(0)=='m')
    {
      if(command.length()>2)
      {
        String stepsToMoveTo = command.substring(2);
        unsigned long steps = stepsToMoveTo.toInt();
        if(command.charAt(1) == '+')
        {
          digitalWrite(DIRECTION_PIN, 1);
          delay(STOP_TIME);
          for(unsigned long i = 0; i < steps; i++)
          {
            digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
            delayMicroseconds(WAIT_TIME_MICROSECONDS);
            digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
            delayMicroseconds(WAIT_TIME_MICROSECONDS);
            
          }
          Serial.print("+");
          Serial.println(steps);
        }
        else if(command.charAt(1) == '-')
        {
          digitalWrite(DIRECTION_PIN, 0);
          delay(STOP_TIME);
          for(unsigned long i = 0; i < steps; i++)
          {
            digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
            delayMicroseconds(WAIT_TIME_MICROSECONDS);
            digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
            delayMicroseconds(WAIT_TIME_MICROSECONDS);
            
          }
          Serial.print("-");
          Serial.println(steps);
        }
        else
        {
          Serial.println("ERR: Stepper Bad command");
        }
      }
    }
    else
    {
      exit=true;
    }
    
  }
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}
