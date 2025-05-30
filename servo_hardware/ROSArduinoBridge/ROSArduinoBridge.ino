/*********************************************************************
    ROSArduinoBridge

    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org

    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson

    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

       Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
       Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
/* The Pololu VNH5019 dual motor driver shield */
//#define POLOLU_VNH5019

/* The Pololu MC33926 dual motor driver shield */
//#define POLOLU_MC33926

/* The RoboGaia encoder shield */
//#define ROBOGAIA

/* Encoders directly attached to Arduino board */
#define ARDUINO_ENC_COUNTER

/* L298 Motor driver*/
#define L298_MOTOR_DRIVER
#endif

#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
//#define USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     9600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
#include <Servo.h>
#include "servos.h"
#endif

#ifdef USE_BASE
/* Motor driver function definitions */
#include "motor_driver.h"

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "diff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE           30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);


  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
 /*   case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;*/
/*    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;*/
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case PING:
      Serial.println(Ping(arg1));
      break;
#ifdef USE_SERVOS
    case SERVO_WRITE:

      Serial.print("Servo command received: ");
      Serial.print(arg1);
      Serial.print(" ");
      Serial.println(arg2);
      servos[STEERING_RIGHT].setTargetPosition(arg1);
      servos[STEERING_LEFT].setTargetPosition(arg1);
      Serial.println("OK\n\r");
      break;
    case SERVO_UPDATE:
      servos[STEERING_RIGHT].updateTargetPosition(arg1);
      servos[STEERING_LEFT].updateTargetPosition(arg1);
      break;
    
 /*   case SERVO_READ:
      Serial.println(servos[arg1].getServo().read());
      break;*/
#endif
#ifdef USE_MOTORS
    case BRUSH_WRITE:
      motors[STEERING_RIGHT].updateTargetPosition(arg1);
      motors[STEERING_LEFT].updateTargetPosition(arg1);
      break;
#endif

#ifdef USE_BASE
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      }
      else moving = 1;
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK");
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      resetPID();
      moving = 0; // Sneaky way to temporarily disable the PID
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK");
      break;
 /*   case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      Serial.println("OK");
      break;*/
#endif
    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  // Initialize the motor controller if used */
#ifdef USE_BASE
#ifdef ARDUINO_ENC_COUNTER
  //set as inputs
  DDRD &= ~(1 << LEFT_ENC_PIN_A);
  DDRD &= ~(1 << LEFT_ENC_PIN_B);
  DDRC &= ~(1 << RIGHT_ENC_PIN_A);
  DDRC &= ~(1 << RIGHT_ENC_PIN_B);

  //enable pull up resistors
  PORTD |= (1 << LEFT_ENC_PIN_A);
  PORTD |= (1 << LEFT_ENC_PIN_B);
  PORTC |= (1 << RIGHT_ENC_PIN_A);
  PORTC |= (1 << RIGHT_ENC_PIN_B);

  // tell pin change mask to listen to left encoder pins
  PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
  PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);

  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
#endif
  initMotorController();
  resetPID();
#endif

  /* Attach servos if used */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(
      servoPins[i],
      stepDelay[i],
      servoInitPosition[i]);
  }
#endif
#ifdef USE_MOTORS
  int i;
  for (i = 0; i < N_MOTORS; i++) {
    motors[i].initMotor(
      motorPins[i],
      stepDelay[i],
      servoInitPosition[i]);
  }
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/

/*   DEBUGG */
Servo myservo;

void move_debug2()
{
  servos[0].setTargetPosition(180);
  servos[0].doSweep();
  delay(30);
}

void move_debug()
{
  myservo.attach(3);
  for (int pos = 0; pos <= 180; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
  for (int pos = 180; pos >= 10; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
}


#define dureeMinimaleImpulsionCommandeESC     1000        // La durée minimale pour une impulsion est de 1000 µs, soit 1 ms (comme pour un servomoteur, en fait)
#define dureeMaximaleImpulsionCommandeESC     2000   
#define pinPilotageESC             9 

Servo ESC; 
int first = 0;

void loop_temp2()
{
  if (first == 0)
  {
      ESC.attach(9,1000,2000);
      first = 1;
      
  }
  for (int i = 50; i  < 60; i++)
  {
    ESC.writeMicroseconds(i); 
    delay(1);
   }
  
   
}


void loop_temp() {

  if (Serial.available() > 0) {
    move_debug();
    String message = Serial.readString();
    message.trim();


    if (message.equals("ON")) {
      move_debug();
    }
  }
}

/* END DEBUG */


void loop() {
  while (Serial.available() > 0) {
    // Read the next character
    String msg  = Serial.readString();
    msg.trim();

    for (int i = 0; msg[i]; i++)
    {
      chr = msg[i];
      Serial.println("la");

      // Terminate a command with a CR
      if (chr == '\r') {

        if (arg == 1) {
          argv1[index] = NULL;
        }
        else if (arg == 2) {
          argv2[index] = NULL;
        }
        
        
        runCommand();
        resetCommand();
        // Sweep servos
        #ifdef USE_MOTORS
      
        for (int i = 0; i < N_SERVOS; i++) {
          servos[i].doSweep();
          motors[i].doSweep();
        }
        #endif
      }
      // Use spaces to delimit parts of the command
      else if (chr == ' ') {
        // Step through the arguments
        if (arg == 0) arg = 1;
        else if (arg == 1)  {
          argv1[index] = NULL;
          arg = 2;
          index = 0;
        }
        continue;
      }
      else {
        if (arg == 0) {
          // The first arg is the single-letter command
          cmd = chr;
        }
        else if (arg == 1) {
          // Subsequent arguments can be more than one character
          argv1[index] = chr;
          index++;
        }
        else if (arg == 2) {
          argv2[index] = chr;
          index++;
        }
      }
    }

      
  }

  // If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if  we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    ;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif
}
