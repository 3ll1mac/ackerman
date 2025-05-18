// #define USE_BASE      // Enable the base controller code
// #undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
/* The Pololu VNH5019 dual motor driver shield */
//#define POLOLU_VNH5019

/* The Pololu MC33926 dual motor driver shield */
//#define POLOLU_MC33926

/* The RoboGaia encoder shield */
//#define ROBOGAIA

/* Encoders directly attached to Arduino board */
//#define ARDUINO_ENC_COUNTER

/* L298 Motor driver*/
//#define L298_MOTOR_DRIVER
#endif

#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#define USE_MOTORS    // Disable use of PWM servos
        
/* Serial port baud rate */
#define BAUDRATE     115200
 

/* Maximum PWM signal */
//#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
// #include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
#include <Servo.h>
#include "servos.h"
#include "motors.h"
#endif

Servo esc;

#ifdef USE_BASE
/* Motor driver function definitions */
// #include "motor_driver.h"

/* Encoder driver function definitions */
//  #include "encoder_driver.h"

/* PID parameters and functions */
// #include "diff_controller.h"

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

char cmd; // Variable to hold the current single-character command
char argv1[16]; // Character arrays to hold the first and second arguments
long arg1; // The arguments converted to integers


/* Clear the current command parameters */
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  arg1 = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand(char *argv) {
  int i = 0;
  int pid_args[4];
  arg1 = atoi(argv);

  switch (cmd) {
#ifdef USE_SERVOS
    case SERVO_WRITE:
      servos[STEERING_RIGHT].setTargetPosition(arg1);
      servos[STEERING_LEFT].setTargetPosition(arg1);
      break;
    case SERVO_UPDATE:
      Serial.println("SERVO UPDATE");
      Serial.println(arg1);
      Serial.println(arg1);
      servos[STEERING_RIGHT].updateTargetPosition(arg1);
      servos[STEERING_LEFT].updateTargetPosition(arg1);
      break;
    
 /*   case SERVO_READ:
      Serial.println(servos[arg1].getServo().read());
      break;*/
#endif
#ifdef USE_MOTORS
    case BRUSH_WRITE:
      Serial.println("BRUSH WRITE");
      Serial.println(arg1);
      Serial.println(arg1);
      motors[RIGHT].updateTargetPosition(arg1);
      motors[LEFT].updateTargetPosition(arg1);
      break;
#endif
  }
}


/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
   
  /* Attach servos if used */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(
      servoPins[i],
      100,
      servoInitPosition[i]);
  }
#endif
#ifdef USE_MOTORS 
  //esc.attach(9,1000,2000);
 /* esc.writeMicroseconds(1000); // initialise the esc signal to low level
  delay(1000);                 // wait for 1 second
  esc.writeMicroseconds(2000); // then set it to high level
  delay(1000);                 // then wait again for 1 second
      esc.writeMicroseconds(1000);*/
  for (i = 0; i < N_MOTORS; i++) {
    motors[i].initMotor(
      motorPins[i],
      stepDelay[i],
      motorInitPosition[i]);
    
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
    delay(10);
  }
  for (int pos = 180; pos >= 10; pos -= 1) {
    myservo.write(pos);
    delay(10);
  }
}


#define dureeMinimaleImpulsionCommandeESC     1000        // La durée minimale pour une impulsion est de 1000 µs, soit 1 ms (comme pour un servomoteur, en fait)
#define dureeMaximaleImpulsionCommandeESC     2000   
#define pinPilotageESC             9 


void loop_Temp2()
{
  Serial.println("in loop");
  for (int i = 1300; i < 1600; i++)
  {
    #ifdef USE_MOTORS
        motors[LEFT].setTargetPosition(i);
        motors[RIGHT].setTargetPosition(i);


        for (int j = 0; j < N_SERVOS; j++) {
          Serial.println(i);
          motors[j].doSweep();
        }
        #endif
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



bool newData = false;

void loop() {
    static boolean recvInProgress = false;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    int index = 0;
    int arg = 0;
    char argv[50];
    
    String m = String();
    while (Serial.available() > 0) {
        rc = Serial.read();
        Serial.println("rc: ");
        Serial.println(rc);
 
        if (recvInProgress == true) {
          if (rc == ' ') {
            if (arg == 0) 
            {
              Serial.println("Space");
              arg = 1;
            }
               
          }
          else if (rc == endMarker) {
             if (arg == 1) {
                argv[index] = '\0';
                recvInProgress = false;
                arg = 0;
                Serial.println(argv);
                Serial.println(cmd);
                
                execute(argv);
                Serial.println("DONE\r\n");
              }
          }
          else {
            
             if (arg == 0) {
                Serial.println("command" + cmd);
                cmd = rc;
              }
              else {
                Serial.println("else " + rc);
                argv[index] = rc;                
                index++;
              }
          }   
      }else if (rc == startMarker) {
          recvInProgress = true;
          index = 0;
          arg = 0;
      }
  }
}



void execute(char *argv) {   
      runCommand(argv);
      resetCommand();

      #ifdef USE_MOTORS
      for (int i = 0; i < N_SERVOS; i++) {
        servos[i].doSweep();
        motors[i].doSweep();
      }
      #endif
        
}
 
  
