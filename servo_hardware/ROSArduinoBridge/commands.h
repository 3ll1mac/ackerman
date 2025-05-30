/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'g'
#define BRUSH_WRITE 'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_UPDATE   'u'
//#define SERVO_READ     't'
//#define UPDATE_PID     'u'
//#define DIGITAL_WRITE  'w'
//#define ANALOG_WRITE   'x'
#define STEERING_RIGHT  0
#define STEERING_LEFT   1
#define LEFT            0
#define RIGHT           1

#endif
