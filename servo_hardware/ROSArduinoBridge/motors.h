#ifndef MOTORS_H
#define MOTORS_H


#define N_MOTORS 2

// This delay in milliseconds determines the pause 
// between each one degree step the servo travels.  Increasing 
// this number will make the servo sweep more slowly.  
// Decreasing this number will make the servo sweep more quickly.
// Zero is the default number and will make the servos spin at
// full speed.  150 ms makes them spin very slowly.


// Pins
byte motorPins [N_MOTORS] = { 9, 8};

// Initial Position
byte motorInitPosition [N_MOTORS] = { 1500, 1500};

class Motor
{
  public:
    Motor();
    void initMotor(
        int motorPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    void updateTargetPosition(int position);
    Servo getMotor();

  private:
    Servo motor;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

Motor motors [N_MOTORS];

#endif
