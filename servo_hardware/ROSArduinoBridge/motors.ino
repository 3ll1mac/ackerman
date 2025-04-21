#ifdef MOTORS_H


// Constructor
Motor::Motor()
{
  this->currentPositionDegrees = 0;
  this->targetPositionDegrees = 0;
  this->lastSweepCommand = 0;
}


// Init
void Motor::initMotor(
    int motorPin,
    int stepDelayMs,
    int initPosition)
{
  this->motor.attach(motorPin);
  this->stepDelayMs = stepDelayMs;
  this->currentPositionDegrees = initPosition;
  this->targetPositionDegrees = initPosition;
  this->lastSweepCommand = millis();
}


// Perform Sweep
void Motor::doSweep()
{

  this->motor.writeMicroseconds(this->targetPositionDegrees);
  // Get ellapsed time
/*  int delta = millis() - this->lastSweepCommand;

  // Check if time for a step
  if (delta > this->stepDelayMs) {
    // Check step direction
    if (this->targetPositionDegrees > this->currentPositionDegrees) {
      this->currentPositionDegrees++;
      this->servo.write(this->currentPositionDegrees);
    }
    else if (this->targetPositionDegrees < this->currentPositionDegrees) {
      this->currentPositionDegrees--;
      
      this->servo.write(this->currentPositionDegrees);
    }
    // if target == current position, do nothing

    // reset timerthis->servo.write(this->currentPositionDegrees);
    this->lastSweepCommand = millis();
  }*/
}


// Set a new target position
void Motor::setTargetPosition(int position)
{
  this->targetPositionDegrees = position;
}


void Motor::modifyTargetPosition(int position)
{
  this->targetPositionDegrees += position;
}


// Accessor for servo object
Servo Motor::getServo()
{
  return this->motor;
}


#endif
