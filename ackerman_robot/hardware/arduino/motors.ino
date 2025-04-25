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
  this->motor.attach(motorPin, 1000, 2000);
  this->stepDelayMs = stepDelayMs;
  this->currentPositionDegrees = initPosition;
  this->targetPositionDegrees = initPosition;
  this->lastSweepCommand = millis();

  /*this->motor.writeMicroseconds(2000); // initialise the esc signal to low level
  delay(2000);                 // wait for 1 second
  this->motor.writeMicroseconds(1000); // then set it to high level
  delay(2000);                 // then wait again for 1 second
  this->motor.writeMicroseconds(2000);*/
}


// Perform Sweep
void Motor::doSweep()
{

  this->motor.writeMicroseconds(this->targetPositionDegrees);
  delay(10);
  Serial.print("DoSweep\n\r");
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


void Motor::updateTargetPosition(int position)
{
  /*if (this->targetPositionDegrees + position >= 1000 && this->targetPositionDegrees + position <= 2000)
  {
    this->targetPositionDegrees += position;
  }*/
  if (position < 0)
  {
    this->targetPositionDegrees = 1300;
  }
  else if (position > 0)
  {
     this->targetPositionDegrees = 1600;
  }
  else
  {
    this->targetPositionDegrees = 1500;
  }
}


// Accessor for servo object
Servo Motor::getMotor()
{
  return this->motor;
}


#endif
