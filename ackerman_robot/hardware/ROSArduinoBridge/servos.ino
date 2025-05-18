/***************************************************************
   Servo Sweep - by Nathaniel Gallinger

   Sweep servos one degree step at a time with a user defined
   delay in between steps.  Supports changing direction 
   mid-sweep.  Important for applications such as robotic arms
   where the stock servo speed is too fast for the strength
   of your system.

 *************************************************************/

#ifdef USE_SERVOS


// Constructor
SweepServo::SweepServo()
{
  this->currentPositionDegrees = 0;
  this->targetPositionDegrees = 0;
  this->lastSweepCommand = 0;
}


// Init
void SweepServo::initServo(
    int servoPin,
    int stepDelayMs,
    int initPosition)
{
  //Serial.print("DoSweep\r\n");
  this->servo.attach(servoPin);
  this->stepDelayMs = stepDelayMs;
  this->currentPositionDegrees = initPosition;
  this->targetPositionDegrees = initPosition;
  this->lastSweepCommand = millis();
}


// Perform Sweep
void SweepServo::doSweep()
{
  int delta = millis() - this->lastSweepCommand;
    
  if (delta > this->stepDelayMs) {
    this->lastSweepCommand =  millis(); 
    if (this->currentPositionDegrees < this->targetPositionDegrees)
    {
      for (; this->currentPositionDegrees < this->targetPositionDegrees; this->currentPositionDegrees++)
      {
        this->currentPositionDegrees += 1;
        this->servo.write(this->currentPositionDegrees);
        delay(10);
      }
    }
    else
    {
      for (; this->currentPositionDegrees > this->targetPositionDegrees; this->currentPositionDegrees--)
      {
        this->servo.write(this->currentPositionDegrees);
        delay(10);
      }
    }
  }
  
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
void SweepServo::setTargetPosition(int position)
{
  //Serial.print("Set\r\n");
  if (position >= 30 && position <= 150)
  {
    this->targetPositionDegrees = position;
  }
  
}


void SweepServo::updateTargetPosition(int position)
{
  int margin = position > 0? 1: -1;
  if (this->targetPositionDegrees + position + margin >= 0 && this->targetPositionDegrees + position + margin <= 180)
  {
     this->targetPositionDegrees += (position + margin);
  }
}


// Accessor for servo object
Servo SweepServo::getServo()
{
  //Serial.println("la\r\n");
  return this->servo;
}


#endif
