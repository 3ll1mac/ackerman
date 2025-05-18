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
  this->currentPositionDegrees = 1500;
  this->targetPositionDegrees = 1500;
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
  Serial.println("DoSweep");
  Serial.println(this->currentPositionDegrees);
  Serial.println(this->targetPositionDegrees);
  
  if (this->currentPositionDegrees < this->targetPositionDegrees)
  {
    for (; this->currentPositionDegrees < this->targetPositionDegrees; this->currentPositionDegrees+=2)
    {
        Serial.println(this->currentPositionDegrees);
       this->motor.writeMicroseconds(this->currentPositionDegrees);  
       delay(3);
    }
  }else
  {
    for (; this->currentPositionDegrees > this->targetPositionDegrees; this->currentPositionDegrees-=2)
    {
       Serial.println(this->currentPositionDegrees);
       this->motor.writeMicroseconds(this->currentPositionDegrees);  
       delay(3);
    }
  }
}


// Set a new target position
void Motor::setTargetPosition(int position)
{
  //Serial.print("Set\r\n");
  this->targetPositionDegrees = position;
}


void Motor::updateTargetPosition(int position)
{
  /* Keep between 1000 and 2000 */
  if (position < -500)
    position = -500;
  if (position > 500)
    position = 500;
  this->targetPositionDegrees = 1500 + position;
  
  /*if (position < 0)
  {
    Serial.println("MINUS");
    this->targetPositionDegrees = 1200;
  }
  else if (position > 0)
  {   
    Serial.println("PLUS");
     this->targetPositionDegrees = 1700;
  }
  else
  {
    Serial.println("NEUTRE");
    this->targetPositionDegrees = 1500;
  }*/
}


// Accessor for servo object
Servo Motor::getMotor()
{
  //Serial.println("la\r\n");
  return this->motor;
}


#endif
