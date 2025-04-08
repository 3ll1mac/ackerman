# Servos

## RosArduinoBridge

The main file here is the <code>RosArduinoBridge</code> file. IN this file, you will find the <code>setup</code> and <code>loop</code> functions. 

### Loop function

After one message is sent to the arduino, the Serial will detect your presence and  you will enter the <code>Serial.available</code> while loop. After that it will, character by character parse the string you sent.
   
Each string sent has the following format <code>"(a|b|c|de|m|o|p|r|s|t|u|w|x) \<arg1\> \<arg2\>\r"</code>. For more information about which letter is linked to which command, please take a look at the <code>commands.h</code> file.  
For instance, to send the command to move the servo number 1 to 180 degree, the sent string would be: <code>"s 0 180\r"</code>

The <code>'\r'</code> is important because, the arduino received the message character by character and stop only when <code>'\r'</code> is received. 

When the carriage return character is received, the loop function calls the run command, which matched the first letter given to an action.   
For instance, with the last exemple, it will set the destination to 180 degree.

Thus, when the <code>doSweep</code> function is called at the end of the loop function, it will move the servo motor to the 180 degree.

### Servos file

To see the functions related to the servo motor, take a look at the <code>servos</code> file. The function are pretty explainitory by themselves.


### Note for all the others file

For now, which consist at just moving one servo, the encoders, sensors, and motor drivers are not necessary.


## ros2_demos/servo

### driver.py

Here we are going to explain what is happening the the <code>driver.py</code> file.

It first initialize the <code>MotorDriver</code> instance. The primary function called is the <code>servo_keyboard()</code> function which will, wait for your input (<code>k</code> or <code>i</code>), adapt the wanted angle, send it to be formated and sent to the arduino. 


## Run

### Setup Arduino 

Connect the arduino to your computer / Raspberry PI

First put the <code>RosArduinoBridge</code> content in your arduino.
Connect your servo motor to your arduino uno with the black thread to the ground, red one to the 5V pin and the orange one to the number 3 digital pin.


### Setup PI

Then, go to the <code>ros2_demos/servo/serial_motor_demo
</code> directory. Here type the following command.

```bash
$ colcon build
$ . install/setup.bash
$ ros2 run serial_motor_demo driver --ros-args -p encoder_cpr:=3440 \ -p loop_rate:=30 -p serial_port:=/dev/ttyACM1 -p baud_rate:=9600
```

To move "forward" the robot, type <code>i</code>.
To move "backward" the robot, type <code>k</code>.

If everything works as intended, you should see the servo motor move a little bit to the right/left depending on input.

Note: the code was based on another project, not every part of the code is used, but it was not removed as it could be useful for future project / amelioration.

# Possible Issue

If you can't seem to communicate, please, make sure you put <code>9600</code> as baudrate on the pycode, the command and your arduino code.