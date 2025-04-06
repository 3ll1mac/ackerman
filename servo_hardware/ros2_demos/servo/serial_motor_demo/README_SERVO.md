# Servos

## Run

Connect the arduino to your computer / Raspberry PI

First put the RosArduinoBridge content in your arduino.

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