<div id="number1"></div>
<span style="color:darkred">

# NUMBER 1 - Convert xacro to urdf on jazzy  
</span>

## Context

If you want to convert .xacro to .urdf the <code>rosrun xacro</code> command is usually recommanded. However <code>rosrun</code> is reserved for noetic distro.



## Problem 
``` bash 
# error on terminal
$ rosrun
Command 'rosrun' not found, but can be installed with:
sudo apt install rosbash

42sh$ sudo apt install rosbash
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
E: Unable to locate package rosbash
```

## Solution
``` bash  
$ xacro vehicle.xacro > vehicule.urdf

# And  .urdf  to .sdf
$ gz sdf -p file.urdf > file.sdf
```

<div id="number2"></div>
<span style="color:darkred">

# NUMBER 2 - Fail to parse 
</span>

## Problem
``` bash
[robot_state_publisher-2] Error:   Error=XML_ERROR_PARSING_TEXT ErrorID=8 (0x8) Line number=1
[robot_state_publisher-2]          at line 101 in ./urdf_parser/src/model.cpp
[robot_state_publisher-2] Failed to parse robot description using: urdf_xml_parser/URDFXMLParser
[robot_state_publisher-2] terminate called after throwing an instance of 'std::runtime_error'
[robot_state_publisher-2]   what():  Unable to initialize urdf::model from robot description
```

## Issue
This is not a urdf file you need to first convert your xacro file to urdf file

To verify your urdf is correctly parsed use to following command:
```bash
$ check_urdf <path_to_file>
```

## Solution 
If file is <code>.xacro</code>, refer to issue **[NUMBER 1](#number1)**.

If the file is a <code>.urdf</code> and <code>check_urdf</code> works, verify your CMakeList and launch.py.

<div id="number3"></div>
<span style="color:darkred">

# NUMBER 3 - Robot not appearing in gazebo 
</span>

## Problem

Your robot is not appearing in gazebo but you have no compilation error.

## Solution

First, if you are using <code>.py</code> to launch make sure you have put the intended file in your launch file.

Otherwise, Gazebo needs the <code>inertial</code> tags for every link in your <code>.urdf</code>

Furthermore your world (<code>.sdf</code> file) needs to have a ground plane.


<div id="number4"></div>
<span style="color:darkred">

# NUMBER 4 - Requesting list of name 
</span>

## Problem:
You are running gazebo and you are blocked with a repetitive : 

```bash
Requesting list of name
```

## Solution: 

First make sure you have a world file and it is well integrated in the launch file.

Otherwise just redo everything from scratch honestly.

<div id="number5"></div>
<span style="color:darkred">

# NUMBER 5 - Gazebo keeping old instances of robot 
</span>

## Context

When you launch ros with gazebo you still have all your old instances of robots and your new instances is displayed on top of the others.

## Solution 

We are going to want to erase any file that is already compiled, like <code>/install</code>, <code>/log</code> ... Anything where the data of our old robots could be saved.

Try first to remove the <code>build/</code>, <code>log/</code>, <code>install/</code> directories.

If it does not change anything, re-delete these folders and reboot your computer.

In it still does not work, click on the <code>reset</code> button in the gazebo API (next to the <code>play</code> button).

However if after reseeting you are having troubles such as

``` bash
No clock received, using time argument instead! Check your node's clock configuration (use_sim_time parameter) and if a valid clock source is available
```

Or gazebo run but does not even show you the ground plance, I would recommend to uninstall and reinstall gazebo.

If you are using a VM et it still does not work or you have broken gazebo by uninstall/reinstalled it, create another virtual machine and re-install everything from scratch.

 