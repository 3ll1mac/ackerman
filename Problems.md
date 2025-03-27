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
$ xacro vehicle_xacro.xacro > vehicule_urdf.urdf

# And  .urdf  to .sdf
$ gz sdf -p file.urdf > file.sdf
```

<div id="number2"></div>
<span style="color:darkred">

# NUMBER 2 - Fail to parse 
</span>

## Problem
``` bash
# something like that 
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
However be carefull, if your file contains xacro command, check_urdf will mistake them for errors. 

## Answer 
Read all answers before acting.

<u>Answer 1</u>: If your file is a <code>.xacro</code>, refer to issue **[NUMBER 1](#number1)**.

<u>Answer 2</u>: If your file is a <code>.urdf</code>, but contains <code>xacro:macro</code>, refer **[NUMBER 1](#number1)**. But this time, replace <code>vehicule_xacro.xacro</code> by your <code>.urdf</code> file. This command will parse and calculate your calculs. 

<u>Answer 3</u>: Or, instead of answer 1 and 2, in your launch.py, instead of opening and parsing a <code>.urdf</code>, open and parse an <code>.xacro</code> file. Here is an example.

```bash
xacro_file = os.path.join(pkg_share, "src/description/two_wheels.xacro")
robot_desc = xacro.process_file(xacro_file).toxml()
```


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

First make sure you have a world file and it is well integrated in the launch file. In your launch file you need to have a gazebo and a spawn_robot node. Here is an example:

```base
world_file_name = 'world.sdf'
world_path = os.path.join(pkg_share, 'worlds', world_file_name)
  
# Gazebo launch
gazebo = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
          launch_arguments={'gz_args': ['-r ', world_path],
          'on_exit_shutdown': 'true'}.items()
          # launch_arguments={'world': world_path}.items(),
 )
 
# spawn robot by getting the description of the robot from the
# /robot_description topic 
spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=['-topic','robot_description',
        '-name', 'diffbot',
        '-z', '1.0'],
        output='screen'
)

ld.add_action(gazebo)
ld.add_action(spawn_entity)

```

For example, missing the gazebo node will cause the "requestions list of name" error.

Otherwise just redo everything from scratch honestly or use a working github repository to compare.

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

However if after reseting you are having troubles such as

``` bash
No clock received, using time argument instead! Check your node's clock configuration (use_sim_time parameter) and if a valid clock source is available
```

Or gazebo run but does not even show you the ground plance, I would recommend to uninstall and reinstall gazebo.

If you are using a VM and it still does not work or you have broken gazebo by uninstall/reinstalled it, create another virtual machine and re-install everything from scratch.

 