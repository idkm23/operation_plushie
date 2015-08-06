Purpose
=====
This program allows Baxter to pick up a plush toy from a bowl and hand it to a detected person. Please note that it only tracks these specific toys and tries to find a bowl of them first.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
![Logo](https://github.com/idkm23/operation_plushie/blob/master/res/plush_toy.png)
&nbsp;&nbsp;&nbsp;&nbsp;
![Logo](https://github.com/idkm23/operation_plushie/blob/master/res/Plushie%20Bowl.jpg)

Important!
=====
The following instructions are meant to be used on the robot-lab5 and leviathan computers, with the Xtion Pro plugged into the leviathan. This is also assuming that all of the packages are still on those computers. If this is not the case, see below for more details.

This program is also currently set up to use Baxter's left arm. Although some hardcoded values can be adjusted to use its right arm, it would be better to set it up so that Baxter is ambidexterous when performing this task. 

Directions
=====
1. Move to the root of the workspace containing the operation_plushie package (Run ```cd ~/indigo/Baxter_ws``` if you're using robot-lab5) and type ```. ./src/operation_plushie/scripts/baxter_setup.sh``` in the terminal to start the cameras.

2. If you do not hear a loud fan noise coming from Baxter or the fan noise ceases for whatever reason, then its motors are not enabled. Type ```rosrun baxter_tools enable_robot.py -e``` to enable it.

3. In a new terminal, ssh into the computer that has the Xtion Pro plugged into it (If the setup hasn't changed, use ```ssh leviathan``` to connect to leviathan). You will need the robot lab password to continue. If the Xtion Pro is plugged into the computer you're currently using, then simply complete step 4 in a new terminal without using ssh.

4. In the terminal that is connected to the other computer, type ```roslaunch operation_plushie kinect_pc.launch``` to start the drivers for the Xtion Pro.

5. In the main terminal, type ```rosrun rqt_reconfigure rqt_reconfigure```. Then, click "drivers" on the side bar in the window that pops up and check the box that says "depth_registration".

6. In the main terminal, type ```roslaunch operation_plushie operation_plushie.launch``` to start up the program.

7. To end the program, click on the main terminal that is currently running "roslaunch operation_plushie operation_plushie.launch" and press CTRL+C to stop it. You can then use CTRL+C or CTRL+\ to stop the drivers on the leviathan terminal.

8. To disable Baxter's motors, start by moving Baxter's arms away from any tables or other objects. His arms will fall when they are disabled, so make sure they won't hit anything. Then, type ```rosrun baxter_tools enable_robot.py -d```.

Starting From Scratch
=====
If you are trying to run this on a brand new computer, then you will need the following:

Package Dependencies
-----
  - This package
  - ROS Indigo (This has not been tested with other versions!)
  - OpenCV 2.4
  - CvBridge
  - PCL 1.7
  - Rethink SDK for Baxter
  
Hardware
-------
  - An Asus Xtion Pro that will be mounted on Baxter's chest.
  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;![Logo](https://github.com/idkm23/operation_plushie/blob/master/res/xtion_pro_chest_image.png)

  - If the Xtion Pro is going to be plugged into a second computer, install all packages on that, as well  

Troubleshooting
=====
Here's a list of the most common errors and their solutions:

  - Although sourcing is done in one of the bash scripts that you ran in step 1, if packages can't be found, it may be           necessary to head to the root of the workspace and run ```source devel/setup.bash```.

  - If you get error messages saying that you could not connect to ROS_MASTER, then check your environment variables with       the following: run ```env | grep ROS``` and look at ROS_MASTER_URI and ROS_IP. ROS_MASTER_URI should be set to               "http://baxter.local:11311" or "http://baxter.lan:11311" and ROS_IP should be set to your IP address (use ```ifconfig```     to find this. Use ```export ROS_<MASTER_URI/IP>=<whatever it should be set to>```.
  
  - When running the Kinect Drivers (used for the Xtion Pro) on leviathan, it may provide an error message saying "Couldn't     find an AF_INET address for [baxter.lan]." Try the following: ```export ROS_MASTER_URI=http://baxter.local:11311```
  
  - If the gripper fails to close after starting the program and Baxter begins to skip stages, then this is a problem with       the gripper's state. If you run ```rostopic echo /robot/end_effector/left_gripper/state```, you will see that almost all     of its binary parameters are set to 2 (likely an error state). We have no solution for this or any idea why this occurs.     The problem stopped once we reset Baxter once or twice, but we cannot confirm that to be a solution.

Future Work
=====
In the res folder, there should be a file called "haarcascade_plushie.xml". This is not being used at the moment. It is a trained classifier that allows the computer to (somewhat) detect the plush toys. The goal was to try and use this to replace the color tracking, however,it has not been implemented.

We are also currently trying to get this to work with both arms so that, when one arm cannot reach the bowl, the other one will be used instead. However, in order to get ambidexterity to work, you ~~must~~ find a way to dynamically open and close Baxter's cameras (i.e. during the run time), since only two of its cameras can be open at a time!
  
Authors
=====
Christopher Gibbs

Christopher Munroe

Last Updated
=====
August 4, 2015
