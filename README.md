IMPORTANT!
=====
The following instructions are meant to be used on the robot-lab5 and leviathan computers, with the Xtion Pro plugged into the leviathan. This is also assuming that all of the packages are still on those computers. If this is not the case, see below for more details.

This program is also currently set up to use Baxter's left arm, though hardcoded values can be adjusted to use its right arm.

WHAT TO DO:
=====
1. Move to the root of the workspace containing the operation_plushie package (Run ```cd ~/indigo/Baxter_ws``` if you're using robot-lab5) and run ```. ./src/operation_plushie/scripts/baxter_setup.sh``` in terminal to start the cameras.

2. If you do not hear a loud fan noise coming from Baxter or the fan noise ceases for whatever reason, then its motors are not enabled. Type ```rosrun baxter_tools enable_robot.py -e``` to enable it.

3. In a new terminal, ssh into the computer that has the Xtion Pro plugged into it (If the setup hasn't changed, use ```ssh leviathan``` to connect to leviathan). You will need the robot lab password to continue. If the Xtion Pro is plugged into the computer you're currently using, then simply complete step 4 in a new terminal without using ssh.

4. In the terminal that is connected to the other computer, type ```roslaunch operation_plushie kinect_pc.launch``` to start the drivers for the Xtion Pro.

5. In the main terminal, type ```rosrun rqt_reconfigure rqt_reconfigure```. Then, click "drivers" on the side bar in the window that pops up and check the box that says "depth_registration".

6. In the main terminal, type ```roslaunch operation_plushie operation_plushie.launch``` to start up the program.

7. To end the program, click on the main terminal that is currently running "roslaunch operation_plushie operation_plushie.launch" and press CTRL+C to stop it. You can then use CTRL+C or CTRL+\ to stop the drivers on the leviathan terminal.

8. To disable Baxter's motors, start by moving Baxter's arms away from any tables or other objects. His arms will fall when they are disabled, so make sure they won't hit anything. Then, type ```rosrun baxter_tools enable_robot.py -d```.

STARTING FROM SCRATCH:
=====
If you are trying to run this on a brand new computer, then you will need the following:

  Packages
-----
  - This package
  - ROS Indigo (This has not been tested with other versions!)
  - OpenCV 2.4
  - CvBridge
  - PCL 1.7
  - Rethink SDK for Baxter
  
  Hardware
  -------
  - An Asus Xtion Pro that will be mounted on Baxter's chest
  - If the Xtion Pro is going to be plugged into a second computer, install all packages on that, as well  
  
AUTHORS
=====
Christopher Gibbs
Christopher Munroe

LAST UPDATED
=====
August 4, 2015
