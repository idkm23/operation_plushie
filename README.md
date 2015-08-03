WHAT TO DO:
=====
1. Run ```cd ~/indigo/Baxter_ws``` and ```. ./src/operation_plushie/scripts/baxter_setup.sh``` in terminal to start the cameras.

2. If you do not hear a loud fan noise coming from Baxter or the fan noise ceases for whatever reason, then its motors are not enabled. Type ```rosrun baxter_tools enable_robot.py -e``` to enable it.

3. Type ```ssh leviathan``` in a new terminal to connect to leviathan (Eric's computer) (You will need the robot lab password to continue).

3. In the new leviathan terminal that you made, type ```roslaunch operation_plushie kinect_pc.launch``` to start the drivers for the Xtion Pro.

4. In the main terminal, type ```rosrun rqt_reconfigure rqt_reconfigure```. Then, click "drivers" on the side bar in the window that pops up and check the box that says "depth_registration".

5. In the main terminal, type ```roslaunch operation_plushie operation_plushie.launch``` to start up the program.

6. To end the program, click on the main terminal that is currently running "roslaunch operation_plushie operation_plushie.launch" and press CTRL+C to stop it. You can then use CTRL+C or CTRL+\ to stop the drivers on the leviathan terminal.

7. To disable Baxter's motors, start by moving Baxter's arms away from any tables or other objects. His arms will fall when they are disabled, so make sure they won't hit anything. Then, type ```rosrun baxter_tools enable_robot.py -d```.
