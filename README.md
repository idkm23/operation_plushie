WHAT TO DO:
1. run "cd ~/indigo/Baxter_ws" and ". ./src/operation_plushie/scripts/baxter_setup.sh" in terminal
2. roslaunch operation_plushie kinect_pc.launch on the pc with the kinect plugged into it (leviathan)
3. rosrun rqt_reconfigure rqt_reconfigure (click "drivers" on side bar in window that pops up and check the box that says "depth_registration")
4. the usual (roslaunch operation_plushie operation_plushie.launch)
