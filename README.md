# RET
## ROS Endurance Test.

Compare long term endurance of ROS application vs native Robot application. Ideally, the ROS App should not have more system level faults than the native app.

## RET Server
The RET Server waits for socket connections from the Robot and the Raspberry Pi. Currently only supports PRBT, but can be extended to handle UR later.

It also starts the Monitor thread, which has the following logic:   
* Wait for log from Robot - Blocking
* Wait for log from RPi - Blocking
* Evaluate the two log messages - if same button was pressed, and within the allowed time interval

### Expected Socket Log Format
The Monitor expects the log message in a very strict format from either the RPi or the Robot:   
`"<prbt/rpi>;<wall_time_float>;<1/2>"`   
Ex: 

The PRBT should log something like - `"prbt;1623456.1243;1"`   
The RPi should log something like - `"rpi;1623461.4321;1"`   

Furthermore, the **RPi HAS TO LOG THE SAME EVENT 0.5s AFTER THE ROBOT LOGS IT**. This is because of the aforementioned Monitor logic, which always expects the Robot log first, followed by RPi log second.

### Mock Test
A simple script that fires off some exemplary Socket logs from PRBT and RPi for testing the server.   
Change the host variable value inside both the scripts to switch between testing on a local machine or running on Pilz PC.   
`"169.254.60.100"` -> Used for running on Pilz environment     
`"192.168.56.1"` -> Used for running on UR environment     
`"127.0.0.1"` -> Used for testing on local machine   

### Bringup
CD to scripts folder, run the server script. Ex: `./ret_server.py`   
Similarly, to run the mock test script, `./mock_test.py`  (Optional arg: name of mock robot)


### TODOS
1. Monitor could evaluate more criteria, like if Button2 is pressed after Button1, time between Button2 and Button1 etc..

---

## RET Application

To run with UR5e Robot, requires the [`ur_manipulation`](https://github.com/ipa-kut/ur_manipulation) and its dependencies to be in the same workspace.   
To run with PRBT Robot, requires the [`ur_manipulation`](https://github.com/ipa-kut/ur_manipulation) and [`pilz_robots`](https://github.com/ipa-alb/pilz_robots) to be in the same workspace **Under debugging**

### Bringup (UR5e ROS Based test):

1. Follow the instructions of the robot packages to start the robot + moveit + rviz[link](https://github.com/ipa-kut/ur_manipulation#ur5e-real-robot).

2. Then start the [RET Server script](https://github.com/ipa-kut/ret#bringup) as described above - USE THE [CORRECT IP](https://github.com/ipa-kut/ret#mock-test) depending on the testing environment.

3. Start the Button Press Detection application on the Raspberry Pi.

4. Launch the RET Application - USE THE CORRECT LAUNCH PARAMETERS depending on the testing environment:   
`roslaunch ret ret_application.launch <args>`
for UR5e:
`roslaunch ret ret_application.launch robot:=ur_ros sim:=false`

### Bringup (UR Native)

1. Start the [RET Server script](https://github.com/ipa-kut/ret#bringup) as described above

2. Start the `ret.urp` program on the UR5e Polyscope pendant

3. Start the Button Press Detection application on the Raspberry Pi.

4. The pendant may ask you to make the movement to starting position, press and hold the `Auto` option until it does. Then, press `Play` to start the loop.

### Bringup (prbt)
1. Connect pc with robot by both internet cable and USB, set the PCI connection to prbt with `IP: 169.254.60.100, Netmask: 255.255.255.0`, check the connection by `ping 169.254.60.100`
   
2. Set the can connection to robot by `sudo ip link set can0 up type can bitrate 1000000`, when reboot the robot, better to set the can down by `sudo ip link set can0 down` and bring it up again

3. Start the RET Server script as described above

4. launch the robot controller and rviz by `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=false pipeline:=ompl`

5. Launch ret application by `roslaunch ret ret_application robot:=prbt sim:=false`, add attribute `prompt:=true` if needed

### TODOS

- [x] Update the RET Application code from the current simple square movement logic into the complete button masher logic
- [ ] Extract the execution parameters (robot name, ip, port, button pose calues etc..) into params loaded from configuration files under the `config` folder -> Update configs for UR5e and PRBT : DONE ur part
- [ ] Test & update the application so that the same code works for UR5e and PRBT.
- [ ] Check the planning frame for ur for there's some pose difference between native_driver and ros_driver
- [ ] Feature: automatically return ready pose when socket connection failed?
- [ ] Feature: 

