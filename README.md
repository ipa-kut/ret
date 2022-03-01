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
`"169.254.60.100"` -> Used for running on Pilz PC   
`"192.168.56.1"` -> Used for running on Pilz PC   
`"127.0.0.1"` -> Used for testing on local machine   


### Bringup
CD to scripts folder, run the server script. Ex: `./ret_server.py`   
Similarly, to run the mock test script, `./mock_test.py`  


### TODOS
1. Monitor could evaluate more criteria, like if Button2 is pressed after Button1, time between Button2 and Button1 etc..

---

## RET Application

To run with UR5e Robot, requires the [`ur_manipulation`](https://github.com/ipa-kut/ur_manipulation) package to be in the same workspace.   
To run with PRBT Robot, requires the [TODO: Fill this part]

### Bringup:

1. Follow the instructions of the robot packagse to start the robot + moveit + rviz.    

2. Then start the RET Server script as described above - USE THE CORRECT IP depending on the testing environment.

3. Launch the RET Application - USE THE CORRECT LAUNCH PARAMETERS depending on the testing environment:   
`roslaunch ret ret_application <args>`

### TODOS

1. Update the RET Application code from the current simple square movement logic into the complete button masher logic
2. Extract the execution parameters (robot name, ip, port, button pose calues etc..) into params loaded from configuration files under the `config` folder -> Update configs for UR5e and PRBT
3. Test & update the application so that the same code works for UR5e and PRBT.


