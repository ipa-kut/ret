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

To run with UR5e Robot, requires the [`ur_manipulation`](https://github.com/ipa-kut/ur_manipulation) package to be in the same workspace.   
To run with PRBT Robot, requires the [TODO: Fill this part]

### Bringup (UR5e ROS Based test):

1. Follow the instructions of the robot packagse to start the robot + moveit + rviz.    

2. Then start the RET Server script as described above - USE THE CORRECT IP depending on the testing environment.

3. Start the Button Press Detection application on the Raspberry Pi.

4. Launch the RET Application - USE THE CORRECT LAUNCH PARAMETERS depending on the testing environment:   
`roslaunch ret ret_application.launch <args>`
for UR5e:
`roslaunch ret ret_application.launch robot:=ur_ros`

### Bringup (UR Native)

1. Start the RET Server script as described above

2. Start the `ret.urp` program on the UR5e Polyscope pendant

3. Start the Button Press Detection application on the Raspberry Pi.

4. It may ask you to make the movement to starting position, press and hold the `Auto`option until it does. Then, press `Play` to start the loop.

### Database structure

#### now : two measurements with timestamp
- RET_Logs_<Datetime>
|time|button::field|datetime::field|source::tag|
|----|----|----|----|
|server time|button NO.|time mashing the button|data source: robot / rpi|

- RET_EVENTS_<Datetime>
|time|description::field|type::tag|
|----|----|----|
|server time|event description|mismatch / timeout|

- Comments:
 1. (-) When retrieving data for GUI, the measurement needs to be switched manuelly for each panel
 2. (-) Tag value can't be selected alone, not able to show current working robot or the data source
 3. (-) Not enough for analysing error when one occurs: maybe data sent both from robot and rpi

##### Suggesting: two measurement without timestamp
- RET_Logs
|time|button::field|datetime::field|source::tag|source::field|button::tag|
|----|----|----|----|----|----|
|server time|button NO.|time mashing the button|data source: robot / rpi|data source to be selected|button NO. for where clause|

- RET_EVENTS
|time|description::field|type::tag|
|----|----|----|----|
|server time|event description|mismatch / timeout|

- Comments:
 1. No need to switch measurements everyday
 2. All values can be selected alone
 3. Maybe the measurement is too large to maintain?
 4. In Grafana, the where clause does not support field value
 5. (-) Not enough for analysing error when one occurs: maybe data sent both from robot and rpi 


### TODOS

- [x] Update the RET Application code from the current simple square movement logic into the complete button masher logic
- [ ] Extract the execution parameters (robot name, ip, port, button pose calues etc..) into params loaded from configuration files under the `config` folder -> Update configs for UR5e and PRBT : DONE ur part
- [ ] Test & update the application so that the same code works for UR5e and PRBT.
- [ ] Check the planning frame for ur for there's some pose difference between native_driver and ros_driver
- [ ] Feature: automatically return ready pose when socket connection failed?
- [ ] Database structure

