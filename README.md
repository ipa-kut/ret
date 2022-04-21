# RET - ROS Endurance Test.

Compare long term endurance of ROS application vs native Robot application. Ideally, the ROS application should not have more system level faults than the native application.

This repository contains code for the Button Masher Application (BMA)* for ROS + Moveit under `ret/src` and the code for the RET server* under `ret/scripts`.    
\* terms explained below

Currently supports the following robots:
* Pilz PRBT - ROS only
* Universal Robots UR5e - ROS and Native

## Experimental setup

* Robot that runs the BMA on ROS and/or natively. The BMA presses two buttons alternatively in an infinite loop and sends socket log messages to the RET server whenever it thinks it has pressed a specific button.
  *  If ROS, driver+moveit+BMA(C++) runs on experiment PC. 
  *  If native, BMA(native code) runs from a teach pendant/IPC etc..
  *  If a robot supports both, only one is run at a time per experiment
*  Raspberry Pi which runs a Button Press Detectopn (BPD) python application which detects whenever a button is pressed and sends this confirmation to the RET server via socket log messages.
*  RET server which is a python application running on the experiment PC. Creates a socket server that receives socket log messages from Robot and Raspberry Pi. 
  *  Runs some evaluation logic to check for faults based on the logs.
  *  Saves all log messages into InfluxDB which is also visualised in Grafana.

---

## RET Server
Run this first since BMA and BPD run socket clients that attempt to connect to this socket server first.

It also starts the Monitor thread, which has the following logic:   
* Wait for log from Robot - Blocking
* Wait for log from RPi - Blocking
* Evaluate the two log messages - if same button was pressed, and within the allowed time interval

### Socket log messagse format
All socket log messages sent to the server from any client must adhere to a very strict format:   
`"<source>;<wall_time>;<button_number>"`   
* `source`: The client that sends the log (robot or RPi)
* `wall_time`: The time at which the log was sent (epoch time)
* `button_number`: Number of the button button which was pressed  (1/2)

Ex:    

The PRBT client should log something like - `"prbt;1623456.1243;1"`   
The RPi client should log something like - `"rpi;1623461.4321;1"`   
The UR5e-ROS client should log something like - `"ur_ros;1623461.4321;1"`   
The UR5e-Native client should log something like - `"ur_native;1623461.4321;1"`   

Furthermore, the **RPi HAS TO LOG THE SAME EVENT 0.5s AFTER THE ROBOT LOGS IT**. This is because of the aforementioned Monitor logic, which always expects the Robot log first, followed by RPi log second.

### Mock Test
A simple script that fires off some exemplary Socket logs from PRBT and RPi for testing the server.   
  

### Bringup (Server)
Navigate to `ret/scripts` folder, run the server script. Ex: `./ret_server.py`      
  - Edit the script manually to use (i.e. uncomment) the correct IP for the server host variable [in this line](https://github.com/ipa-kut/ret/blob/229535458c36d8d7198e76306538690e33c0ffbb/scripts/ret_server.py#L36) depending on the testing environment  

### Bringup (Mock self test)

Navigate to `ret/scripts` folder, run the mock script. Ex: `./mock_test.py`  (Optional arg: any name for the mock robot)   
  - Edit the script manually to use (i.e. uncomment) the correct IP for the server host variable [in this line](https://github.com/ipa-kut/ret/blob/229535458c36d8d7198e76306538690e33c0ffbb/scripts/mock_test.py#L6) to switch between testing environments

### Todos
- [ ] Pass args to the server script to pick the correct IP automatically
- [ ] Add more evaluators

---

## RET Button Press Detection (BPD)

This is to be started on the Raspberry Pi after the RET server is up.  

### Bringup 
The application and its bringup instructions are available in the following repository: [link](https://github.com/ipa-alb/Rpi_ButtonDetection)

---

## RET Button Masher Application (BMA)

This is also to be started after the RET server is up.

### Requirements

Varies depending on the experimental setup:

* UR5e-ROS requires the [`ur_manipulation`](https://github.com/ipa-kut/ur_manipulation) package and its dependencies in the same workspace.   
* UR5e-Native has no further requirements
* Pilz PRBT requires the [`ur_manipualtion`](https://github.com/ipa-kut-cl/ur_manipulation) and [`pilz_robots`](https://github.com/IPA-KUT-CL/pilz_robots) in the same workspace.

### Configuration

Each testing environment has a unique configuration, and these are saved per enviornment under `ret/config`. Add/Edit the values here accordingly as needed.

### Bringup (UR5e ROS Based test):

1. Start the RET Server script [as described above ](https://github.com/ipa-kut/ret#bringup) 

2. Follow the instructions to start the robot + moveit + rviz: [link](https://github.com/ipa-kut/ur_manipulation#ur5e-real-robot).

3. Start the Button Press Detection application on the Raspberry Pi as described above.

4. Launch the RET Application:     

`roslaunch ret ret_application.launch robot:=ur_ros sim:=false`   
* Add `prompt:=true` if needed. This pauses execution for every new trajectory to get user confirmation in Rviz. Useful for debugging.

### Bringup (UR Native)

1. Start the RET Server script [as described above ](https://github.com/ipa-kut/ret#bringup) 

2. Start the `ret.urp` program on the UR5e Polyscope pendant. A copy of this program is saved under `ret/urscripts`

3. Start the Button Press Detection application on the Raspberry Pi as described above.

4. The pendant may ask you to make the movement to starting position, press and hold the `Auto` option until it does. Then, press `Play` to start the loop.

### Bringup (prbt)
1. Start the RET Server script [as described above ](https://github.com/ipa-kut/ret#bringup) 

2. Follow the instructions from [`pilz_robots`](https://github.com/IPA-KUT-CL/pilz_robots#on-robot)

3. Launch ret application   
   ```roslaunch ret ret_application.launch robot:=prbt sim:=false```    
  * Add `prompt:=true` if needed. This pauses execution for every new trajectory to get user confirmation in Rviz. Useful for debugging.

### Simulation (prbt)

1. Start the RET Server script [as described above ](https://github.com/ipa-kut/ret#bringup) 

2. Follow the instructions from [`pilz_robots`](https://github.com/IPA-KUT-CL/pilz_robots#simulation)
   
3. Launch the ret application by `roslaunch ret ret_application.launch robot:=prbt sim:=true`, add attribute `prompt:=true` if needed

### TODOS
- [ ] Feature: automatically return ready pose when socket connection failed?

---

## Results logging
 
All socket log messages are stored in InfluxDB by the RET Server in the Measurement `RET_Logs`.   
Evaluation logic in the RET Server Monitor can also log failures as events in the measurement `RET_Events`

#### InfluxDB data storage format
- RET_Logs

    |time|button::field|datetime::field|source::tag|source::field|button::tag|
    |----|----|----|----|----|----|
    |server time|button NO.|time mashing the button|data source: robot / rpi|data source to be selected|button NO. for where clause|

- RET_EVENTS
  
    |time|description::field|type::tag|
    |----|----|----|
    |server time|event description|mismatch / timeout|

### Grafana GUI panel
1. Check grafana server status: `sudo service grafana-server status`
2. If the server failed, restart by `sudo service grafana-server restart`
3. Open `localhost:3000` in browser and log in
4. Select RET_Panel

![RET_Panel](./media/Screenshot%20from%202022-04-06%2015-22-48.png)

### Todos

-[] Export the Grafana panel .json and save it in this repo under `ret/config`

