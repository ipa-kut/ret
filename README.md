# RET
## ROS Endurance Test.

Compare long term endurance of ROS application vs native Robot application. Ideally, the ROS App should not have more system level faults than the native app.

## Package Scripts Descritpion
### RET Server
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
`"127.0.0.1"` -> Used for testing on local machine   

## Launch
CD to scripts folder, run the server script. Ex: `./ret_server.py`   
Similarly, to run the mock test script, `./mock_test.py`   


## TODOS
1. Monitor could evaluate more criteria, like if Button2 is pressed after Button1, time between Button2 and Button1 etc..
