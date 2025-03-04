# Safety Controller readme.md

## File Overview
**safety_controller.py:** Node definition for the actual safety controller. In summary, it combines the most recent LaserScan data with the most recent AckermannDrive data to determine whether the robot is in danger of crashing. If so, it will send an AckermannDrive command with velocity 0 (stop the robot).

**test_safety_controller.py:** Node definition for a basic node that sends AckermannDrive commands of a constant velocity at 20Hz until the robot is safety-stopped.
## How to Test
1. Open rviz in your browser via http://localhost:6080/vnc.html?resize=remote
2. Start the simulation. Any simulation with walls should work but I've been using
```
ros2 launch racecar_simulator simulate.launch.xml
```
2. Start the safety controller with required params:
    1. scan_topic: Where the robot sends AckermannDrive commands
    2. laser_topic: Where the robot reads LaserScan messages from
    3. drive_topic: Where the robot sends its E-STOP (still AckermannDrive) commands
```
ros2 run safety_controller safety_controller --ros-args -p scan_topic:=/drive -p laser_topic:=/scan -p drive_topic:=/drive
```
3. Use "Pose" to point the robot in the direction you want it to drive (it will just go straight in that direction). Ideally, you would point it at a nearby wall.
3. Whenever you want to test, start `test_safety_controller`. This will send "go forward" commands until the robot is stopped by the safety controller. You will have to restart the node after it is stopped by the safety controller in order to make the robot start moving again.
    1. scan_topic: Where the robot reads LaserScan messages from
    2. drive_topic: Where the robot sends its E-STOP commands
    3. velocity: How fast the robot moves forward
```
ros2 run safety_controller test_safety_controller --ros-args -p scan_topic:=/scan -p drive_topic:=/drive -p velocity:=2.0
```

## What to Edit
There are a few parameters that can be "tuned" to improve the safety controller's performance

| Parameter  | Units | Desc |
| ------------- |:-------------:|-------|
| DT      | sec     | Used to calculate project position at the next timestep (via drive command's velocity * DT)
| ang_range      | rad     | Cone of scan data that the robot considers, specifically the drive command's drive angle +/- ang_range
| dist_tolerance     | m     | Used to calculate the minimum safety distance, robot will stop if the projected position at the next timestep is < dist_tolerance away from the wall |
| danger_threshold   | %, 0-1 | If the % of range data that is below the minimum safety distance (closer than the allowed distance) is > danger_threshold, we will STOP |
