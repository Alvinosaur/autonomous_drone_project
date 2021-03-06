# autonomous_drone_project
URO SRC Research Project with Professor George Kantor

[Full Project Documentation](https://drive.google.com/drive/folders/1nRpnLqMAy6tX2C9C7IgCKpaO1uepiI_S?usp=sharing)

## Running the Drone Simulator
1. Start up roscore in a separate tab by running:
```
roscore
```

2. Start up Px4 Drone Simulator using ROS Gazebo:
```
cd ~/src/Firmware  # assuming you have downloaded the most recent Px4 Firmware. Do not include in workspace!
make px4_sitl_default gazebo
```
A gazebo window should now pop up with the default Iris 3DR+ Drone loaded.

#### NOTE: Please make sure you've built and sourced your workspace! Need to use catkin build, not catkin_make.

To source:
```
source auton_drone/devel/setup.bash
```

3. Run mavros to set up ROS communication with drone simulator, almost like real setup:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```
Information should scroll down and you should see information such as the following:
```
[ INFO] [1548025762.529638959]: MAVROS started. MY ID 1.240, TARGET ID 1.1
[ INFO] [1548025801.985629583]: udp0: Remote address: 127.0.0.1:14580
[ INFO] [1548025801.986042035]: IMU: High resolution IMU detected!
[ INFO] [1548025802.249840146]: FCU: [logger] file: ./log/2019-01-20/23_10_01.ulg
[ INFO] [1548025802.859082897]: IMU: Attitude quaternion IMU detected!
[ INFO] [1548025802.947095577]: CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
[ INFO] [1548025802.948727744]: IMU: High resolution IMU detected!
[ INFO] [1548025802.958893900]: IMU: Attitude quaternion IMU detected!
[ INFO] [1548025803.952132087]: VER: 1.1: Capabilities         0x000000000000e4ef
[ INFO] [1548025803.952201659]: VER: 1.1: Flight software:     01090040 (0000000004AEF5AB)
[ INFO] [1548025803.952241386]: VER: 1.1: Middleware software: 01090040 (0000000004AEF5AB)
[ INFO] [1548025803.952273969]: VER: 1.1: OS software:         040f00ff (0000000000000000)
[ INFO] [1548025803.952310971]: VER: 1.1: Board hardware:      00000001
[ INFO] [1548025803.952340975]: VER: 1.1: VID/PID:             0000:0000
[ INFO] [1548025803.952363484]: VER: 1.1: UID:                 4954414c44494e4f
```
This shows that mavros was able to establish a connection with the drone and receive information such as IMU data and firmware version.

4. Now that the simulator and ros connections are set up, we can try to send commands to the drone!
```
rosrun auton_drone offboard
```
You should see the following output over time:
```
[ INFO] [1548025836.440769542]: Waiting for FCU Connection
[ INFO] [1548025836.490617899]: Waiting for FCU Connection
[ INFO] [1548025836.540612568]: Waiting for FCU Connection
[ INFO] [1548025836.590615629]: Waiting for FCU Connection
[ INFO] [1548025836.640600265]: Waiting for FCU Connection
[ INFO] [1548025836.690592362]: Waiting for FCU Connection
[ INFO] [1548025836.740608801]: Sending initial points before starting offboard
[ INFO] [1548025846.792831818]: Offboard enabled
[ INFO] [1548025851.849003067]: Vehicle armed
```
Now your simulated drone should rise up and hover!

## Visualizing Ultrasonic Sensors
1. Upload [sensor-publishing code](https://github.com/Alvinosaur/autonomous_drone_project/tree/master/ultrasonic_peripherals) to Arduino, [hook up circuit](https://drive.google.com/open?id=18Ha9dL9g0wC-dsZFRsInmZCISCCwBH5L), and connect with USB

2. Start up roscore in a separate tab by running:
```
roscore
```

3. Find which USB port connected to Arduino:
```
ls /dev/tty  # tab-complete to find one of the following listed: ACM0, ACM1, USB0, USB1
```

4. Set up ROS serial communication with Arduino:
```
bash arduino_connect.sh tty<port>  # Port that you found from step 3 (ex: ACM0)
```

5. Run rviz in another tab to start visualization:
```
rosrun rviz rviz
```

6. Run visualization script: 
```
rosrun auton_drone vizsensors
```

7. Change Fixed Frame(listed under Global Options) from "map" to "main" 
8. Still selecting Fixed Frame, add new topic listed as "Marker"

You should now be able to visualize relative distances from each of the four sensors!
