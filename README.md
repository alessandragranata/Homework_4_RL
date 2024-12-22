# Homework_4_RL

## Launch Gazebo world
To spawn the mobile robot in the gazebo world in the pose: 
- `x = −3 m`
- `y = 3.5 m`
- `Y = −90 deg`
You need to uncomment the following  code lines`#position = [-3.0, 3.5, 0.100]
    #orientation = [0, 0, -90]` and `#"-Y", str(orientation[2])` and also need to comment `position = [0.0, 0.0, 0.100]`, in the launch file gazebo_fra2mo.launch.py and finally run the following command from the terminal:
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
## IMPORTANT
In gazebo, we click on the small play button in the bottom left corner of the GUI in order to start the simulation.
To run the camera:
```bash
ros2 run rqt_image_view rqt_image_view 
```
and select the `/camera` topic.
## Autonomous Navigation Task
After doing what was previously mentioned, in a new terminal you need to send:
```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```
Finally, in another terminal launch:
```bash
ros2 run rl_fra2mo_description follow_waypoints.launch.py
```
## Launch Rviz 
To open rviz with our configuration, after launching `gazebo_fra2mo.launch.py` and `fra2mo_explore.launch.py`, in a new terminal run:
```bash
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```
## Vision-based Navigation 4a-4b
To spawn the mobile robot in the gazebo world at the initial position, you need to comment the following lines of code `position = [-3.0, 3.5, 0.100]
    orientation = [0, 0, -90]` and `"-Y", str(orientation[2])` and to uncomment `position = [0.0, 0.0, 0.100]`, in the launch file gazebo_fra2mo.launch.py and then run the following command from the first terminal:
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
In the second terminal, to detect the ArUco marker and activate the navigation, launch:
```bash
ros2 launch rl_fra2mo_description nav_fra2mo.launch.py
```
In the third terminal,in order to send the robot in the proximity of the ArUco marker, detect it and then return to the initial position, launch:
```bash
ros2 run rl_fra2mo_description look_aruco.py
```
To run the camera:
```bash
ros2 run rqt_image_view rqt_image_view 
```
In the camera we will obtain the detection of the ArUco tag by selecting the `/aruco_single/result` topic.
## TF Trasformation 4c
To spawn the mobile robot in the gazebo world, launch:
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
In the second terminal launch:
```bash
ros2 launch rl_fra2mo_description nav_fra2mo.launch.py
```
To run the camera:
```bash
ros2 run rqt_image_view rqt_image_view 
```
In the camera we will obtain the detection of the ArUco tag by selecting the `/aruco_single/result` topic.
In the another terminal launch:
```bash
ros2 run rl_fra2mo_description look_aruco.py
```
We can see a printout of the position of the ArUco marker relative to the map. 
To see the the Aruco pose published as TF, run:
```bash
ros2 topic echo /tf_static
