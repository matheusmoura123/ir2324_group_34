GROUP 34

Matheus Henrique Ferreira Moura, matheushenrique.ferreiramoura@studenti.unipd.com

Galiya Yegemberdi, galiya.yegemberdi@studenti.unipd.it

Spagnolo Lorenzo, lorenzo.spagnolo@studenti.unipd.it


HOW TO RUN THE PACKAGE:

After the repo is cloned to ./catkin_ws/src, build the package:
> cd ~/catkin_ws/src

> catkin build

After it finishes, open two terminals.
Run this in every terminal:
> start_tiago (only if using VLAB)

> source /opt/ros/noetic/setup.bash && source /tiago_public_ws/devel/setup.bash && source ~/catkin_ws/devel/setup.bash
	
At the cmd console 1, launch the navigate.launch which will start the 
simulation, start the navigation stack and also the navigate.serve (which is our solution)
> roslaunch group34_pkg1 navigate.launch

At the cmd console 2, start the navigate_client node passing the desired Pose_B.

The Pose_B should be defined in the WORLD frame with x and y in meters and yaw angle in degrees.
> rosrun group34_pkg1 navigate_client x y yaw
 
For each new Pose_B, just run the client node again with the new desired pose.

(

If you want to run the code without the launch file:

cmd console 1
> roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library

cmd console 2
> roslaunch tiago_iaslab_simulation navigation.launch

cmd console 3
> rosrun group34_pkg1 navigate_server

cmd console 4
> rosrun group34_pkg1 navigate_client x y yaw

)