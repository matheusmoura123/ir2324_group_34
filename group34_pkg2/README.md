GROUP 34

Matheus Henrique Ferreira Moura, matheushenrique.ferreiramoura@studenti.unipd.com

Galiya Yegemberdi, galiya.yegemberdi@studenti.unipd.it

Spagnolo Lorenzo, lorenzo.spagnolo@studenti.unipd.it


HOW TO RUN THE PACKAGE:

After the repo is cloned to ./catkin_ws/src, build the package:
> cd ~/catkin_ws/src
> start_tiago (only if using VLAB)
> catkin build

After it finishes, open two terminals.
Run this in every terminal:

> start_tiago (only if using VLAB)

> source /opt/ros/noetic/setup.bash && source /tiago_public_ws/devel/setup.bash && source ~/catkin_ws/devel/setup.bash
	
At the cmd console 1, launch the pkg2.launch which will start all the simulation and servers

> roslaunch group34_pkg2 pkg2.launch

If the simulation opens correctly, at the cmd console 2, run pick_place_server:

> rosrun group34_pkg2 pick_place_server

At the cmd console 3, run read_tags_server:

> rosrun group34_pkg2 read_tags_server

Wait unitl Tiago tucks his arm and then at the cmd console 4, run node_A:

> rosrun group34_pkg2 node_A
 
