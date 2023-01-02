# ScanBot
A ROS based room scanning robot.  
My master's thesis.  

## First trials
Using gazebo classic to emulate the real world environment and my robot. Added gaussian noise so that LIDAR readings are not perfect.  
![top-down view of the robot](./res/scanbot-gazebo-1.png)  
![perspective view](./res/scanbot-gazebo-2.png)  

```
resource sources:
gazebo 3d models https://data.nvision2.eecs.yorku.ca/3DGEMS/
3dxo ArrowayTextures
```

```
Setup A Catkin Workspace

These instructions require the use of the catkin build system.

If you do not have a catkin workspace setup, try the following commands:

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make

Then add to your .bashrc file a source to the setup scripts:

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
