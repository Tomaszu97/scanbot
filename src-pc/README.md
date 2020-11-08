# PC Application notes  

## ROS module


ScanBot Conrtol Application is a ROS module, which means that to run it you neeed to have working ROS Noetic setup with required packages. 

Tested Linux/Python setup:
```
Xubuntu 20.04
Python 3.8.5
pip 20.0.2
Pyside2 (Qt for Python)
```

Required ROS packages can be downloaded directly from git or apt repository.
The most important one is __slam_gmapping__:
```bash
sudo apt install ros-noetic-slam-gmapping
```

For convenience move all required ROS bash export commands to __~/.bashrc__ or __/etc/bash.bashrc__

```bash
# ROS
source /opt/ros/noetic/setup.bash
source /home/<your_username>/catkin_ws/devel/setup.bash
```


Just clone whole ScanBot repository and drop it on your Desktop or wherever it's convenient.

Don't forget to install required python dependencies from __requirements.txt__

```bash
pip3 install -r <this_repo_directory>/src-pc/scanbot_communicator/requirements.txt
```
---
### IMPORTANT NOTE:
Older Versions of pyqtgraph (ex. 0.11.0) doesn't have clear() implemented
in 3d graphs. This might be an issue - if it fails, just download newer pyqtgraph version from its git repository on https://github.com/pyqtgraph/pyqtgraph  

---
You also have to link __scanbot_communicator__ folder inside __catkin_ws/src/__ like below:  
```bash
ln -s <this_repo_directory>/src-pc/scanbot_communicator /home/<your_username>/catkin_ws/src/
```

Then go to __catkin_ws__ folder and run
```bash
catkin_make
```
If no errors are thrown everything should work. Connect robot in this manner:
```
[PC]---[USB/Serial Adapter]---[HC-05 module] )))((( [ScanBot robot]

--- wire
((( Bluetooth Connection

```

Finally run
```bash
roslaunch scanbot_communicator start_communicator.launch
```

---
### IMPORTANT NOTE:

start_communicator.launch has hardcoded RViz config path.  
If you dont want to change it and it fails, just remove portion
of code in this file, presented below:
```bash
-d /home/blep/Pulpit/ScanBot/src-pc/res/rviz-config.rviz
```
---