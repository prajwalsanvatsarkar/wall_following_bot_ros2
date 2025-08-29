# Wall Following Bot (ROS 2)

This project is a simple **wall-following robot with obstacle avoidance** built in ROS 2.  
It runs in **Gazebo** with **TurtleBot3**, subscribes to `/scan`, and publishes velocity commands to `/cmd_vel`.

---

## Features
- Follows walls in the TurtleBot3 Stage1 world
- Detects obstacles (manually dropped cubes in Gazebo)
- Avoids collisions and resumes wall-following
- Clean Python implementation (`follower_node.py`)

---

## Setup

```bash
# create a workspace (if you don’t already have one)
mkdir -p ~/wall_follower_new/src
cd ~/wall_follower_new/src

# clone this repo
git clone https://github.com/prajwalsanvatsarkar/wall_following_bot_ros2.git

# build
cd ..
colcon build --symlink-install

# source it
source install/setup.bash```


## This launch starts Gazebo with TurtleBot3 Stage1 and then launches the wall-following node after a short delay (to give you time to drop cubes).
```bash
# in a terminal
cd ~/wall_follower_new
source install/setup.bash

# set turtlebot3 model
export TURTLEBOT3_MODEL=burger

# avoid DDS multicast warnings
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7

# launch it
ros2 launch wall_following_bot follower_node.launch.py```

## Topics
# subscribes: /scan (sensor_msgs/LaserScan)

# publishes: /cmd_vel (geometry_msgs/Twist)

## Repo layout
```bash
wall_following_bot_ros2/
├─ wall_following_bot/
│  ├─ launch/
│  │   └─ follower_node.launch.py
│  ├─ wall_following_bot/
│  │   └─ follower_node.py
│  ├─ package.xml
│  ├─ setup.py
│  ├─ setup.cfg
│  └─ resource/
│      └─ wall_following_bot
└─ README.md```

## License
```bash
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.
```
## Author
```bash
Built by Prajwal Sanvatsarkar
```
