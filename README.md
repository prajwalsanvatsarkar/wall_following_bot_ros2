# wall_following_bot (ROS 2)

Wall-following with obstacle avoidance for TurtleBot3 in Gazebo.  
Package: `wall_following_bot` • Node: `follower_node` • Launch: `launch/follower_node.launch.py`

---

## ✅ Features
- Follows walls using `/scan` (LaserScan)
- Basic obstacle avoidance behavior
- Gazebo simulation with TurtleBot3 (Stage1 world)
- ROS 2-native Python node

---

## 🧰 Prerequisites
- ROS 2 (Humble/Foxy/etc.) with `colcon`
- `turtlebot3_gazebo` installed
- Python 3.x

> If you use TurtleBot3, set the model (e.g., `burger`, `waffle`, `waffle_pi`).

---

## 📦 Build

Clone into a ROS 2 workspace (example here uses `~/iki_workspace/iki_workspace`):

```bash
# 1) Create or use an existing workspace
mkdir -p ~/iki_workspace/iki_workspace/src
cd ~/iki_workspace/iki_workspace/src

# 2) Clone this repo
git clone https://github.com/prajwalsanvatsarkar/wall_following_bot_ros2.git

# 3) Build
cd ..
colcon build --symlink-install

# 4) Source
source install/setup.bash
