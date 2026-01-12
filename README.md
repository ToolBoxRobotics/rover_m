# rover_m

### Completing the Noetic Environment

#### A. Dependencies
```bash
sudo apt-get install ros-noetic-rosserial-arduino ros-noetic-rosserial
sudo apt-get install ros-noetic-freenect-launch  # For Kinect ***
sudo apt-get install ros-noetic-rplidar-ros      # For C1 Lidar
sudo apt-get install ros-noetic-moveit           # For the Arm
```

#### C. Localization & Navigation (SLAM)
```bash
roslaunch rtabmap_ros rtabmap.launch \
   rtabmap_args:="--delete_db_on_start" \
   depth_topic:=/camera/depth_registered/image_raw \
   rgb_topic:=/camera/rgb/image_rect_color \
   camera_info_topic:=/camera/rgb/camera_info
```

Launch: ``Run roslaunch rover_complete.launch``


#### Start the Setup Assistant:
```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```


### How to Test
1. Launch: ``roslaunch rover_nav.launch``
2. Visualize: Open RViz on your PC.
  - Add Map (Topic: ``/map``)
  - Add LaserScan (Topic: ``/scan``)
  - Add RobotModel
3. Drive: Use a teleop key node (``rosrun teleop_twist_keyboard teleop_twist_keyboard.py``) to drive the rover manually first. You should see the map being drawn in RViz (Black = Obstacle, White = Free space, Grey = Unknown).
4. Autonomous: Once you have a partial map, click "2D Nav Goal" in RViz and click somewhere in the white space. The rover should plan a path (Green line) and drive there automatically.





