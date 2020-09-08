# gridmap_generate
该栅格地图代码从港科大[Teach Repeat Replan](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan )简单的分离出来

# Build & Run on ROS

build
```
  cd  catkin_ws/src
  git clone https://github.com/JohnnyYeh/gridmap_generate.git
  cd ../
  catkin_make
  source devel/setup.bash
```
run
```
roslaunch gridmap_generate demo.launch
rosrun pcl_ros pcd_to_pointcloud test.pcd 1.0 _frame_id:=map
```
