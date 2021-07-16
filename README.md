voxel3d_node
----------------------
A ROS node to work with 5Voxel 5Z01A 3D camera, and publish following topics
 - /voxel3d/confidence 
 - /voxel3d/depth
 - /voxel3d/points
 - /voxel3d/camera_info

You need to setup your environment
--------------------------------------
Environments : Ubuntu 18.04 and install ROS

Locate the dir to catkin_ws/src/ and 
Run ros core.

Open a terminal 1:

`$ roscore`

Open a terminal 2:
Run image_view to receive two topics: /voxel3d/depth and /voxel3d/confidence

`$ rosrun image_view image_view image:=/voxel3d/depth`
`$ rosrun image_view image_view image:=/voxel3d/confidence`

Open a terminal 3:
Run image publisher.

`$ rosrun voxel3d_node voxel3d_node`

Open a terminal 4:
Run rviz to receive also pointcloud topic: /voxel3d/points

`$ rosrun rviz rviz -f odom`

Click 'Add' -> 'By topic' -> '/voxel3d/points/PointCloud2' -> 'OK'

Open a terminal 5:
Run rostopic to show camera_info

`$ rostopic echo /voxel3d/camera_info`

