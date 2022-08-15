voxel3d_nodelet
----------------------
ROS nodelet to work with 5Voxel 5Z01A 3D camera(s), and publish following topics
 - /voxel3d_nodelet_ns/camera1/confidence 
 - /voxel3d_nodelet_ns/camera1/depth
 - /voxel3d_nodelet_ns/camera1/points
 - /voxel3d_nodelet_ns/camera1/camera_info
 - /voxel3d_nodelet_ns/camera2/confidence 
 - /voxel3d_nodelet_ns/camera2/depth
 - /voxel3d_nodelet_ns/camera2/points
 - /voxel3d_nodelet_ns/camera2/camera_info  
 ....
 
 Note:
   1. camera name, as an argument in ROS nodelet, can be set within launch file or taken from the command line.  
      The name of topics will be updated accordingly. See more parameters in launch file.  
          &nbsp;&nbsp;ex: "roslaunch voxel3d_nodelet multi_cameras.launch cam1_name:=Left_cam cam2:=Right_cam"  
          &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;topics become /voxel3d_nodelet_ns/Left_cam/confidence ...
   2. To run multiple cameras, you need to set the corresponding "serial_number" parameters in multi_cameras.launch.  
      Library will scan devices based on these parameters, and fail if any of them can't be found.
   3. To run with single camera, use single_camea.launch with "" in serial_number parameter. Library will scan  
      and select the 1st found device to start. Its serial number will also be displayed in screen.


You need to setup your environment
--------------------------------------
Environments : Ubuntu 18.04 and install ROS

1. Locate the dir to catkin_ws/src/, catkin_make, and install
2. To run with multiple 5Z01A devices, modify multi_cameras.launch with correct serial number
3. Run ros core.

Open a terminal 1:

`$ roscore`

Open a terminal 2:
Run image_view to receive two topics: /voxel3d_nodelet_ns/camera1/depth and /voxel3d_nodelet_ns/camera1/confidence
Note: topics might be changed based on configured camera name

`$ rosrun image_view image_view image:=/voxel3d_nodelet_ns/camera1/depth`  
`$ rosrun image_view image_view image:=/voxel3d_nodelet_ns/camera1/confidence`  

Open a terminal 3:
Run image publisher.

`$ ros_launch voxel3d_nodelet single_camera.launch`, or  
`$ ros_launch voxel3d_nodelet multi_cameras.launch`  

Open a terminal 4:
Run rviz to receive also pointcloud topic: /voxel3d_nodelet_ns/camera1/points

`$ rosrun rviz rviz -f odom`

Click 'Add' -> 'By topic' -> '/voxel3d_nodelet_ns/camera1/points/PointCloud2' -> 'OK'

Open a terminal 5:
Run rostopic to show camera_info

`$ rostopic echo /voxel3d_nodelet_ns/camera1/camera_info`

