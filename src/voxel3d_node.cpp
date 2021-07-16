/*******************************************************
 *
 *  Copyright (c) 2021 5Voxel Co., Ltd.
 *
 *******************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "voxel3d.h"

static const std::string TOPIC_NAME_CONF = "/voxel3d/confidence";
static const std::string TOPIC_NAME_DEPTH = "/voxel3d/depth";
static const std::string TOPIC_NAME_POINTS = "/voxel3d/points";
static const std::string TOPIC_NAME_CAMERA_INFO = "/voxel3d/camera_info";

static float xyz[TOF_DEPTH_PIXELS * 3];

int voxel3d_build_cam_info(sensor_msgs::CameraInfo *p_cam_info)
{
    int ret = false;

    CameraInfo voxel3d_cam_info;

    ret = voxel3d_read_camera_info(&voxel3d_cam_info);
    if (ret) {
        //Camera resolution
        p_cam_info->width = TOF_DEPTH_WIDTH;
        p_cam_info->height = TOF_DEPTH_HEIGHT;

        //Distortion model used
        p_cam_info->distortion_model = "plumb_bob";
    
        //Radial and Tangential Distortion
        p_cam_info->D.resize(5);
        p_cam_info->D[0] = voxel3d_cam_info.K1;
        p_cam_info->D[1] = voxel3d_cam_info.K2;
        p_cam_info->D[2] = voxel3d_cam_info.P1;
        p_cam_info->D[3] = voxel3d_cam_info.P2;
        p_cam_info->D[4] = voxel3d_cam_info.K3;

        //Camera Matrix
        p_cam_info->K[0] = voxel3d_cam_info.focalLengthFx;
        p_cam_info->K[1] = 0;
        p_cam_info->K[2] = voxel3d_cam_info.principalPointCx;
        p_cam_info->K[3] = 0;
        p_cam_info->K[4] = voxel3d_cam_info.focalLengthFy;
        p_cam_info->K[5] = voxel3d_cam_info.principalPointCy;
        p_cam_info->K[6] = 0;
        p_cam_info->K[7] = 0;
        p_cam_info->K[8] = 1;

        //Rectification Matrix
        p_cam_info->R[0] = 1;
        p_cam_info->R[1] = 0;
        p_cam_info->R[2] = 0;
        p_cam_info->R[3] = 0;
        p_cam_info->R[4] = 1;
        p_cam_info->R[5] = 0;
        p_cam_info->R[6] = 0;
        p_cam_info->R[7] = 0;
        p_cam_info->R[8] = 1;

        //Projection Matrix
        p_cam_info->P[0] = voxel3d_cam_info.focalLengthFx;
        p_cam_info->P[1] = 0;
        p_cam_info->P[2] = voxel3d_cam_info.principalPointCx;
        p_cam_info->P[3] = 0;
        p_cam_info->P[4] = 0;
        p_cam_info->P[5] = voxel3d_cam_info.focalLengthFy;
        p_cam_info->P[6] = voxel3d_cam_info.principalPointCy;
        p_cam_info->P[7] = 0;
        p_cam_info->P[8] = 0;
        p_cam_info->P[9] = 0;
        p_cam_info->P[10] = 1;
        p_cam_info->P[11] = 0;

        //Binning
        p_cam_info->binning_x = 1;
        p_cam_info->binning_y = 1;

        //ROI
        p_cam_info->roi.width = TOF_DEPTH_WIDTH;
        p_cam_info->roi.height = TOF_DEPTH_HEIGHT;
    }

    return (ret);
}

int voxel3d_publishImage(void)
{
    unsigned int frame_index = 0;
    ros::Time ros_time;
    ros::NodeHandle nh;

    /*
     * Depth topic
     */
    ROS_INFO("Topic : %s", TOPIC_NAME_DEPTH.c_str());
    sensor_msgs::Image depth_image;
    depth_image.width = TOF_DEPTH_WIDTH;
    depth_image.height = TOF_DEPTH_HEIGHT;
    depth_image.encoding = "16UC1";
    depth_image.is_bigendian = false;
    depth_image.step = TOF_DEPTH_WIDTH * sizeof(unsigned short);
    depth_image.data.resize(TOF_DEPTH_ONLY_FRAME_SIZE);
    ros::Publisher pub_depth = nh.advertise<sensor_msgs::Image>(TOPIC_NAME_DEPTH, 1);

    /*
     * IR topic
     */
    ROS_INFO("Topic : %s", TOPIC_NAME_CONF.c_str());
    sensor_msgs::Image ir_image;
    ir_image.width = TOF_IR_WIDTH;
    ir_image.height = TOF_IR_HEIGHT;
    ir_image.encoding = "16UC1";
    ir_image.is_bigendian = false;
    ir_image.step = TOF_IR_WIDTH * sizeof(unsigned short);
    ir_image.data.resize(TOF_IR_ONLY_FRAME_SIZE);
    ros::Publisher pub_conf = nh.advertise<sensor_msgs::Image>(TOPIC_NAME_CONF, 1);

    /*
     * PointCloud topic
     */
    ROS_INFO("Topic : %s", TOPIC_NAME_POINTS.c_str());
    ros::Publisher pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>(TOPIC_NAME_POINTS, 1);
 
    /*
     * Camera_info topic
     */
    ROS_INFO("Topic : %s", TOPIC_NAME_CAMERA_INFO.c_str());
    ros::Publisher pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>(TOPIC_NAME_CAMERA_INFO, 1);
    sensor_msgs::CameraInfo camera_info;
    voxel3d_build_cam_info(&camera_info);

    ros::Rate loop_rate(15);

    while (nh.ok())
    {
        /*
         * Query frame from 5Voxel 5Z01A
         */
        frame_index = voxel3d_queryframe(
                       (unsigned short *)&(depth_image.data[0]),
                       (unsigned short *)&(ir_image.data[0]));

        ros_time = ros::Time::now();

        /*
         * Publish Depth image
         */
        depth_image.header.frame_id = frame_index;
        depth_image.header.stamp = ros_time;
        pub_depth.publish(depth_image);

        /*
         * Publish Depth image
         */
        ir_image.header.frame_id = frame_index;
        ir_image.header.stamp = ros_time;
        pub_conf.publish(ir_image);

        /*
         * 5Voxel library to generate PointCloud data
         */
        int pcl_pixels = voxel3d_generatePointCloud(
                          (unsigned short *)&(depth_image.data[0]),
                          (float *)xyz);

        /*
         * Publish PointCloud2
         */
        if (pcl_pixels) {
            sensor_msgs::PointCloud2 output;
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud.width = pcl_pixels;
            cloud.height = 1;
            cloud.header.frame_id = "odom";
            //cloud.header.stamp = ros_time;
            cloud.points.resize(cloud.width * cloud.height);
            pcl_conversions::toPCL(ros_time, cloud.header.stamp);

            int xyz_idx = 0;
            for (int ix = 0; ix < cloud.points.size();
                 ix++, xyz_idx += 3) {
                cloud.points[ix].x = xyz[xyz_idx];
                cloud.points[ix].y = xyz[xyz_idx + 1];
                cloud.points[ix].z = xyz[xyz_idx + 2];
            }

            pcl::toROSMsg(cloud, output);
            pub_pointcloud.publish(output);
        }

        /*
         * Publish Camera Info
         */
        camera_info.header.frame_id = "5Z01A";
        camera_info.header.stamp = ros_time;
        pub_camera_info.publish(camera_info);

        ros::spinOnce();
        loop_rate.sleep();
    }

}

int main(int argc, char** argv)
{
    int ret = -1;

    ros::init(argc, argv, "voxel3d_node");
    ret = voxel3d_init();
    if (!ret) {
    return (-1);
    }

    voxel3d_publishImage();

    voxel3d_release();
    return 0;
}

