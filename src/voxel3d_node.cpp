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

#define NUM_VERSION(m, n, r)    (m * 10000 + n * 100 + r)
#define MIN_FW_VERSION          NUM_VERSION(2, 0, 5)

static const std::string TOPIC_NAME_CONF = "/voxel3d/confidence";
static const std::string TOPIC_NAME_DEPTH = "/voxel3d/depth";
static const std::string TOPIC_NAME_POINTS = "/voxel3d/points";
static const std::string TOPIC_NAME_CAMERA_INFO = "/voxel3d/camera_info";

static unsigned short depthmap[TOF_DEPTH_PIXELS];
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

int voxel3d_config_params(const ros::NodeHandle &_nh)
{
    int range_mode = 0;
    int conf_threshold = 40;
    int ret = false;
    char buf[64];

    ret = voxel3d_read_prod_sn(buf, sizeof(buf));
    if (ret) {
        ROS_INFO("5Voexl 5Z01A S/N: %s", buf);
    }

    ret = voxel3d_read_fw_version(buf, sizeof(buf));
    if (ret) {
        int maj, min, rel;

        ROS_INFO("5Voexl 5Z01A FW version: %s", buf);
        sscanf(buf, "V%d.%d.%d", &maj, &min, &rel);

        if (NUM_VERSION(maj, min, rel) < MIN_FW_VERSION) {
            ROS_WARN("To work with latest ROS driver, please upgrade 5Z01A Camera F/W");
            ROS_WARN("F/W upgrade support contact: support@5voxel.com");
        }
    }

    if (_nh.getParam("/voxel3d_node/range_mode", range_mode)) {
        if (range_mode >= SHORT_RANGE && range_mode <= LONG_RANGE) {
            voxel3d_set_range_mode((unsigned int)range_mode);
        }
    }

    if (_nh.getParam("/voxel3d_node/conf_threshold", conf_threshold)) {
        if (conf_threshold >= MIN_CONF_THRESHOLD) {
            voxel3d_set_conf_threshold((unsigned int)conf_threshold);
        }
    }
}

int voxel3d_publishImage(void)
{
    unsigned int frame_index = 0;
    int pcl_pixels = 0;
    float *f_data;
    ros::Time ros_time;
    ros::NodeHandle nh;

    voxel3d_config_params(nh);

    /*
     * Depth topic
     */
    ROS_INFO("Topic : %s", TOPIC_NAME_DEPTH.c_str());
    sensor_msgs::Image depth_image;
    depth_image.width = TOF_DEPTH_WIDTH;
    depth_image.height = TOF_DEPTH_HEIGHT;
    depth_image.encoding = "32FC1";
    depth_image.is_bigendian = false;
    depth_image.step = depth_image.width * sizeof(float);
    depth_image.data.resize(depth_image.step * depth_image.height);
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
                       (unsigned short *)&depthmap[0],
                       (unsigned short *)&(ir_image.data[0]));

        ros_time = ros::Time::now();

        /*
         * Publish Depth image
         */
        depth_image.header.frame_id = frame_index;
        depth_image.header.stamp = ros_time;
        f_data = (float *)&depth_image.data[0];
        for (int ix = 0; ix < TOF_DEPTH_PIXELS; ix++, f_data++) {
            *f_data = (float)depthmap[ix] * 0.001f; //mm -> m
        }
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
        pcl_pixels = voxel3d_generatePointCloud(
                      &depthmap[0],
                          (float *)xyz);

        /*
         * Publish PointCloud2
         */
        if (pcl_pixels) {
            int xyz_idx = 0;

            sensor_msgs::PointCloud2 cloud;
            cloud.header.frame_id = "odom";
            cloud.header.stamp = ros_time;
            cloud.width = pcl_pixels;
            cloud.height = 1;
            cloud.is_bigendian = false;
            cloud.is_dense = false;
            cloud.point_step = (uint32_t)(3 * sizeof(float)); //XYZ
            cloud.row_step = (uint32_t)(cloud.point_step * pcl_pixels);
            cloud.fields.resize(3);
            cloud.fields[0].name = "x";
            cloud.fields[0].offset = 0;
            cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            cloud.fields[0].count = 1;
            cloud.fields[1].name = "y";
            cloud.fields[1].offset = cloud.fields[0].offset + (uint32_t)sizeof(float);
            cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            cloud.fields[1].count = 1;
            cloud.fields[2].name = "z";
            cloud.fields[2].offset = cloud.fields[1].offset + (uint32_t)sizeof(float);
            cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            cloud.fields[2].count = 1;
            cloud.data.resize(cloud.point_step * pcl_pixels);

            for (int ix = 0; ix < pcl_pixels; ix++, xyz_idx += 3) {
                float *p_x = (float *)&cloud.data[ix * cloud.point_step];
                *p_x = xyz[xyz_idx];
                *(p_x + 1) = xyz[xyz_idx + 1];
                *(p_x + 2) = xyz[xyz_idx + 2];
            }
            pub_pointcloud.publish(cloud);

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

