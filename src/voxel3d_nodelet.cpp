/*******************************************************
 *
 *  Copyright (c) 2022 5Voxel Co., Ltd.
 *
 *******************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "voxel3d.h"
#include "voxel3d_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define NUM_VERSION(m, n, r)    (m * 10000 + n * 100 + r)
#define MIN_FW_VERSION          NUM_VERSION(2, 0, 5)

static const std::string _NAME_CONF = "confidence";
static const std::string _NAME_DEPTH = "depth";
static const std::string _NAME_POINTS = "points";
static const std::string _NAME_CAMERA_INFO = "camera_info";

namespace voxel3d_nodelet_ns
{

Voxel3dNodelet::Voxel3dNodelet()
{
    cam_sn = "";
    cam_name = "";
}

Voxel3dNodelet::~Voxel3dNodelet()
{
  voxel3d_release((char *)cam_sn.c_str());
}

int voxel3d_build_cam_info(std::string sn, sensor_msgs::CameraInfo *p_cam_info)
{
    int ret = false;

    CameraInfo voxel3d_cam_info;

    ret = voxel3d_read_camera_info((char *)sn.c_str(), &voxel3d_cam_info);
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


int Voxel3dNodelet::voxel3d_config_params(const ros::NodeHandle &_nh)
{
    std::string dev_name, dev_sn, dev_pcl_frame_id;
    int range_mode = 0;
    int conf_threshold = 40;
    bool auto_exposure = false;
    int ret = false;
    char buf[64];

    if (_nh.getParam("camera", dev_name)) {
        cam_name.assign(dev_name);
    } else {
        cam_name.assign("camera");
    }

    if (_nh.getParam("serial_number", dev_sn)) {
        cam_sn.assign(dev_sn);
    } else {
        cam_sn.assign("");
    }

    ROS_INFO("%s : serial number - %s", cam_name.c_str(), cam_sn.c_str());

    ret = voxel3d_init((char *)cam_sn.c_str());
    if (ret <= 0) {
        ROS_ERROR("Error! Failed to find 5Voxel device SN: %s", (char *)cam_sn.c_str());
        exit(0);
    }

    memset(buf, 0x0, sizeof(buf));
    ret = voxel3d_read_lib_version(buf, sizeof(buf));
    if (ret) {
        ROS_INFO("%s : 5Voxel Library version - %s",cam_name.c_str(), buf);
    }

    memset(buf, 0x0, sizeof(buf));
    ret = voxel3d_read_fw_version((char *)cam_sn.c_str(), buf, sizeof(buf));
    if (ret) {
        int maj, min, rel;

        ROS_INFO("%s : 5Voxel 5Z01A FW version - %s", cam_name.c_str(), buf);
        sscanf(buf, "V%d.%d.%d", &maj, &min, &rel);

        if (NUM_VERSION(maj, min, rel) < MIN_FW_VERSION) {
            ROS_WARN("To work with latest ROS driver, please upgrade 5Z01A Camera F/W");
            ROS_WARN("F/W upgrade support contact: support@5voxel.com");
        }
    }


    if (_nh.getParam("range_mode", range_mode)) {
        if (range_mode >= SHORT_RANGE_MODE && range_mode <= LONG_RANGE_MODE) {
            ROS_INFO("%s : set range mode to %d", cam_name.c_str(), range_mode);
            voxel3d_set_range_mode((char *)cam_sn.c_str(), (unsigned int)range_mode);
        }
    }

    if (_nh.getParam("conf_threshold", conf_threshold)) {
        if (conf_threshold >= MIN_CONF_THRESHOLD) {
            ROS_INFO("%s : set confidence threshold to %d", cam_name.c_str(), conf_threshold);
            voxel3d_set_conf_threshold((char *)cam_sn.c_str(), (unsigned int)conf_threshold);
        }
    }

    if (_nh.getParam("auto_exposure", auto_exposure)) {
        ROS_INFO("%s : %s auto exposure", cam_name.c_str(), auto_exposure ? "enable" : "disable");
        voxel3d_set_auto_exposure_mode((char *)cam_sn.c_str(), (unsigned int)auto_exposure);
    }

    if (_nh.getParam("frame_id", dev_pcl_frame_id)) {
        cam_pcl_frame_id.assign(dev_pcl_frame_id);
        ROS_INFO("%s : set PointCloud2 frame id to %s", cam_name.c_str(), cam_pcl_frame_id.c_str());
    }
}

void Voxel3dNodelet::onInit()
{
    ros::NodeHandle &nh = getPrivateNodeHandle();
    int range_mode = 0, ret = 0;
    std::string topic_name_depth, topic_name_conf;
    std::string topic_name_points, topic_name_camera_info;

    voxel3d_config_params(nh);

    /*
     * Depth topic
     */
    topic_name_depth = _NAME_DEPTH;
    ROS_INFO("Topic : %s/%s", cam_name.c_str(), topic_name_depth.c_str());
    depth_image.width = TOF_DEPTH_WIDTH;
    depth_image.height = TOF_DEPTH_HEIGHT;
    depth_image.encoding = "32FC1";
    depth_image.is_bigendian = false;
    depth_image.step = depth_image.width * sizeof(float);
    depth_image.data.resize(depth_image.step * depth_image.height);
    pub_depth = nh.advertise<sensor_msgs::Image>(topic_name_depth, 1);

    /*
     * IR topic
     */
    topic_name_conf = _NAME_CONF;
    ROS_INFO("Topic : %s/%s", cam_name.c_str(), topic_name_conf.c_str());
    ir_image.width = TOF_IR_WIDTH;
    ir_image.height = TOF_IR_HEIGHT;
    ir_image.encoding = "16UC1";
    ir_image.is_bigendian = false;
    ir_image.step = TOF_IR_WIDTH * sizeof(unsigned short);
    ir_image.data.resize(TOF_IR_ONLY_FRAME_SIZE);
    pub_conf = nh.advertise<sensor_msgs::Image>(topic_name_conf, 1);

    /*
     * PointCloud topic
     */
    topic_name_points = _NAME_POINTS;
    ROS_INFO("Topic : %s/%s", cam_name.c_str(), topic_name_points.c_str());
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>(topic_name_points, 1);
 
    /*
     * Camera_info topic
     */
    topic_name_camera_info = _NAME_CAMERA_INFO;
    ROS_INFO("Topic : %s/%s", cam_name.c_str(), topic_name_camera_info.c_str());
    pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>(topic_name_camera_info, 1);
    voxel3d_build_cam_info(cam_sn, &camera_info);

    publoop_timer = nh.createTimer(
      ros::Duration(.04), &Voxel3dNodelet::Run, this);
}

void Voxel3dNodelet::Run(const ros::TimerEvent &)
{
    unsigned int frame_index = 0;
    int pcl_pixels = 0;
    float *f_data;

    /*
     * Query frame from 5Voxel 5Z01A
     */
    frame_index = voxel3d_queryframe((char *)cam_sn.c_str(),
                   (unsigned short *)&depthmap[0],
                   (unsigned short *)&(ir_image.data[0]));

    if (frame_index > 0) {
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
         * Publish IR image
         */
        ir_image.header.frame_id = frame_index;
        ir_image.header.stamp = ros_time;
        pub_conf.publish(ir_image);

        /*
         * 5Voxel library to generate PointCloud data
         */
        pcl_pixels = voxel3d_generatePointCloud((char *)cam_sn.c_str(),
                          &depthmap[0],
                          (float *)xyz);

        /*
         * Publish PointCloud2
         */
        if (pcl_pixels) {
            int xyz_idx = 0;

            sensor_msgs::PointCloud2 cloud;
            cloud.header.frame_id = cam_pcl_frame_id;
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

    }
}

} // namespace voxel3d_nodelet_ns

PLUGINLIB_EXPORT_CLASS(voxel3d_nodelet_ns::Voxel3dNodelet, nodelet::Nodelet)
