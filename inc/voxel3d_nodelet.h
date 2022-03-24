/*******************************************************
 *
 *  Copyright (c) 2022 5Voxel Co., Ltd.
 *
 *******************************************************/


#ifndef __VOXEL3D_NODELET_H__
#define __VOXEL3D_NODELET_H__

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <nodelet/nodelet.h>
#include "voxel3d.h"

namespace voxel3d_nodelet_ns
{
    class Voxel3dNodelet : public nodelet::Nodelet
    {
    public:
        Voxel3dNodelet();
        ~Voxel3dNodelet();

        virtual void onInit();

    private:
        ros::Publisher pub_depth;
        ros::Publisher pub_conf;
        ros::Publisher pub_pointcloud;
        ros::Publisher pub_camera_info;

        ros::Time ros_time;
        ros::Timer publoop_timer;

        sensor_msgs::Image depth_image;
        sensor_msgs::Image ir_image;
        sensor_msgs::CameraInfo camera_info;

        std::string cam_sn;
        std::string cam_name;

        unsigned short depthmap[TOF_DEPTH_PIXELS];
        float xyz[TOF_DEPTH_PIXELS * 3];

        void Run(const ros::TimerEvent &);

    };
} // namespace voxel3d_nodelet_ns

#endif  /* __VOXEL3D_NODELET_H__ */

