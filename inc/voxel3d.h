/*******************************************************
 *
 *  Copyright (c) 2022 5Voxel Co., Ltd.
 *
 *******************************************************/

#ifndef __VOXEL3D_H__
#define __VOXEL3D_H__

#ifdef PLAT_WINDOWS
#pragma once

#ifdef LIBVOXEL3D_EXPORTS
#define VOXEL3D_API_DLL __declspec(dllexport)
#elif LIBVOXEL3D_STATIC
#define VOXEL3D_API_DLL
#else
#define VOXEL3D_API_DLL __declspec(dllimport)
#endif

#else /* PLAT_LINUX */
#define VOXEL3D_API_DLL
#endif /* PLAT_WINDOWS */

#define TOF_CAM_VID               "0483"
#define TOF_CAM_PID               "a307"

#define TOF_DEPTH_WIDTH           (224)
#define TOF_DEPTH_HEIGHT          (172)
#define TOF_DEPTH_PIXELS          (TOF_DEPTH_WIDTH * TOF_DEPTH_HEIGHT)
#define TOF_IR_WIDTH              TOF_DEPTH_WIDTH
#define TOF_IR_HEIGHT             TOF_DEPTH_HEIGHT
#define TOF_IR_PIXELS             TOF_DEPTH_PIXELS
#define TOF_DEPTH_ONLY_FRAME_SIZE (TOF_DEPTH_WIDTH * TOF_DEPTH_HEIGHT * sizeof(unsigned short))
#define TOF_IR_ONLY_FRAME_SIZE    TOF_DEPTH_ONLY_FRAME_SIZE
#define TOF_DEPTH_IR_FRAME_SIZE   (TOF_IR_ONLY_FRAME_SIZE * 2)

#define MIN_CONF_THRESHOLD        (10)

#define MAX_SUPPORTED_CAMERA_MODULE (10)
#define MAX_PRODUCT_NAME_LEN        128
#define MAX_PRODUCT_SN_LEN          128

enum _range_mode {
    SHORT_RANGE_MODE = 1,
    MIDDLE_RANGE_MODE,
    LONG_RANGE_MODE,
};

struct CameraInfo {
    float focalLengthFx;
    float focalLengthFy;
    float principalPointCx;
    float principalPointCy;
    float K1;
    float K2;
    float P1;
    float P2;
    float K3;
    float K4;
    float K5;
    float K6;
};

struct VOXEL3D_API_DLL CamDevInfo {
    int num_of_devices = 0;
    char product_sn[MAX_SUPPORTED_CAMERA_MODULE][MAX_PRODUCT_SN_LEN] = {};
};


/*
 * Function name: voxel3d_scan
 * Description:
 *     Perform the scan of 5Voxel 5Z01A devices
 * Input:
 *     CamDevInfo: structure to store the scanned result
 * Output:
 *     > 0: number of device(s) found
 *     others: can't find device
 */
extern "C" VOXEL3D_API_DLL int voxel3d_scan(CamDevInfo *cam_dev_info);

/*
 * Function name: voxel3d_init
 * Description:
 *     Perform the scan of 5Voxel 5Z01A device
 * Note:
 *     This function has to be called before voxel3d_queryframe(),
 *     otherwise the query will fail
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 * Output:
 *     true: found device
 *     < 0: can't find device or data error
 */
extern "C" VOXEL3D_API_DLL int voxel3d_init(char *dev_sn);

/*
 * Function name: voxel3d_release
 * Description:
 *     Release the resource allocated for 5Voxel 5Z01A device
 * Note:
 *     This function has to be called before program exit
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 * Output: None
 */
extern "C" VOXEL3D_API_DLL void voxel3d_release(char* dev_sn);

/*
 * Function name: voxel3d_queryframe
 * Description:
 *     Grab a depth & ir frame from 5Z01A camera
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     depthmap: pointer of user-allocated buffer for Depth frame storage
 *     irmap: pointer of user-allocated buffer for IR frame storage
 * Output:
 *     >  0: current frame count (1 ~ UINT_MAX)
 *     <= 0: failed to query a frame from device
 */
extern "C" VOXEL3D_API_DLL unsigned int voxel3d_queryframe(char* dev_sn,
                                                           unsigned short *depthmap,
                                                           unsigned short *irmap);

/*
 * Function name: voxel3d_generatePointCloud
 * Description:
 *     Generate pointcloud data based on input deptpmap and the
 *     calibration parameters from 5Z01A camera
 * Note:
 *     The unit of x/y/z is meter
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     depthmap: pointer of Depth frame filled by voxel3d_queryframe()
 *     xyz: pointer of user-allocated buffer for pointcloud frame storage
 * Output:
 *     > 0: pixels of pointcloud xyz filled in xyz buffer
 *     <= 0: failed to generate pointcloud
 */
extern "C" VOXEL3D_API_DLL int voxel3d_generatePointCloud(char* dev_sn,
                                                          unsigned short *depthmap,
                                                          float *xyz);

/*
 * Function name: voxel3d_read_fw_version
 * Description:
 *     Read out camera F/W version
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     fw_ver: pointer of user-allocated buffer to store fw version string
 *     max_len: length of user-allocated buffer
 * Output:
 *     true: buffer shall be filled with F/W version string
 *     < 0: failed to get F/W version from device or error on input parameters
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_fw_version(char* dev_sn,
                                                       char *fw_ver,
                                                       unsigned int max_len);

/*
 * Function name: voxel3d_read_fw_build_date
 * Description:
 *     Read out camera F/W build date
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     fw_build_date: pointer of user-allocated buffer to store fw build date
 *                    string
 *     max_len: length of user-allocated buffer
 * Output:
 *     true: buffer shall be filled with F/W build date string
 *     < 0: failed to get F/W build date from device or error on input parameters
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_fw_build_date(char* dev_sn,
                                                          char *fw_build_date,
                                                          unsigned int max_len);

/*
 * Function name: voxel3d_read_camera_info
 * Description:
 *     Grab camera info from 5Voxel library
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     camera_params: pointer of uesr-allocated buffer to store camera info
 * Output:
 *     true: buffer shall be filled with related camera info
 *     < 0: failed to get camera info from library/device or error on inputa
 *            parameter
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_camera_info(char* dev_sn,
                                                        CameraInfo *cam_info);

/*
 * Function name: voxel3d_get_conf_threshold
 * Description:
 *     Get current confidence threshold value from camera
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 * Output:
 *     >= 0: confidence threshold value read from camera
 *     < 0: failed to get confidence threshold
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_conf_threshold(char* dev_sn);

/*
 * Function name: voxel3d_set_conf_threshold
 * Description:
 *     Set run-time confidence threshold to camera
 * Note:
 *     1. Call this function after voxel3d_init() is completed and successfully,
 *         otherwise, it returns false
 *     2. Phase will be 0 if pixel confidence is lower than threshold
 *     3. The value will go back to default when camera is power-cycled
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     conf_threshold: 0~4095
 * Output:
 *     true: set confidence threshold successfully
 *     < 0: failed to set confidence threshold
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_conf_threshold(char* dev_sn,
                                                         unsigned int conf_threshold);

/*
 * Function name: voxel3d_get_range_mode
 * Description:
 *     Get current range mode setting from camera
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 * Output:
 *     1~3: 1 -> short range
 *          2 -> middle range
 *          3 -> long range
 *     < 0: failed to get range mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_range_mode(char* dev_sn);

/*
 * Function name: voxel3d_set_range_mode
 * Description:
 *     Set range mode to camera
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     range_mode: only accept 1 ~ 3, others will return failure
 *          1 -> short range
 *          2 -> middle range
 *          3 -> long range
 * Output:
 *     true: set range mode successfully
 *     < 0: failed to set range mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_range_mode(char* dev_sn,
                                                      unsigned int range_mode);

/*
 * Function name: voxel3d_get_auto_exposure_mode
 * Description:
 *     Get current auto exposure mode setting from camera
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 * Output:
 *     0: auto exposure is disabled
 *     1: auto exposure is eanbled
 *     < 0: failed to get auto exposure mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_auto_exposure_mode(char* dev_sn);

/*
 * Function name: voxel3d_set_auto_exposure_mode
 * Description:
 *     Set auto exposure mode to camera
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     enable:
 *          0 -> disable auto exposure
 *          others -> enable auto exposure
 * Output:
 *     true: set auto exposure mode successfully
 *     < 0: failed to set auto exposure mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_auto_exposure_mode(char* dev_sn,
                                                              unsigned int enable);

/*
 * Function name: voxel3d_get_depth_hfov
 * Description:
 *     Get camera HFoV
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 * Output:
 *     > 0: calculated camera HFoV
 *     <=0: failed to get camera HFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_get_depth_hfov(char* dev_sn);

/*
 * Function name: voxel3d_get_depth_vfov
 * Description:
 *     Get camera VFoV
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 * Output:
 *     > 0: calculated camera VFoV
 *     <=0: failed to get camera VFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_get_depth_vfov(char* dev_sn);

/*
 * Function name: voxel3d_read_lib_version
 * Description:
 *     Read out library version
 * Input:
 *     lib_version: pointer of user-allocated buffer to store library version
 *                    string
 *     max_len: length of user-allocated buffer
 * Output:
 *     true: buffer shall be filled with library version string
 *     false: failed to get library version
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_lib_version(char* lib_version,
                                                        int max_len);

/*
 * Function name: voxel3d_read_lib_build_date
 * Description:
 *     Read out library build date
 * Input:
 *     lib_build_date: pointer of user-allocated buffer to store library build date
 *                    string
 *     max_len: length of user-allocated buffer
 * Output:
 *     true: buffer shall be filled with library build date string
 *     false: failed to get library build date
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_lib_build_date(char* lib_build_date,
                                                           int max_len);

/*
 * Function name: voxel3d_dev_fw_upgrade
 * Description:
 *     5Z01A device firmware upgrade utility
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     file_path: point to the location of the new firmware image
 *                    string
 * Note:
 *     Device firmware upgrade utility allows both upgrade to new version of firmware
 *     and also downgrade to older version. User can use voxel3d_read_fw_version() to
 *     confirm the firmware version running on device. It can also be used to confirm
 *     if the firmware version on device after upgrade.
 * Output:
 *     true: completed sending specific firmware to device for upgrade successfully
 *     < 0: upgrade failure
 */
extern "C" VOXEL3D_API_DLL int voxel3d_dev_fw_upgrade(char* dev_sn,
                                                      char* file_path);

/*
 * Function name: voxel3d_set_filter_fpr_mode
 * Description:
 *     Configure filter (Flying Pixel Removal) mode
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 *     mode: 0 -> Disable
 *           1 -> Light
 *           2 -> Standard
 *           3 -> Heavy
 * Output:
 *     true: completed configration
 *     < 0: configuration failure
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_filter_fpr_mode(char* dev_sn,
                                                          int mode);

/*
 * Function name: voxel3d_get_filter_fpr_mode
 * Description:
 *     Get current configuration of filter (Flying Pixel Removal) mode
 * Input:
 *     dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *             initialize the 1st scanned device
 * Output:
 *     mode: 0 -> Disable
 *           1 -> Light
 *           2 -> Standard
 *           3 -> Heavy
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_filter_fpr_mode(char* dev_sn);

#endif /* __VOXEL3D_H__ */

