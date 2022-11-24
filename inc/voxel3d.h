/**
 @file      voxel3d.h
 @brief     libvoxel3d APIs for 5Voxel 5Z01A device
 @author    Jackie Lee
 @copyright Copyright (c) 2022 5Voxel Co., Ltd.
*/

#ifndef __VOXEL3d_H__
#define __VOXEL3d_H__

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

#define TOF_CAM_VID                   "0483"
#define TOF_CAM_PID                   "a307"

#define TOF_DEPTH_WIDTH               (224)
#define TOF_DEPTH_HEIGHT              (172)
#define TOF_DEPTH_PIXELS              (TOF_DEPTH_WIDTH * TOF_DEPTH_HEIGHT)
#define TOF_IR_WIDTH                  TOF_DEPTH_WIDTH
#define TOF_IR_HEIGHT                 TOF_DEPTH_HEIGHT
#define TOF_IR_PIXELS                 TOF_DEPTH_PIXELS
#define TOF_DEPTH_ONLY_FRAME_SIZE     (TOF_DEPTH_PIXELS * sizeof(unsigned short))
#define TOF_IR_ONLY_FRAME_SIZE        TOF_DEPTH_ONLY_FRAME_SIZE
#define TOF_DEPTH_IR_FRAME_SIZE       (TOF_DEPTH_ONLY_FRAME_SIZE + TOF_IR_ONLY_FRAME_SIZE)

#define MIN_CONF_THRESHOLD            (10)

#define MAX_SUPPORTED_CAMERA_MODULE   (10) 
#define MAX_PRODUCT_NAME_LEN          (128)
#define MAX_PRODUCT_SN_LEN            (128)

/**
 * @brief  Supported range mode  
 * ### Short range:
 *     - FPS: 25fps
 *     - Suggested sensing range with AE-enabled: 0.1m ~ 1.5m  
 * ### Middle Range:
 *     - FPS: 15fps
 *     - Suggested sensing range with AE-enabled: 0.1m ~ 3.0m  
 * ### Long Range:
 *     - FPS: 5fps
 *     - Suggested sensing range with AE-enabled: 0.1m ~ 4.5m
 */
enum _range_mode {
    SHORT_RANGE_MODE = 1,
    MIDDLE_RANGE_MODE,
    LONG_RANGE_MODE,
};

/**
 * @brief  Structure used in voxel3d_read_camera_info() to read out camera info from device
 */
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

/**
 * @brief  Structure used in voxel3d_scan() to fill up with scanned device number
 *         and product serial number for each device
 */
struct VOXEL3D_API_DLL CamDevInfo {
    int num_of_devices = 0;
    char product_sn[MAX_SUPPORTED_CAMERA_MODULE][MAX_PRODUCT_SN_LEN] = {};
};


/**
 * @brief       Perform the scan of 5Voxel 5Z01A devices
 * @param[out]  CamDevInfo: structure to store the scanned result
 * @return      > 0: number of device(s) found
 * @return      others: can't find device
 */
extern "C" VOXEL3D_API_DLL int voxel3d_scan(CamDevInfo *cam_dev_info);


/**
 * @brief       Perform the initialization of sepcific 5Voxel 5Z01A device
 * @warning     This function has to be called before voxel3d_queryframe(),
 *              otherwise the query will fail
 * @param[in]   dev_sn  device S/N, which can be read from the label on 5Z01A device
 *                      or from result of voxel3d_scan(). Input S/N with NULL pointer
 *                      or empty string will initialize the 1st scanned device
 * @return      true    found device
 * @return      < 0     can't find device or data error
 */
extern "C" VOXEL3D_API_DLL int voxel3d_init(char *dev_sn);


/**
 * @brief       Release the resource allocated for 5Voxel 5Z01A device
 * @warning     This function has to be called before program exit
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 */
extern "C" VOXEL3D_API_DLL void voxel3d_release(char* dev_sn);


/**
 * @brief       Grab a depth & ir frame from 5Z01A camera
 * @warning     Call voxel3d_init() to initialize specific device before query
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  depthmap: pointer of user-allocated buffer for Depth frame storage
 *                        Buffer size shall be TOF_DEPTH_ONLY_FRAME_SIZE (in bytes)
 * @param[out]  irmap: pointer of user-allocated buffer for IR frame storage
 *                     Buffer size shall be TOF_IR_ONLY_FRAME_SIZE (in bytes)
 * @return      > 0: current frame count (1 ~ UINT_MAX)
 * @return      < 0: failed to query a frame from device
 */
extern "C" VOXEL3D_API_DLL unsigned int voxel3d_queryframe(char* dev_sn,
                                                           unsigned short *depthmap,
                                                           unsigned short *irmap);


/**
 * @brief       Generate pointcloud data based on input deptpmap and the
 *              calibration parameters from 5Z01A camera
 * @details     The unit of x/y/z is meter
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   depthmap: pointer of Depth frame filled by voxel3d_queryframe()
 * @param[out]  xyz: pointer of user-allocated buffer for pointcloud frame storage
 * @return      > 0: pixels of pointcloud xyz filled in xyz buffer
 * @return      <= 0: failed to generate pointcloud
 */
extern "C" VOXEL3D_API_DLL int voxel3d_generatePointCloud(char* dev_sn,
                                                          unsigned short *depthmap,
                                                          float *xyz);


/**
 * @brief       Read out camera F/W version
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  fw_ver: pointer of user-allocated buffer to store fw version string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with F/W version string
 * @return      < 0: failed to get F/W version from device or error on input parameters
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_fw_version(char* dev_sn,
                                                       char *fw_ver,
                                                       unsigned int max_len);


/**
 * @brief       Read out camera F/W build date
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  fw_build_date: pointer of user-allocated buffer to store fw build date
 *                    string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with F/W build date string
 * @return      < 0: failed to get F/W build date from device or error on input parameters
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_fw_build_date(char* dev_sn,
                                                          char *fw_build_date,
                                                          unsigned int max_len);


/**
 * @brief       Grab camera info from 5Voxel library
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  camera_params: pointer of uesr-allocated buffer to store camera info
 * @return      true: buffer shall be filled with related camera info
 * @return      < 0: failed to get camera info from library/device or error on inputa
 *                   parameter
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_camera_info(char* dev_sn,
                                                        CameraInfo *cam_info);


/**
 * @brief       Get current confidence threshold value from camera
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      >= 0: confidence threshold value read from camera
 * @return      < 0: failed to get confidence threshold
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_conf_threshold(char* dev_sn);


/**
 * @brief       Set run-time confidence threshold to camera
 * @warning
 *     1. Call this function after voxel3d_init() is completed and successfully,
 *        otherwise, it returns false
 *     2. Corresponding pixel depth will be zero if its confidence is lower than threshold
 *     3. Configured threshold will go back to default when camera is power-cycled
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   conf_threshold: 0~4095
 * @return      true: set confidence threshold successfully
 * @return      < 0: failed to set confidence threshold
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_conf_threshold(char* dev_sn,
                                                         unsigned int conf_threshold);


/**
 * @brief       Get current range mode setting from camera
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      1 -> short range
 * @return      2 -> middle range
 * @return      3 -> long range
 * @return      < 0: failed to get range mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_range_mode(char* dev_sn);


/**
 * @brief       Set range mode to camera
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   range_mode: only accept 1 ~ 3, others will return failure.
 *              (1 -> short range, 2 -> middle range, 3 -> long range)
 * @return      true: set range mode successfully
 * @return      < 0: failed to set range mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_range_mode(char* dev_sn,
                                                      unsigned int range_mode);


/**
 * @brief       Get current auto exposure mode setting from camera
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      0: auto exposure is disabled
 * @return      1: auto exposure is eanbled
 * @return      < 0: failed to get auto exposure mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_auto_exposure_mode(char* dev_sn);


/**
 * @brief       Set auto exposure mode to camera
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              therwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   enable: 0 -> disable auto exposure, others -> enable auto exposure
 * @return      true: set auto exposure mode successfully
 * @return      < 0: failed to set auto exposure mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_auto_exposure_mode(char* dev_sn,
                                                              unsigned int enable);


/**
 * @brief       Get camera HFoV
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated camera HFoV
 * @return      <=0: failed to get camera HFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_get_depth_hfov(char* dev_sn);


/**
 * @brief       Get camera VFoV
 * @warning     Call this function after voxel3d_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated camera VFoV
 * @return      <=0: failed to get camera VFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_get_depth_vfov(char* dev_sn);


/**
 * @brief       Read out library version
 * @param[out]  lib_version: pointer of user-allocated buffer to store library version
 *                           string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with library version string
 * @return      false: failed to get library version
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_lib_version(char* lib_version,
                                                        int max_len);


/**
 * @brief       Read out library build date
 * @param[out]  lib_build_date: pointer of user-allocated buffer to store library build date
 *                              string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with library build date string
 * @return      false: failed to get library build date
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_lib_build_date(char* lib_build_date,
                                                           int max_len);


/**
 * @brief       5Z01A device firmware upgrade utility
 * @warning     Device firmware upgrade utility allows both upgrade to new version of firmware
 *              and also downgrade to older version. User can use voxel3d_read_fw_version() to
 *              confirm the firmware version running on device. It can also be used to confirm
 *              if the firmware version on device after upgrade.
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   file_path: point to the location of the new firmware image string
 * @return      true: completed sending specific firmware to device for upgrade successfully
 * @return      < 0: upgrade failure 
 */
extern "C" VOXEL3D_API_DLL int voxel3d_dev_fw_upgrade(char* dev_sn,
                                                      char* file_path,
                                                      unsigned char (*fw_upgrade_cb)(int state, unsigned int percent_complete));


/**
 * @brief       Function to poll the state and p5Z01A device firmware upgrade utility
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  state: reference of the state varaible for API to write current fw download state
 *              state: < 0 (error), 0: (initial), 1 (downloading), 2 (complete)
 * @param[out]  percent_complete: reference of the percentage varaible for API to write current percentage of completion
 *              percent_complete: 0 ~ 100
 * @return      true: In upgrade procedure
 * @return      < 0: Not in upgrade procedure
 *              Note: while output < 0, state & percent_complete can be used to know if the previous upgrade
 *                    had error or completed withtout failure
 */
extern "C" VOXEL3D_API_DLL int voxel3d_dev_fw_upgrade_state_poll (char* dev_sn, int &state,
                                                                  unsigned int &percent_complete);


/**
 * @brief       Configure filter (Flying Pixel Removal) mode
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   mode: (0 -> Disable, 1 -> Light, 2 -> Standard, 3 -> Heavy)
 * @return      true: completed configration
 * @return      < 0: configuration failure
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_filter_fpr_mode(char* dev_sn,
                                                           int mode);


/**
 * @brief       Get current configuration of filter (Flying Pixel Removal) mode
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      mode: 0 -> Disable
 *                    1 -> Light
 *                    2 -> Standard
 *                    3 -> Heavy
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_filter_fpr_mode(char* dev_sn);

#endif /* __VOXEL3d_H__ */

