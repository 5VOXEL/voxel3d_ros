/*******************************************************
 *
 *  Copyright (c) 2021 5Voxel Co., Ltd.
 *
 *******************************************************/

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

enum _range_mode {
    SHORT_RANGE = 1,
    MIDDLE_RANGE,
    LONG_RANGE,
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
} __attribute__((packed));


/*
 * Function name: voxel3d_init
 * Description:
 *     Perform the scan of 5Voxel 5Z01A device
 * Note:
 *     This function has to be called before voxel3d_queryframe(),
 *     otherwise the query will fail
 * Input: None
 * Output:
 *     true: found device
 *     false: can't find device
 */
int voxel3d_init(void);

/*
 * Function name: voxel3d_release
 * Description:
 *     Release the resource allocated for 5Voxel 5Z01A device
 * Note:
 *     This function has to be called before program exit
 * Input: None
 * Output: None
 */
void voxel3d_release(void);

/*
 * Function name: voxel3d_queryframe
 * Description:
 *     Grab a depth & ir frame from 5Z01A camera
 * Input:
 *     depthmap: pointer of user-allocated buffer for Depth frame storage
 *     irmap: pointer of user-allocated buffer for IR frame storage
 * Output:
 *     > 0: current frame count (1 ~ UINT_MAX)
 *     = 0: failed to query a frame from device
 */
unsigned int voxel3d_queryframe(unsigned short *depthmap,
                                unsigned short *irmap);

/*
 * Function name: voxel3d_qeneratePointCloud
 * Description:
 *     Generate pointcloud data based on input deptpmap and the
 *     calibration parameters from 5Z01A camera
 * Note:
       The unit of x/y/z is meter
 * Input:
 *     depthmap: pointer of Depth frame filled by voxel3d_queryframe()
 *     xyz: pointer of user-allocated buffer for pointcloud frame storage
 * Output:
 *     > 0: pixels of pointcloud xyz filled in xyz buffer
 *     <= 0: failed to generate pointcloud
 */
int voxel3d_generatePointCloud(unsigned short *depthmap, float *xyz);

/*
 * Function name: voxel3d_read_fw_version
 * Description:
 *     Read out camera F/W version
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     fw_ver: pointer of user-allocated buffer to store fw version string
 *     max_len: length of user-allocated buffer
 * Output:
 *     true: buffer shall be filled with F/W version string
 *     false: failed to get F/W version from device or error on input parameters
 */
int voxel3d_read_fw_version(char *fw_ver, int max_len);

/*
 * Function name: voxel3d_read_fw_build_date
 * Description:
 *     Read out camera F/W build date
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     fw_build_date: pointer of user-allocated buffer to store fw build date
 *                    string
 *     max_len: length of user-allocated buffer
 * Output:
 *     true: buffer shall be filled with F/W build date string
 *     false: failed to get F/W version from device or error on input parameters
 */
int voxel3d_read_fw_build_date(char *fw_build_date, int max_len);

/*
 * Function name: voxel3d_read_prod_sn
 * Description:
 *     Read out camera production serial number
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     prod_sn: pointer of user-allocated buffer to store product s/n string
 *     max_len: length of user-allocated buffer
 * Output:
 *     true: buffer shall be filled with product s/n string
 *     false: failed to get product s/n from device or error on input parameters
 */
int voxel3d_read_prod_sn(char *prod_sn, int max_len);

/*
 * Function name: voxel3d_read_camera_info
 * Description:
 *     Grab camera info from 5Voxel library
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     camera_params: pointer of uesr-allocated buffer to store camera info
 * Output:
 *     true: buffer shall be filled with related camera info
 *     false: failed to get camera info from library/device or error on inputa
 *            parameter
 */
int voxel3d_read_camera_info(CameraInfo *cam_info);

/*
 * Function name: voxel3d_get_conf_threshold
 * Description:
 *     Get current confidence threshold value from camera
 * Note:
 *     Call this function after voxel3d_init() is completed and successfully,
 *     otherwise, it returns false
 * Input:
 *     None
 * Output:
 *     >= 0: confidence threshold value read from camera
 *     < 0: failed to get confidence threshold
 */
int voxel3d_get_conf_threshold(void);

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
 *     conf_threshold: 0~4095
 * Output:
 *     true: set confidence threshold successfully
 *     false: failed to set confidence threshold
 */
int voxel3d_set_conf_threshold(unsigned int conf_threshold);

 /*
  * Function name: voxel3d_get_range_mode
  * Description:
  *     Get current range mode setting from camera
  * Note:
  *     Call this function after voxel3d_init() is completed and successfully,
  *     otherwise, it returns false
  * Input:
  *     None
  * Output:
  *     1~3: 1 -> short range
  *          2 -> middle range
  *          3 -> long range
  *     others: failed to get range mode
  */
 int voxel3d_get_range_mode(void);

 /*
  * Function name: voxel3d_set_range_mode
  * Description:
  *     Set range mode to camera
  * Note:
  *     Call this function after voxel3d_init() is completed and successfully,
  *     otherwise, it returns false
  * Input:
  *     range_mode: only accept 1 ~ 3, others will return failure
  *          1 -> short range
  *          2 -> middle range
  *          3 -> long range
  * Output:
  *     true: set range mode successfully
  *     false: failed to set range mode
  */
 int voxel3d_set_range_mode(unsigned int range_mode);

