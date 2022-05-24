# Header
std_msgs/Header        header

# Calibration Target
yac_ros/CalibTarget    calib_target

# IMU
float64                imu_rate
yac_ros/ImuParams      imu_params
yac_ros/Extrinsics     imu_exts

# Cameras
float64                cam_rate
yac_ros/CameraParams[] cam_params
yac_ros/Extrinsics[]   cam_exts

# Fiducial pose
float64[7]             fiducial_pose

# Calibration info
float64[36]            calib_info
