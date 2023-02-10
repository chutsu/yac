# Header
std_msgs/Header     header

# CalibTarget
string              target_type
uint32              tag_rows
uint32              tag_cols
float64             tag_size
float64             tag_spacing

# Data
uint32              camera_index
yac_ros/KeyPoint[]  keypoints
