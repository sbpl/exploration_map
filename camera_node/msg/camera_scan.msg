#camera_scan.msg
#pretty much a carbon copy of sensor_msgs/LaserScan, but with is paired with camera image

Header header     

sensor_msgs/Image image_data # corresponding image for this scan       

float32 yaw_angle_min        # start angle of the scan [rad]
float32 yaw_angle_max        # end angle of the scan [rad]
float32 yaw_angle_increment  # angular distance between measurements [rad]

float32 roll_angle_min
float32 roll_angle_max
float32 roll_angle_increment

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
