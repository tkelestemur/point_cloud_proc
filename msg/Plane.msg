byte XAXIS=0
byte YAXIS=1
byte ZAXIS=2
byte NOAXIS=3

Header header

float64[4] coef

geometry_msgs/Point center

geometry_msgs/Vector3 min

geometry_msgs/Vector3 max

geometry_msgs/Point32[] polygon

sensor_msgs/PointCloud2 cloud

std_msgs/Int32 size

byte orientation
