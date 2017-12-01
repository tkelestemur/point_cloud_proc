## Point Cloud Processing

#### Installation
This package is tested on Ubuntu 14.04 with ROS Indigo.
1. Navigate to your ROS workspace : `cd ~/catkin_ws/src`
2. Clone the package : `git clone https://github.com/tkelestemur/point_cloud_proc.git`
3. Build the package : `cd .. && catkin build`

#### Usage


#### TODO:

- [x] Individual object clustering (with custom msg).
- [x] rename PlaneObect msg to Plane msg.
- [ ] Single plane segments also finds outliers, need to fix parameters.
- [ ] When there is no point cloud published, service calls throws segmentation fault and kills the node. Fix that.
- [ ] Write documentation.
