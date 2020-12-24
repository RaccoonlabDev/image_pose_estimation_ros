# image_pose_estimation_ros

The Ros package to track and get position in world of the selected object.
Written on Python Opencv with SIFT + Homography + PnP + blur detector.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/V6yGv8Z46hM/0.jpg)](https://youtu.be/V6yGv8Z46hM "ros image pose estimation")

**Run:**
```bash
roslaunch image_pose_estimation_ros pose_estimation.launch 
```
key *"i"* - select image<br/>
key *"s"* - save image<br/>
key *"q"* - exit<br/>

#### Subscribed Topics:

image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Raw image stream from the camera driver.<br/>
camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)): Camera metadata.<br/>

#### Publisher Topics:

find_object[geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)): Position of the found object relative "map_id"<br/>
tf2 ([geometry_msgs/TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html)): Tf2 pose of find object<br/> 
find_object/image ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Debug image with draw object.<br/>

#### Parameters:

~rate (float, default: 10)<br/>
&emsp;&emsp;*Frame rate of node. If rate <= 0, work without delay<br/>*
~blur_threshold (int, default: 300)<br/>
&emsp;&emsp;*Filter of blur detector. The higher the value, the more sensitive the filter.<br/>*
~min_match_count (int, default: 10)<br/>
&emsp;&emsp;*Value of SIFT detector. The lower the value, the more sensitive the filter.<br/>*
~camera_name (string, default: "camera")<br/>
&emsp;&emsp;*Name of camera.<br/>*
~frame_id (string, default: "object")<br/>
&emsp;&emsp;*Name of find object.<br/>*
~map_id (string, default: "map")<br/>
&emsp;&emsp;*frame id relative to which pose_stamped is published .<br/>*
~size_image (float, default: 0.1)<br/>
&emsp;&emsp;*The width of the image in meters.<br/>*
~show_image (bool, default: true)<br/>
&emsp;&emsp;*Uses a known image.<br/>*
~use_image (bool, default: false)<br/>
&emsp;&emsp;*Show image in window.<br/>*
~image_path (string, default: "camera")<br/>
&emsp;&emsp;*The path to known image.<br/>*
~max_dist (float, default: "5.0" )<br/>
&emsp;&emsp;*Maximum distance (meter) of find object (need for filter).<br/>*


# ros qr code pose estimation

The Ros package to get QR code data and position in world.
Written on Python Opencv + zbar + PnP + blur detector.

**Run:**
```bash
roslaunch image_pose_estimation qr_pose_estimation.lanch 
```

#### Subscribed Topics:

image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Raw image stream from the camera driver.<br/>
camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)): Camera metadata.<br/>

#### Publisher Topics:

find_qr[drone_msgs/QrCode](): Position and data of the found object relative "map_id"<br/>
tf2 ([geometry_msgs/TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html)): Tf2 pose of find object<br/> 

#### Parameters:

~rate (float, default: 10)<br/>
&emsp;&emsp;*Frame rate of node. If rate <= 0, work without delay<br/>*
~blur_threshold (int, default: 300)<br/>
&emsp;&emsp;*Filter of blur detector. The higher the value, the more sensitive the filter.<br/>*
~camera_name (string, default: "camera")<br/>
&emsp;&emsp;*Name of camera.<br/>*
~frame_id (string, default: "object")<br/>
&emsp;&emsp;*Name of find object.<br/>*
~map_id (string, default: "map")<br/>
&emsp;&emsp;*frame id relative to which pose_stamped is published .<br/>*
~size_image (float, default: 0.1)<br/>
&emsp;&emsp;*The width of the image in meters.<br/>*
~show_image (bool, default: true)<br/>
&emsp;&emsp;*Uses a known image.<br/>*
~max_dist (float, default: "5.0" )<br/>
&emsp;&emsp;*Maximum distance (meter) of find object (need for filter).<br/>*

