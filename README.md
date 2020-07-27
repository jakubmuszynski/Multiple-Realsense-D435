# Multiple-Realsense-D435

Transform matrixes between main camera link and all other cameras were calculated using [Kalibr](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration) on 1080p RGB video feed. Based on camera setup used, It is neccesary to take into account translation between RGB and depth sensors. Distance between those sensors was measured at 44 milimeters using camera model.

## Commands:

roslaunch realsense2_camera rs_aligned_depth.launch camera:=cam_L serial_no:=947522072464 \
roslaunch realsense2_camera rs_aligned_depth.launch camera:=cam_M serial_no:=948122072058 \
roslaunch realsense2_camera rs_aligned_depth.launch camera:=cam_R serial_no:=938422071315 \
\
todo: custom launch files based on rs_aligned_depth.launch with <arg name="enable_pointcloud"   default="true"/> \
\
python src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py cam_M_link cam_R_link -0.07713135 -0.13010438 -0.01722759 -50 2.8649545 2.5538075 \
python src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py cam_M_link cam_R_link -0.07713135 0.13010438 -0.01722759 50 2.8649545 2.5538075 \
\
3x3 rotation matrix (part of 4x4 transform matrix from Kalibr) was converted to 3 Euler angles [deg] using [this](https://www.andre-gaschler.com/rotationconverter/) tool. Translation values were taken directly from 4x4 Kalibr matrix. The variable sequence is: transX, transY, transZ, rotZ, rotY, rotX. Depending on the application, translation between RGB and depth sensors (44mm) should be added appropriately.
