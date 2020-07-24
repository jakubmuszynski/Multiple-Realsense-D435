# Multiple-Realsense-D435

Transform matrixes between main camera link and all other cameras were calculated using [Kalibr](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration) on 1080p RGB video feed. Based on camera setup used, It is neccesary to take into account translation between RGB and depth sensors.

## Commands:

roslaunch realsense2_camera rs_aligned_depth.launch camera:=cam_M serial_no:=938422071315 \
roslaunch realsense2_camera rs_aligned_depth.launch camera:=cam_M serial_no:=838212070656 \
roslaunch realsense2_camera rs_aligned_depth.launch camera:=cam_M serial_no:=819112070749 \
todo: custom launch files based on rs_aligned_depth.launch with <arg name="enable_pointcloud"   default="true"/> \

python src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py cam_M_link cam_R_link -0.04365167508425966 -0.017474652 -0.009166258 -50 0 0

figure out why the distances are shorter
