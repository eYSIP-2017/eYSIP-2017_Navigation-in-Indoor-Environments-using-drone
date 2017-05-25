# cd to the package workspace
echo "enter the workspace name"
read ws_name
cd ~/"$ws_name"/src

git clone https://github.com/simubhangu/pal_vision_segmentation -b hydro-devel
git clone https://github.com/simubhangu/marker_pose_detection
git clone https://github.com/pal-robotics/aruco_ros


cd ~/"ws_name"

catkin_make