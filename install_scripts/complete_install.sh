sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full -y
sudo rosdep init
rosdep update

echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~/catkin_ws/src

git clone https://github.com/AutonomyLab/ardrone_autonomy
git clone https://github.com/dougvk/tum_simulator
git clone https://github.com/simubhangu/pal_vision_segmentation -b hydro-devel
git clone https://github.com/simubhangu/marker_pose_detection
git clone https://github.com/pal-robotics/aruco_ros
git clone https://github.com/eYSIP-2017/eYSIP-2017_Navigation-in-Indoor-Environments-using-drone.git

sudo apt install ros-indigo-moveit -y

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
catkin_make
