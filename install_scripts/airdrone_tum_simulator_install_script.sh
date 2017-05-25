# cd to the package name
echo "enter the workspace name"
read ws_name
cd ~/"$ws_name"/src

git clone https://github.com/AutonomyLab/ardrone_autonomy
# for indigo
git clone https://github.com/dougvk/tum_simulator


# for kinetic
#git clone https://github.com/angelsantamaria/tum_simulator

cd ~/"ws_name"

catkin_make