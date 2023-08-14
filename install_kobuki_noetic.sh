sudo apt install liborocos-kdl-dev
sudo apt-get install ros-noetic-ecl-*
sudo apt-get install libusb-dev
sudo apt-get install libftdi-dev

mkdir -p ~/SDK_ws/src
cd ~/SDK_ws/src
git clone https://github.com/yujinrobot/kobuki.git
git clone https://github.com/yujinrobot/yujin_ocs.git
git clone https://github.com/yujinrobot/kobuki_msgs.git
git clone https://github.com/yujinrobot/kobuki_core.git

cd yujin_ocs
mkdir save 
mv yocs_cmd_vel_mux save
mv yocs_controllers save
mv yocs_velocity_smoother save
rm -rf yocs*
cd save 
mv * ..
cd .. && rmdir save
cd ~/SDK_ws/
catkin_make

#enable kokubi serial comm
cat /etc/udev/rules.d/57-kobuki.rules
#rosrun kobuki_ftdi create_udev_rules 
#https://snapcraft.io/blog/ros-production-obtaining-confined-access-to-the-turtlebot-45
