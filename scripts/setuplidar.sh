ls -l /dev |grep ttyUSB

sudo chmod 666 /dev/ttyUSB0
source /home/doan/catkin_ws/devel/setup.bash
roslaunch rplidar_ros rplidar.launch  

xdotool key ctrl+shift+n #sang new terminal

