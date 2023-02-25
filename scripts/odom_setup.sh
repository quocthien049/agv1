#SERIAL
roscore
#Check xem USB 
ls -l /dev | grep ttyUSB
#Cap quyen USB
sudo chmod 666 /dev/ttyUSB*
# Chay node
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200




