#MASTER_IP=192.168.1.202

#Ray's computer when on superuser_5ghz
MASTER_IP=192.168.1.6 
MASTER_NAME=ros-master
export ROS_MASTER_URI=http://${MASTER_IP}:11311
export ROS_IP=${MASTER_IP}
