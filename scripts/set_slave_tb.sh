#Ray's computer when on superuser_5ghz
MASTER_IP=192.168.1.6

#Toughbook when on superuser_5ghz
THIS_IP=192.168.1.8
export ROS_MASTER_URI=http://${MASTER_IP}:11311
export ROS_IP=${THIS_IP}
