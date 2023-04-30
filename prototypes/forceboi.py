from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import time
import numpy as np


rtde_c = RTDEControl("169.254.9.43")
rtde_r = rtde_receive.RTDEReceiveInterface("169.254.9.43")

task_frame = [0, 0, 0, 0, 0, 0]
selection_vector = [0, 0, 1, 0, 0, 0]
wrench_down = [0, 0, -20, 0, 0, 0] #tried up to 500, same as 150
wrench_up = [0, 0, 10, 0, 0, 0]
force_type = 2
limits = [2, 2, 3.5, 1, 1, 1]
joint_q = [-47.8, -78, -66, -124, 90, 53]
#joint_q = [-57.47* 2*3.1415/360.0, -94.04* 2*3.1415/360.0, -139.9* 2*3.1415/360.0, -33.61* 2*3.1415/360.0, 92.0* 2*3.1415/360.0, 0.85* 2*3.1415/360.0]
joint_speed=[0,0,1,0,0,0]
def rad_angle(arr):
	c=[i*2*3.1415/360.0 for i in arr]
	return c
dt=1.0/500
acceleration=0.1

# Move to initial joint position with a regular moveJ
rtde_c.moveJ(rad_angle(joint_q), 1.50)

#rtde_c.forceModeSetDamping(0.7)
target = rtde_r.getActualTCPPose()
target[2]-=0.2
rtde_c.moveL(target,0.1,0.5,True)
#time.sleep(1)
# Execute 500Hz control loop for 4 seconds, each cycle is 2ms

for i in range(4000):
    #t_start = rtde_c.initPeriod()
    #print(t_start)
    #print(rtde_c.getJointTorques())
    print(f"Magnitude of force at loop {i} is {np.linalg.norm(rtde_r.getActualTCPForce())}")
    print(rtde_r.getActualTCPForce())
    #time.sleep(1)
    #print(target[2])
    # First move the robot down for 2 seconds, then up for 2 seconds
    #print(rtde_r.getActualTCPPose())
    #target[2]+=0.01
    #print(target[2])
    #rtde_c.moveL(target,0.25,0.5,True)
    time.sleep(0.1)
    #rtde_c.waitPeriod(1)

rtde_c.stopL(1)

rtde_c.stopScript()


