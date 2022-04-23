#!/usr/bin/env python3

#
# This node is the interface between your flight computer in your GNC algorithms 
# In simulation mode (SIL), it interfaces with the simulator instead of the flight computer
# 
# Inputs: 
#   - Controlled 3D force and torque from template_control:	/control_pub
#   - Finite state machine from the template_fsm :	        /gnc_fsm_pub
#   - Estimated state from template_navigation:		        /kalman_rocket_state
#   - [SIL mode]: Simulated sensor data                     /simu_sensor_pub
#
# Important parameters: (none)
#
# Outputs:
#   - Sensor data (IMU and barometer):                          /sensor_pub
#   - Measured 3D force and torque from the rocket engine :     /control_measured
#

from numpy.lib.type_check import real
import rospy
import rospkg

import numpy as np
import math
import serial
from scipy.interpolate import interp1d 
import time

from rocket_utils.msg import Control
from rocket_utils.msg import FSM
from rocket_utils.msg import State
from rocket_utils.msg import Sensor


rocket_state = State()

# Send back sensors and control as official flight data for GNC
def simu_sensor_callback(sensor):
    sensor_pub.publish(sensor)

# ------- Receive and store GNC data -------
def control_callback(control):
    global rocket_control
    rocket_control = control

def rocket_stateCallback(new_state):
    global rocket_state
    rocket_state = new_state

def fsm_callback(new_fsm):
    global rocket_fsm
    rocket_fsm = new_fsm
    


if __name__ == '__main__':

    GNC_mode = ["Flight", "HIL", "PIL", "SIL"][rospy.get_param("/simulation")]

    THROTTLING = rospy.get_param("/rocket/throttling")

    # Create global variable
    rocket_control = Control()

    rocket_fsm = FSM()
    rocket_fsm.state_machine = "Idle"

    
    # Init ROS
    rospy.init_node('av_interface', anonymous=True)
    
    # Subscribed topics: control, navigation state, fsm  
    rospy.Subscriber("control_pub", Control, control_callback)
    rospy.Subscriber("kalman_rocket_state", State, rocket_stateCallback)
    rospy.Subscriber("gnc_fsm_pub", FSM, fsm_callback)

    # Published topics: sensor data, actuator feedback
    sensor_pub = rospy.Publisher('sensor_pub', Sensor, queue_size=10)
    actuator_pub = rospy.Publisher('control_measured', Control, queue_size=10)

    # If in simulation mode, we enter the ROS loop to remap sensor data from simulation to real sensor data       
    if GNC_mode == "SIL":
        rocket_control_measured = Control()
        rospy.Subscriber("simu_sensor_pub", Sensor, simu_sensor_callback)

        # Load motor thrust curve to get real thrust (for control_measured)
        rospack = rospkg.RosPack()
        thrust_curve = np.loadtxt(rospack.get_path("real_time_simulator") + "/config/thrust_curve/motor_file.txt")
        f_thrust = interp1d(thrust_curve[:,0], thrust_curve[:,1])

        # Init motor force to the one after one integration period
        rocket_control.force.z = rospy.get_param("/rocket/maxThrust")[2]

        rate = rospy.Rate(1.0/(2*rospy.get_param("/rocket/output_delay")))

        while not rospy.is_shutdown():
        
            # Thread sleep time defined by rate
            rate.sleep()

            if rocket_fsm.state_machine != "Idle":

                rocket_control_measured = rocket_control
                real_thrust = 0.0

                if THROTTLING:
                    real_thrust = rocket_control.force.z
                
                # Without throttling activated, the thrust curve is used instead
                else:
                    if rocket_control.force.z != 0.0 and rocket_fsm.time_now >= thrust_curve[0,0] and rocket_fsm.time_now <= thrust_curve[-1,0]:
                        real_thrust = float(f_thrust(rocket_fsm.time_now))

                rocket_control_measured.force.z = real_thrust
                actuator_pub.publish(rocket_control_measured)