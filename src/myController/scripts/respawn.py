#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

post_yoda_pos = [-3.9734945485900206, -2.2946287212990297, 0.040000584331184835, 1.0836921688152988e-07, 4.770596063590212e-07, -0.004357995372461962, 0.9999905038929592]
 
def spawn_position(position):
    """
    @brief makes robot respawn to a particular location and orientation

    @param position position and orientation vector
    """

    msg = ModelState()
    msg.model_name = 'B1'

    msg.pose.position.x = position[0]
    msg.pose.position.y = position[1]
    msg.pose.position.z = position[2]
    msg.pose.orientation.x = position[3]
    msg.pose.orientation.y = position[4]
    msg.pose.orientation.z = position[5]
    msg.pose.orientation.w = position[6]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( msg )

    except rospy.ServiceException:
        print ("Service call failed")
