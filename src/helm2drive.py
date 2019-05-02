#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Twist
from kingfisher_msgs.msg import Drive
from kingfisher_msgs.msg import Helm

class Node():
    def __init__(self,linear_scaling,angular_scaling):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.pub = None
        self.driveMsg =None 
        
    def callback(self,data):
        rospy.logdebug("RX: Helm "+rospy.get_caller_id())
        rospy.logdebug("\tthrust:")
        rospy.logdebug("\t\tx:%f"%(data.thrust))
        rospy.logdebug("\tyaw_rate:")
        rospy.logdebug("\t\tx:%f"%(data.yaw_rate))
        # scaling factors
        linfac = self.linear_scaling
        angfac = self.angular_scaling
        self.driveMsg.left = linfac*data.thrust + angfac*data.yaw_rate
        self.driveMsg.right = linfac*data.thrust - angfac*data.yaw_rate
        
        rospy.logdebug("TX: Drive ")
        rospy.logdebug("\tleft:%f, right:%f"%(self.driveMsg.left,
                                             self.driveMsg.right))
        print("left: %f, right: %f"%(self.driveMsg.left,self.driveMsg.right))
        self.pub.publish(self.driveMsg)


if __name__ == '__main__':

    rospy.init_node('helm2drive', anonymous=True)

    # ROS Parameters
    in_topic = rospy.get_param('~input_topic','cmd_helm')
    out_topic = rospy.get_param('~output_topic','cmd_drive')
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',0.2)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling',0.15)

    rospy.loginfo("Subscribing to <%s>, Publishing to <%s>"%(in_topic,out_topic))
    rospy.loginfo("Linear scaling=%f, Angular scaling=%f"%(linear_scaling,angular_scaling))

    node=Node(linear_scaling,angular_scaling)

    # Publisher
    node.pub = rospy.Publisher(out_topic,Drive,queue_size=10)
    node.driveMsg = Drive()

    # Subscriber
    rospy.Subscriber(in_topic,Helm,node.callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
