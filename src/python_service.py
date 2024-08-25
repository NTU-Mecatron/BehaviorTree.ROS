#!/usr/bin/env python

from __future__ import print_function
import rospy
from behaviortree_ros.srv import AddTwoInts, AddTwoIntsResponse
from std_msgs.msg import Bool

class RosCallback:
    def __init__(self):
        self.variable_true = False
        self.srv_trigger_sub = rospy.Subscriber("srv_end_trigger", Bool, self.srv_end_callback)

    def srv_end_callback(self, msg):
        # Set the variable when the message is received
        self.variable_true = msg.data

    def trigger(self):
        return self.variable_true

# embed your class into the ros service server
def handle_add_two_ints(req):
    class_callback = RosCallback()

    # Wait until the variable is set to True by the subscriber callback
    while not class_callback.trigger():
        print("still in the loop")
        rospy.sleep(0.1)  

    # Once variable_true is True, perform the addition
    result = req.a + req.b
    print("Returning [%s + %s = %s]" % (req.a, req.b, result))
    return AddTwoIntsResponse(result)

def main():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    main()
