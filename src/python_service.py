#!/usr/bin/env python3

import rospy
from behaviortree_ros.srv import AddTwoInts, AddTwoIntsResponse
from std_msgs.msg import Bool

class RosCallback:
    def __init__(self):
        self.variable_true = False

    def trigger(self):
        i = 0
        while i < 10:
            print(i)
            i += 1
        return True

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
