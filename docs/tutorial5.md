# Tutorial 5: Writing a BT node that subscribes/publishes to a ROS topic

## RosSubscriberNode

You only need to overwrite the `bool onMessageReceived()` function. Please refer to [check_bool.h](../examples/check_bool.h) for an example.

## RosPublisherNode

You only need to overwrite the `bool setMessage()`. Please refer to [send_bool.h](../examples/send_bool.h) for an example.