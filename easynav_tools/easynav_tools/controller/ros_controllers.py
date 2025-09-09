
from geometry_msgs.msg import Twist, TwistStamped
from easynav_interfaces.msg import NavigationControl

class TwistSubscriber():
    def __init__(self, node, callback):
        self.twist_sub = node.create_subscription(
            Twist,
            'cmd_vel',
            callback,
            10)

        self.twist_sub

class TwistStampedSubscriber():
    def __init__(self, node, callback):
        self.twist_sub = node.create_subscription(
            TwistStamped,
            'cmd_vel_stamped',
            callback,
            10)

        self.twist_sub

class EasyNavControlSubscriber():
    def __init__(self, node, callback):
        self.control_sub = node.create_subscription(
            NavigationControl,
            'easynav_control',
            callback,
            10)

        self.control_sub
