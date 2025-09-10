
from geometry_msgs.msg import Twist, TwistStamped
from easynav_interfaces.msg import NavigationControl, GoalManagerInfo
from std_msgs.msg import String
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

class GoalManagerInfoSubscriber():
    def __init__(self, node, callback):
        self.gm_info_sub = node.create_subscription(
            GoalManagerInfo,
            'easynav_manager_info',
            callback,
            10)

        self.gm_info_sub

class NavStateSubscriber():
    def __init__(self, node, callback):
        self.node = node
        self.navstate_sub = node.create_subscription(
            String,
            'easynav_navstate',
            callback,
            10)

        self.navstate_sub

    def destroy(self):
        self.node.destroy_subscription(self.navstate_sub)