
import math

from geometry_msgs.msg import Twist, TwistStamped
from easynav_interfaces.msg import NavigationControl, GoalManagerInfo


from rich.text import Text

from std_msgs.msg import String

class TwistProcessor():
    def __init__(self, node, callback):
        self.twist_sub = node.create_subscription(
            Twist,
            'cmd_vel',
            callback,
            10)

        self.twist_sub

    @staticmethod
    def msg2text(msg : Twist) -> str:
        return (
            "Twist:\n"
            f"  linear : x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}\n"
            f"  angular: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}"
        )

class TwistStampedProcessor():
    def __init__(self, node, callback):
        self.twist_sub = node.create_subscription(
            TwistStamped,
            'cmd_vel_stamped',
            callback,
            10)

        self.twist_sub
    @staticmethod
    def msg2text(msg : TwistStamped) -> str:
        tw = msg.twist
        return (
            "TwistStamped:\n"
            f"  linear : x={tw.linear.x:.3f}, y={tw.linear.y:.3f}, z={tw.linear.z:.3f}\n"
            f"  angular: x={tw.angular.x:.3f}, y={tw.angular.y:.3f}, z={tw.angular.z:.3f}"
        )

# Mapping of NavigationControl.type (uint8) to (label, color)
NC_TYPE_MAP: dict[int, tuple[str, str]] = {
    0: ("REQUEST",   "green"),
    1: ("REJECT",    "red"),
    2: ("ACCEPT",    "green"),
    3: ("FEEDBACK",  "green"),
    4: ("FINISHED",  "green"),
    5: ("FAILED",    "red"),
    6: ("CANCEL",    "yellow"),
    7: ("CANCELLED", "yellow"),
    8: ("ERROR",     "red"),
}


# ---------- Formatting helpers ----------
    
def _fmt_duration(dur) -> str:
    try:
        sec = dur.sec
        nsec = dur.nanosec
        return f"{sec + nsec/1e9:.3f}s"
    except Exception:
        return str(dur)

def _fmt_pose(pose) -> str:
    try:
        p = pose.pose if hasattr(pose, "pose") else pose
        px, py, pz = p.position.x, p.position.y, p.position.z
        ox, oy, oz, ow = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        return (
            f"pos=({px:.3f}, {py:.3f}, {pz:.3f}), "
            f"quat=({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})"
        )
    except Exception:
        return str(pose)

class EasyNavControlProcessor():
    def __init__(self, node, callback):
        self.control_sub = node.create_subscription(
            NavigationControl,
            'easynav_control',
            callback,
            10)

        self.control_sub

    @staticmethod
    def msg2text(msg : NavigationControl) -> str:
        t_val: int = msg.type
        label, color = NC_TYPE_MAP.get(t_val, (str(t_val), "white"))
        type_line = f"[{color}]{label}[/{color}]"

        pose_txt = _fmt_pose(msg.current_pose) if msg.current_pose else "—"
        nav_time_txt = _fmt_duration(msg.navigation_time) if msg.navigation_time else "—"
        eta_txt = _fmt_duration(msg.estimated_time_remaining) if msg.estimated_time_remaining else "—"
        dist_cov_txt = f"{msg.distance_covered:.3f} m" if isinstance(msg.distance_covered, (int, float)) else "—"
        dist_goal_txt = f"{msg.distance_to_goal:.3f} m" if isinstance(msg.distance_to_goal, (int, float)) else "—"

        text = (
            f"Type: {type_line}\n"
            f"Message: {msg.status_message}\n"
            f"Current pose: {pose_txt}\n"
            f"Navigation time: {nav_time_txt}\n"
            f"ETA: {eta_txt}\n"
            f"Distance covered: {dist_cov_txt}\n"
            f"Distance to goal: {dist_goal_txt}"
        )
        return text

# Mapping for GoalManagerInfo.status (uint8)
GM_STATUS_MAP: dict[int, tuple[str, str]] = {
    0: ("IDLE",   "yellow"),
    1: ("ACTIVE", "green"),
}

def _quat_to_yaw(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoalManagerInfoProcessor():
    def __init__(self, node, callback):
        self.gm_info_sub = node.create_subscription(
            GoalManagerInfo,
            'easynav_manager_info',
            callback,
            10)

        self.gm_info_sub

    @staticmethod
    def msg2text(msg : GoalManagerInfo) -> str:
        
        s_val: int = msg.status
        s_label, s_color = GM_STATUS_MAP.get(s_val, (str(s_val), "white"))
        status_line = f"Status: [{s_color}]{s_label}[/{s_color}]"

        pos_ok = (msg.position_distance <= msg.position_tolerance)
        ang_ok = (msg.angle_distance <= msg.angle_tolerance)

        if pos_ok:
            pos_line = (
                f"[green]Position: distance={msg.position_distance:.3f} m "
                f"/ tol={msg.position_tolerance:.3f} m[/green]"
            )
        else:
            pos_line = (
                f"Position: distance={msg.position_distance:.3f} m "
                f"/ tol={msg.position_tolerance:.3f} m"
            )

        if pos_ok and ang_ok:
            ang_line = (
                f"[green]Angle: distance={msg.angle_distance:.3f} rad "
                f"/ tol={msg.angle_tolerance:.3f} rad[/green]"
            )
        else:
            ang_line = (
                f"Angle: distance={msg.angle_distance:.3f} rad "
                f"/ tol={msg.angle_tolerance:.3f} rad"
            )

        goals_count = len(msg.goals.goals)
        goals_line = f"Goals remaining: {goals_count}"

        if goals_count > 0:
            g0 = msg.goals.goals[0]
            pose0 = g0.pose if hasattr(g0, "pose") else g0
            x = pose0.position.x
            y = pose0.position.y
            z = pose0.position.z
            yaw = _quat_to_yaw(pose0.orientation)
            first_goal_line = f"First goal: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f} rad"
        else:
            first_goal_line = "First goal: —"

        return "\n".join([status_line, pos_line, ang_line, goals_line, first_goal_line])

class NavStateProcessor():
    def __init__(self, node, callback):
        self.node = node
        self.navstate_sub = node.create_subscription(
            String,
            'easynav_navstate',
            callback,
            10)

        self.navstate_sub

    @staticmethod
    def msg2text(msg : String) -> str:
        return msg.data
    
    def destroy(self):
        self.node.destroy_subscription(self.navstate_sub)