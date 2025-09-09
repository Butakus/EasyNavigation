#!/usr/bin/env python3
import atexit
import rclpy
from rclpy.executors import ExternalShutdownException

from rich.text import Text

from textual.app import App, ComposeResult
from textual.widgets import Static, Footer, Tabs, Tab, Label
from textual.containers import Container, Horizontal, Vertical

from ..controller.ros_controllers import (
    TwistSubscriber,
    TwistStampedSubscriber,
    EasyNavControlSubscriber,
)


class EasyNavTabbedApp(App):
    """TUI with Tabs: 'Status' (Navigation Status + NavState/Time stats) and 'Commanding'."""

    CSS = """
    Screen {
        layout: vertical;
    }

    #tabs {
        dock: top;
    }

    #pages {
        height: 1fr;     /* fill remaining space */
        width: 100%;
    }

    /* --------- Status page: two columns --------- */
    #status_root {
        layout: horizontal;
        width: 100%;
        height: 100%;
    }

    /* Left 35%: Navigation Status (3 sub-boxes) */
    #left_col {
        width: 35%;
        height: 100%;
        layout: vertical;
    }

    /* Right 65%: NavState (top), Time stats (bottom) */
    #right_col {
        width: 65%;
        height: 100%;
        layout: vertical;
        margin-left: 1;   /* spacing from the left column */
    }

    /* Titled block: label + bordered box */
    .titled {
        layout: vertical;
        width: 100%;
        height: auto;
        margin-bottom: 1;
    }

    .title {
        padding: 0 1;
        width: 100%;
        height: auto;
        margin-bottom: 0;
    }

    .box {
        border: round;
        padding: 0 1;
        content-align: left top;
        width: 100%;
        height: auto;
        overflow: auto;  /* scroll when content grows */
    }

    /* Right column halves */
    #navstate_block { height: 1fr; }
    #timestats_block { height: 1fr; margin-top: 1; }

    /* NavState / TimeStats boxes fill their halves */
    #navstate_box { height: 100%; }
    #timestats_box { height: 100% }

    /* Navigation Status container */
    #navstatus_block { height: 100%; }
    #navstatus_box   { height: 100%; }

    /* Sub-boxes inside Navigation Status */
    .navstatus_item {}

    /* Commanding page */
    #page_commanding {
        border: round;
        padding: 1 2;
        content-align: left top;
        width: 100%;
        height: 100%;
        overflow: auto;
    }
    """

    BINDINGS = [
        ("q", "quit", "Salir"),
        ("ctrl+c", "quit", "Salir"),
        ("1", "show_status", "Tab Status"),
        ("2", "show_commanding", "Tab Commanding"),
    ]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # ROS 2 init
        rclpy.init(args=None)
        atexit.register(self._ros_shutdown)
        self.node = rclpy.create_node("easynav_tui_status_commanding")

        # ROS 2 subscribers
        self.subs = {}
        self.subs["twist_subscriber"] = TwistSubscriber(self.node, self.twist_callback)
        self.subs["twist_stamped_subscriber"] = TwistStampedSubscriber(
            self.node, self.twist_stamped_callback
        )
        self.subs["easynav_control"] = EasyNavControlSubscriber(self.node, self.control_callback)

        # Widget refs
        self.st_navstate: Static | None = None
        self.st_timestats: Static | None = None
        self.page_commanding: Static | None = None

        # Sub-boxes inside "Navigation Status"
        self.box_nav_control: Static | None = None   # (Status 1) Navigation Control
        self.box_twist: Static | None = None         # (Status 2) Twist
        self.box_current_goal: Static | None = None  # (Status 3) Current Goal

        # Cached last twist texts
        self._last_twist_text = "—"
        self._last_twiststamped_text = "—"

    def compose(self) -> ComposeResult:
        yield Tabs(
            Tab("Status", id="tab_status"),
            Tab("Commanding", id="tab_commanding"),
            id="tabs",
        )

        with Container(id="pages"):
            # Status page
            with Container(id="page_status"):
                with Horizontal(id="status_root"):
                    # LEFT (35%): Navigation Status (3 sub-boxes)
                    with Vertical(id="left_col"):
                        with Vertical(id="navstatus_block", classes="titled"):
                            yield Label("Navigation Status", classes="title")
                            with Vertical(id="navstatus_box", classes="box"):
                                # Sub-box 1: Navigation Control
                                with Vertical(classes="titled"):
                                    yield Label("Navigation Control", classes="title")
                                    self.box_nav_control = Static(
                                        "Esperando NavigationControl…", classes="box navstatus_item"
                                    )
                                    yield self.box_nav_control
                                # Sub-box 2: Twist
                                with Vertical(classes="titled"):
                                    yield Label("Twist", classes="title")
                                    self.box_twist = Static(
                                        "Esperando Twist/TwistStamped…", classes="box navstatus_item"
                                    )
                                    yield self.box_twist
                                # Sub-box 3: Current Goal
                                with Vertical(classes="titled"):
                                    yield Label("Current Goal", classes="title")
                                    self.box_current_goal = Static(
                                        "Current Goal: (ejemplo) x=1.23, y=4.56, theta=0.78 rad",
                                        classes="box navstatus_item",
                                    )
                                    yield self.box_current_goal

                    # RIGHT (65%): NavState (top) + Time stats (bottom)
                    with Vertical(id="right_col"):
                        with Vertical(id="navstate_block", classes="titled"):
                            yield Label("NavState", classes="title")
                            self.st_navstate = Static(
                                "NavState: esperando…", id="navstate_box", classes="box"
                            )
                            yield self.st_navstate

                        with Vertical(id="timestats_block", classes="titled"):
                            yield Label("Time stats", classes="title")
                            self.st_timestats = Static(
                                self._render_time_stats_table([
                                    ("update_rt",                   (210.5, 15.2), (12.3, 2.1), (81.3, 4.7)),
                                    ("correct_localizer_particles", (845.7,110.8), (37.9, 6.3), (26.4, 3.1)),
                                ]),
                                id="timestats_box",
                                classes="box",
                            )
                            yield self.st_timestats

            # Commanding page
            self.page_commanding = Static(
                "Esperando mensajes TwistStamped…\n\nSalir: 'q' o 'Ctrl+C'",
                id="page_commanding",
            )
            yield self.page_commanding

        yield Footer()

    def on_mount(self) -> None:
        # Show initial page
        self._show_page("status")
        # Non-blocking ROS polling (~20 Hz)
        self.set_interval(0.05, self._ros_spin_once)

        # Initial demo content
        if self.st_navstate:
            self.set_navstate_text("\n".join(
                f"state_line_{i}: value_{i}" for i in range(1, 25)
            ))

    # ---------- Tabs <-> Pages ----------
    def on_tabs_tab_activated(self, event: Tabs.TabActivated) -> None:
        if event.tab.id == "tab_status":
            self._show_page("status")
        elif event.tab.id == "tab_commanding":
            self._show_page("commanding")

    def action_show_status(self) -> None:
        self.query_one(Tabs).active = "tab_status"
        self._show_page("status")

    def action_show_commanding(self) -> None:
        self.query_one(Tabs).active = "tab_commanding"
        self._show_page("commanding")

    def _show_page(self, which: str) -> None:
        page_status = self.query_one("#page_status", Container)
        page_command = self.query_one("#page_commanding", Static)
        if which == "status":
            page_status.display = True
            page_command.display = False
        else:
            page_status.display = False
            page_command.display = True

    # ---------- ROS polling ----------
    def _ros_spin_once(self) -> None:
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.0)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    # ---------- Formatting helpers ----------
    @staticmethod
    def _fmt_duration(dur) -> str:
        """Format a builtin_interfaces/Duration-like object as seconds."""
        try:
            sec = getattr(dur, "sec", getattr(dur, "seconds", 0))
            nsec = getattr(dur, "nanosec", 0)
            return f"{sec + nsec/1e9:.3f}s"
        except Exception:
            return str(dur)

    @staticmethod
    def _fmt_pose(pose) -> str:
        """Format Pose or PoseStamped (uses .pose when available)."""
        try:
            p = pose.pose if hasattr(pose, "pose") else pose
            px = getattr(getattr(p, "position", None), "x", None)
            py = getattr(getattr(p, "position", None), "y", None)
            pz = getattr(getattr(p, "position", None), "z", None)
            ox = getattr(getattr(p, "orientation", None), "x", None)
            oy = getattr(getattr(p, "orientation", None), "y", None)
            oz = getattr(getattr(p, "orientation", None), "z", None)
            ow = getattr(getattr(p, "orientation", None), "w", None)
            if px is None:
                return str(pose)
            return (
                f"pos=({px:.3f}, {py:.3f}, {pz:.3f}), "
                f"quat=({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})"
            )
        except Exception:
            return str(pose)

    @staticmethod
    def _color_for_type(t: str) -> str:
        """Return color name for NavigationControl.type."""
        if not t:
            return "white"
        t_upper = t.upper()
        if t_upper in {"ERROR", "REJECTED"}:
            return "red"
        if t_upper in {"CANCELLED", "CANCEL"}:
            return "yellow"
        if t_upper in {"REQUEST", "ACCEPT", "FEEDBACK", "FINISHED"}:
            return "green"
        return "white"

    @staticmethod
    def _render_time_stats_table(rows) -> Text:
        """
        Build a monospace ASCII table from:
        rows: list[(name, (exec_mean, exec_std), (elapsed_mean, elapsed_std), (freq_mean, freq_std))]
        Columns: function name | execution time (μs) | elapsed time (ms) | frequency (Hz)
        """
        headers = [
            "function name",
            "execution time (μs)",
            "elapsed time (ms)",
            "frequency (Hz)",
        ]
        data = []
        for name, exec_t, elapsed, freq in rows:
            e_mean, e_std = exec_t
            l_mean, l_std = elapsed
            f_mean, f_std = freq
            data.append([
                f"{name}",
                f"{e_mean:.2f} ± {e_std:.2f}",
                f"{l_mean:.2f} ± {l_std:.2f}",
                f"{f_mean:.2f} ± {f_std:.2f}",
            ])
        cols = list(zip(*([headers] + data)))
        widths = [max(len(str(x)) for x in col) for col in cols]
        def fmt_row(cells):
            return " | ".join(str(c).ljust(w) for c, w in zip(cells, widths))
        sep = "-+-".join("-" * w for w in widths)
        lines = [fmt_row(headers), sep]
        for row in data:
            lines.append(fmt_row(row))
        table_str = "\n".join(lines)
        return Text(table_str, no_wrap=True)

    def set_navstate_text(self, text: str) -> None:
        """Replace NavState content."""
        if self.st_navstate is not None:
            self.st_navstate.update(text)

    # ---------- ROS callbacks ----------
    def control_callback(self, msg) -> None:
        """
        NavigationControl fields rendered here:
          - type (string) -> colored
          - status_message (string)
          - current_pose (Pose/PoseStamped)
          - navigation_time (Duration)
          - estimated_time_remaining (Duration)
          - distance_covered (float)
          - distance_to_goal (float)
        """
        if self.box_nav_control is None:
            return

        type_str = getattr(msg, "type", "")
        color = self._color_for_type(type_str)
        type_line = f"[{color}]{type_str}[/{color}]" if color != "white" else type_str

        status_message = getattr(msg, "status_message", "")

        current_pose = getattr(msg, "current_pose", None)
        pose_txt = self._fmt_pose(current_pose) if current_pose is not None else "—"

        nav_time = getattr(msg, "navigation_time", None)
        nav_time_txt = self._fmt_duration(nav_time) if nav_time is not None else "—"

        eta = getattr(msg, "estimated_time_remaining", None)
        eta_txt = self._fmt_duration(eta) if eta is not None else "—"

        dist_cov = getattr(msg, "distance_covered", None)
        dist_cov_txt = f"{dist_cov:.3f} m" if isinstance(dist_cov, (int, float)) else "—"

        dist_goal = getattr(msg, "distance_to_goal", None)
        dist_goal_txt = f"{dist_goal:.3f} m" if isinstance(dist_goal, (int, float)) else "—"

        text = (
            f"Type: {type_line}\n"
            f"Message: {status_message}\n"
            f"Current pose: {pose_txt}\n"
            f"Navigation time: {nav_time_txt}\n"
            f"ETA: {eta_txt}\n"
            f"Distance covered: {dist_cov_txt}\n"
            f"Distance to goal: {dist_goal_txt}"
        )

        # Static supports Rich markup for colors
        self.box_nav_control.update(text)

    def twist_callback(self, msg) -> None:
        """Update Twist sub-box with latest Twist."""
        self._last_twist_text = (
            "Twist:\n"
            f"  linear : x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}\n"
            f"  angular: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}"
        )
        self._update_twist_box()

    def twist_stamped_callback(self, msg) -> None:
        """Update Commanding page and Twist sub-box with latest TwistStamped."""
        tw = msg.twist
        self._last_twiststamped_text = (
            "TwistStamped:\n"
            f"  linear : x={tw.linear.x:.3f}, y={tw.linear.y:.3f}, z={tw.linear.z:.3f}\n"
            f"  angular: x={tw.angular.x:.3f}, y={tw.angular.y:.3f}, z={tw.angular.z:.3f}"
        )
        if self.page_commanding:
            self.page_commanding.update(
                f"{self._last_twiststamped_text}\n\nSalir: 'q' o 'Ctrl+C'."
            )
        self._update_twist_box()

    def _update_twist_box(self) -> None:
        """Render both Twist and TwistStamped (if available) in the Twist sub-box."""
        if self.box_twist is None:
            return
        parts = []
        if self._last_twist_text != "—":
            parts.append(self._last_twist_text)
        if self._last_twiststamped_text != "—":
            parts.append(self._last_twiststamped_text)
        self.box_twist.update("\n\n".join(parts) if parts else "Esperando Twist/TwistStamped…")

    def _ros_shutdown(self) -> None:
        """Destroy node and shutdown ROS on exit."""
        if rclpy.ok():
            try:
                self.node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()


if __name__ == "__main__":
    EasyNavTabbedApp().run()


def run_app() -> None:
    EasyNavTabbedApp().run()
