#!/usr/bin/env python3
import atexit
import math
import os
import re
import rclpy
from rclpy.executors import ExternalShutdownException

from rich.text import Text

from textual.app import App, ComposeResult
from textual.widgets import Static, Footer, Tabs, Tab, Label, Switch
from textual.containers import Container, Horizontal, Vertical

from ..controller.ros_controllers import (
    TwistSubscriber,
    TwistStampedSubscriber,
    EasyNavControlSubscriber,
    GoalManagerInfoSubscriber,
    NavStateSubscriber
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

# Mapping for GoalManagerInfo.status (uint8)
GM_STATUS_MAP: dict[int, tuple[str, str]] = {
    0: ("IDLE",   "yellow"),
    1: ("ACTIVE", "green"),
}

# -------- Running stats (Welford) --------
class RunningStats:
    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.M2 = 0.0

    def update(self, x: float) -> None:
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.M2 += delta * delta2

    def as_tuple(self) -> tuple[float, float]:
        if self.n < 2:
            return (self.mean, 0.0)
        return (self.mean, (self.M2 / (self.n - 1)) ** 0.5)


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
        height: 1fr;
        width: 100%;
    }

    #status_root {
        layout: horizontal;
        width: 100%;
        height: 100%;
    }

    /* Left 35%: Navigation Status (sub-boxes) */
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
        margin-left: 1;
    }

    .titled {
        layout: vertical;
        width: 100%;
        height: auto;
        margin-bottom: 1;
    }

    .hdr {
        layout: horizontal;
        height: auto;
        width: 100%;
    }

    .spacer {
        width: 1fr;
    }

    .title {
        padding: 0 1;
        height: auto;
    }

    .box {
        border: round;
        padding: 0 1;
        content-align: left top;
        width: 100%;
        height: auto;
        overflow: auto;
    }

    #navstate_block { height: 1fr; }
    #timestats_block { height: 1fr; margin-top: 1; }

    #navstate_wrap { height: 100%; }
    #timestats_wrap { height: 100%; }

    #navstatus_block { height: 100%; }
    #navstatus_box   { height: 100%; }

    .navstatus_item {}

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

    # ---------- Time stats config ----------
    _LOG_PATH = "/tmp/easynav.log"
    _LOG_RE = re.compile(r"^(?P<name>\S+)\s+(?P<start>\d+)\s+(?P<end>\d+)\s*$")

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
        self.subs["goal_info"] = GoalManagerInfoSubscriber(self.node, self.goal_info_callback)

        # Widget refs
        self.st_navstate: Static | None = None
        self.st_timestats: Static | None = None
        self.page_commanding: Static | None = None

        # Sub-boxes inside "Navigation Status"
        self.box_nav_control: Static | None = None
        self.box_goal_info: Static | None = None
        self.box_twist: Static | None = None

        # Switch state and buffers
        self.navstate_enabled = True
        self.timestats_enabled = True
        self._last_navstate_text: Text | str = ""
        self._last_timestats_text: Text | str = ""

        # Cached last twist texts
        self._last_twist_text = "—"
        self._last_twiststamped_text = "—"

        # ---- Time stats state (tailing the log) ----
        self._log_fh = None
        self._log_inode = None
        self._log_pos = 0
        # per-function accumulators
        # func -> { 'exec': RunningStats, 'elapsed': RunningStats, 'freq': RunningStats, 'last_start': int|None }
        self._ts_stats: dict[str, dict] = {}

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
                    # LEFT column: Navigation Status
                    with Vertical(id="left_col"):
                        with Vertical(id="navstatus_block", classes="titled"):
                            yield Label("Navigation Status", classes="title")
                            with Vertical(id="navstatus_box", classes="box"):
                                # 1) Navigation Control
                                with Vertical(classes="titled"):
                                    yield Label("Navigation Control", classes="title")
                                    self.box_nav_control = Static(
                                        "Esperando NavigationControl…", classes="box navstatus_item"
                                    )
                                    yield self.box_nav_control
                                # 2) Goal Info
                                with Vertical(classes="titled"):
                                    yield Label("Goal Info", classes="title")
                                    self.box_goal_info = Static(
                                        "Esperando GoalManagerInfo…", classes="box navstatus_item"
                                    )
                                    yield self.box_goal_info
                                # 3) Twist
                                with Vertical(classes="titled"):
                                    yield Label("Twist", classes="title")
                                    self.box_twist = Static(
                                        "Esperando Twist/TwistStamped…", classes="box navstatus_item"
                                    )
                                    yield self.box_twist

                    # RIGHT column: NavState + Time stats
                    with Vertical(id="right_col"):
                        # NavState (switch inside border)
                        with Vertical(id="navstate_block", classes="titled"):
                            with Vertical(id="navstate_wrap", classes="box"):
                                with Horizontal(classes="hdr"):
                                    yield Label("NavState", classes="title")
                                    yield Static("", classes="spacer")
                                    yield Switch(value=True, id="sw_navstate")
                                self.st_navstate = Static("NavState: esperando…")
                                yield self.st_navstate

                        # Time stats (switch inside border)
                        with Vertical(id="timestats_block", classes="titled"):
                            with Vertical(id="timestats_wrap", classes="box"):
                                with Horizontal(classes="hdr"):
                                    yield Label("Time stats", classes="title")
                                    yield Static("", classes="spacer")
                                    yield Switch(value=True, id="sw_timestats")
                                # start with placeholder; will be replaced by log data
                                initial_table = self._render_time_stats_table([])
                                self._last_timestats_text = initial_table
                                self.st_timestats = Static(initial_table)
                                yield self.st_timestats

            # Commanding page
            self.page_commanding = Static(
                "Esperando mensajes TwistStamped…\n\nSalir: 'q' o 'Ctrl+C'",
                id="page_commanding",
            )
            yield self.page_commanding

        yield Footer()

    def on_mount(self) -> None:
        self._show_page("status")
        # ROS polling
        self.set_interval(0.05, self._ros_spin_once)
        # create NavState sub if switch is ON
        if self.query_one("#sw_navstate", Switch).value:
            self.subs["navstate"] = NavStateSubscriber(self.node, self.navstate_callback)
        # Time stats: poll the log periodically (10 Hz is overkill; use ~2 Hz)
        self.set_interval(0.5, self._poll_time_stats_log)

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

    # ---------- Switch handling ----------
    def on_switch_changed(self, event: Switch.Changed) -> None:
        if event.switch.id == "sw_navstate":
            self.navstate_enabled = event.value
            if event.value:
                # ON: (re)create subscriber and restore last content
                self.subs["navstate"] = NavStateSubscriber(self.node, self.navstate_callback)
                if self.st_navstate:
                    self.st_navstate.update(self._last_navstate_text)
            else:
                # OFF: clear UI and destroy subscriber to free resources
                if self.st_navstate:
                    self.st_navstate.update("")
                # try to destroy wrapper and remove
                sub = self.subs.pop("navstate", None)
                if sub is not None and hasattr(sub, "destroy"):
                    try:
                        sub.destroy()
                    except Exception:
                        pass

        elif event.switch.id == "sw_timestats":
            self.timestats_enabled = event.value
            if not event.value and self.st_timestats:
                self.st_timestats.update("")
            elif event.value and self.st_timestats:
                self.st_timestats.update(self._last_timestats_text)

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
        try:
            sec = dur.sec
            nsec = dur.nanosec
            return f"{sec + nsec/1e9:.3f}s"
        except Exception:
            return str(dur)

    @staticmethod
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

    @staticmethod
    def _render_time_stats_table(rows) -> Text:
        headers = [
            "function name",
            "execution time (ms)",
            "elapsed (ms)",
            "frequency (Hz)",
        ]
        data = []
        for name, exec_t, elapsed, freq in rows:
            e_mean, e_std = exec_t      # ms
            l_mean, l_std = elapsed     # ms
            f_mean, f_std = freq        # Hz
            data.append([
                f"{name}",
                f"{e_mean:.3f} ± {e_std:.3f}",
                f"{l_mean:.3f} ± {l_std:.3f}",
                f"{f_mean:.2f} ± {f_std:.2f}",
            ])
        cols = list(zip(*([headers] + data))) if data else [headers]
        widths = [max(len(str(x)) for x in col) for col in cols]
        def fmt_row(cells):
            return " | ".join(str(c).ljust(w) for c, w in zip(cells, widths))
        sep = "-+-".join("-" * w for w in widths)
        lines = [fmt_row(headers), sep]
        for row in data:
            lines.append(fmt_row(row))
        table_str = "\n".join(lines)
        return Text(table_str, no_wrap=True)

    @staticmethod
    def _quat_to_yaw(q) -> float:
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def set_navstate_text(self, text: str) -> None:
        self._last_navstate_text = text
        if self.st_navstate is not None:
            if self.navstate_enabled:
                self.st_navstate.update(text)
            else:
                self.st_navstate.update("")

    def set_time_stats_rows(self, rows) -> None:
        table = self._render_time_stats_table(rows)
        self._last_timestats_text = table
        if self.st_timestats is not None:
            if self.timestats_enabled:
                self.st_timestats.update(table)
            else:
                self.st_timestats.update("")

    # ---------- ROS callbacks ----------
    def control_callback(self, msg) -> None:
        if self.box_nav_control is None:
            return

        t_val: int = msg.type
        label, color = NC_TYPE_MAP.get(t_val, (str(t_val), "white"))
        type_line = f"[{color}]{label}[/{color}]"

        pose_txt = self._fmt_pose(msg.current_pose) if msg.current_pose else "—"
        nav_time_txt = self._fmt_duration(msg.navigation_time) if msg.navigation_time else "—"
        eta_txt = self._fmt_duration(msg.estimated_time_remaining) if msg.estimated_time_remaining else "—"
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
        self.box_nav_control.update(text)

    def goal_info_callback(self, msg) -> None:
        if self.box_goal_info is None:
            return

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
            yaw = self._quat_to_yaw(pose0.orientation)
            first_goal_line = f"First goal: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f} rad"
        else:
            first_goal_line = "First goal: —"

        self.box_goal_info.update("\n".join([status_line, pos_line, ang_line, goals_line, first_goal_line]))

    def twist_callback(self, msg) -> None:
        self._last_twist_text = (
            "Twist:\n"
            f"  linear : x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}\n"
            f"  angular: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}"
        )
        self._update_twist_box()

    def twist_stamped_callback(self, msg) -> None:
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

    def navstate_callback(self, msg) -> None:
        text = msg.data if hasattr(msg, "data") else str(msg)
        self._last_navstate_text = text
        if self.navstate_enabled and self.st_navstate is not None:
            self.st_navstate.update(text)

    def _update_twist_box(self) -> None:
        if self.box_twist is None:
            return
        parts = []
        if self._last_twist_text != "—":
            parts.append(self._last_twist_text)
        if self._last_twiststamped_text != "—":
            parts.append(self._last_twiststamped_text)
        self.box_twist.update("\n\n".join(parts) if parts else "Esperando Twist/TwistStamped…")

    # ---------- Time stats: log tail + aggregation ----------
    def _open_log_if_needed(self) -> None:
        """Open the log file if available, preserving position; handle rotation/truncation."""
        try:
            st = os.stat(self._LOG_PATH)
        except FileNotFoundError:
            # file missing: close if we had it
            if self._log_fh:
                try:
                    self._log_fh.close()
                except Exception:
                    pass
            self._log_fh = None
            self._log_inode = None
            self._log_pos = 0
            return

        if self._log_fh is None:
            # first open: read from start to accumulate history
            self._log_fh = open(self._LOG_PATH, "r", encoding="utf-8", errors="ignore")
            self._log_inode = st.st_ino
            self._log_pos = 0
            return

        # if inode changed or file shrank: reopen and start from 0
        try:
            same_inode = (self._log_inode == st.st_ino)
            curr_size = st.st_size
            if (not same_inode) or (curr_size < self._log_pos):
                try:
                    self._log_fh.close()
                except Exception:
                    pass
                self._log_fh = open(self._LOG_PATH, "r", encoding="utf-8", errors="ignore")
                self._log_inode = st.st_ino
                self._log_pos = 0
        except Exception:
            pass

    @staticmethod
    def _shorten_name(full: str) -> str:
        # drop 'easynav::' prefix if present
        if full.startswith("easynav::"):
            return full[len("easynav::"):]
        return full

    @staticmethod
    def _sort_key_suffix(full: str) -> tuple[str, str]:
        # sort by suffix after last '::', then by full short name
        short = EasyNavTabbedApp._shorten_name(full)
        parts = short.split("::")
        suffix = parts[-1] if parts else short
        return (suffix, short)

    def _accum_sample(self, name: str, start_ns: int, end_ns: int) -> None:
        d = self._ts_stats.get(name)
        if d is None:
            d = {
                "exec": RunningStats(),    # ms
                "elapsed": RunningStats(), # ms
                "freq": RunningStats(),    # Hz
                "last_start": None,        # ns
            }
            self._ts_stats[name] = d

        # Execution time in ms (ns -> ms)
        exec_ns = max(0, end_ns - start_ns)
        exec_ms = exec_ns / 1_000_000.0
        d["exec"].update(exec_ms)

        last_start = d["last_start"]
        d["last_start"] = start_ns

        # Elapsed between consecutive starts (period) in ms; frequency in Hz
        if last_start is not None:
            elapsed_ns = max(0, start_ns - last_start)
            elapsed_ms = elapsed_ns / 1_000_000.0
            d["elapsed"].update(elapsed_ms)
            if elapsed_ns > 0:
                freq_hz = 1_000_000_000.0 / elapsed_ns
                d["freq"].update(freq_hz)

    def _poll_time_stats_log(self) -> None:
        """Read new lines from the log and update the Time stats table."""
        self._open_log_if_needed()
        if self._log_fh is None:
            # No file; render empty (or keep previous). Here we keep previous.
            return

        # seek to last known position and read what’s new
        try:
            self._log_fh.seek(self._log_pos)
            for line in self._log_fh:
                m = self._LOG_RE.match(line)
                if not m:
                    continue
                name = m.group("name")
                start_ns = int(m.group("start"))
                end_ns = int(m.group("end"))
                self._accum_sample(name, start_ns, end_ns)
            self._log_pos = self._log_fh.tell()
        except Exception:
            # On any IO/parsing error, do not crash the UI
            return

        # Build rows from accumulators
        rows = []
        for full_name, d in sorted(self._ts_stats.items(), key=lambda kv: self._sort_key_suffix(kv[0])):
            short = self._shorten_name(full_name)
            exec_mean, exec_std = d["exec"].as_tuple()            # μs
            elap_mean, elap_std = d["elapsed"].as_tuple()         # ms
            freq_mean, freq_std = d["freq"].as_tuple()            # Hz
            rows.append((short, (exec_mean, exec_std), (elap_mean, elap_std), (freq_mean, freq_std)))

        # Push to UI
        self.set_time_stats_rows(rows)

    def _ros_shutdown(self) -> None:
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
