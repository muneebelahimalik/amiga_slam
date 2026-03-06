"""
autonomous_row_coverage.py — Autonomous boustrophedon row coverage with live SLAM.

No field polygon required.  Place the robot at the START of the first onion row,
call ~/start, and the robot:

  1. Drives straight for `row_length` metres (the current row)
  2. Executes a precise end-of-row maneuver:
       a) buffer forward past the crop edge (clears the row)
       b) drives sideways across row_spacing metres to the next row
       c) drives buffer_distance back to the row edge (aligned with next row)
  3. Drives the next row in the opposite direction
  4. Repeats for `num_rows` rows

Throughout, RTAB-Map SLAM is building a full field map.  When coverage is done
you have both a completed weed-control pass AND a saved map for future runs.

End-of-row maneuver geometry (adapted from farm-ng end_of_row_maneuver.py):

  Even row (→ direction):                    Odd row (← direction):

  ═══════════════════► ──► ──► buf           buf ◄── ◄──◄═══════════════════
  row i                   ↓                           ↑
                          ↓ row_spacing               ↑ row_spacing
  ◄═══════════════════ ◄── ◄── buf           buf ──► ──►════════════════════►
  row i+1

  The robot always turns left at the right field edge and right at the left edge
  (so rows always stack in the same lateral direction regardless of which end
  the robot is currently at).

ROS 2 interface
---------------
Services:
  ~/start       (std_srvs/Trigger) — begin autonomous coverage from current pose
  ~/stop        (std_srvs/Trigger) — cancel active coverage
  ~/mark_row_end (std_srvs/Trigger) — manually signal current position = row end
                                       (only used when row_length param == 0)

Topics published:
  ~/coverage_path (nav_msgs/Path)     — full planned path for RViz visualisation
  ~/current_row   (std_msgs/Int32)    — index of the row currently being driven

Parameters (set in launch or config/autonomous_coverage.yaml)
--------------------------------------------------------------
  row_length        float  Row length in metres. Set to 0 to use ~/mark_row_end
                           to record the actual row end on the first pass.
  num_rows          int    Number of rows to cover.  Default: 20.
  row_spacing       float  Distance between row centres (m).  Default: 0.45.
  buffer_distance   float  How far past the crop to drive before turning (m).
                           Default: 1.5. Must be > robot_length/2 (0.55 m).
  nav_frame         str    TF frame for waypoints. Default: map.
  map_frame         str    Frame RTAB-Map publishes the map in. Default: map.
  base_frame        str    Robot base frame. Default: base_link.

Typical usage
-------------
  # 1. Drive the Amiga to the start of the first onion row, facing along the row.
  # 2. In one terminal:
  ros2 launch amiga_bringup autonomous_coverage.launch.py row_length:=45.0 num_rows:=22

  # 3. Once the stack is live, trigger coverage:
  ros2 service call /autonomous_row_coverage/start std_srvs/srv/Trigger {}

  # 4. Stop at any time:
  ros2 service call /autonomous_row_coverage/stop std_srvs/srv/Trigger {}

  # If row_length is unknown, use mark_row_end on the first pass:
  ros2 launch amiga_bringup autonomous_coverage.launch.py row_length:=0.0 num_rows:=22
  ros2 service call /autonomous_row_coverage/start std_srvs/srv/Trigger {}
  # ... robot drives straight, you call this when it reaches the row end:
  ros2 service call /autonomous_row_coverage/mark_row_end std_srvs/srv/Trigger {}
  # Robot then auto-generates and drives all remaining rows.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import TransformException


# ---------------------------------------------------------------------------
# Geometry utilities
# ---------------------------------------------------------------------------

def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _make_pose(x: float, y: float, yaw: float, frame: str, stamp) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = stamp
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation = _quat_from_yaw(yaw)
    return ps


def _advance(x: float, y: float, yaw: float, dist: float):
    """Move (x, y) by dist metres along yaw."""
    return x + dist * math.cos(yaw), y + dist * math.sin(yaw)


def _lateral(x: float, y: float, yaw: float, dist: float):
    """Move (x, y) by dist metres perpendicular-left of yaw."""
    lat_yaw = yaw + math.pi / 2.0
    return x + dist * math.cos(lat_yaw), y + dist * math.sin(lat_yaw)


# ---------------------------------------------------------------------------
# Waypoint generation
# ---------------------------------------------------------------------------

def _end_of_row_maneuver(x, y, heading, row_spacing, buf, frame, stamp):
    """
    Three waypoints that move the robot from the end of one row to the start
    of the next row (lateral direction = left of current heading).

    Mirrors farm-ng end_of_row_maneuver.py:
      1. drive buffer_distance past the row end  (clear crop)
      2. cross row_spacing sideways               (move to next row)
      3. drive buffer_distance back toward field  (align with row edge)

    Intermediate "turn-in-place" is handled by Nav2's DWB RotateToGoal critic.
    """
    # 1. Past the row end (still facing row direction)
    bx, by = _advance(x, y, heading, buf)

    # 2. Across to the next row (heading reversed after crossing)
    #    The next row runs in the opposite direction, so after we cross we face heading+π
    cx, cy = _lateral(bx, by, heading, row_spacing)
    next_heading = heading + math.pi

    # 3. Back toward the row edge (now in the new heading direction)
    nx, ny = _advance(cx, cy, next_heading, buf)

    return [
        _make_pose(bx, by, heading,      frame, stamp),
        _make_pose(cx, cy, next_heading, frame, stamp),
        _make_pose(nx, ny, next_heading, frame, stamp),
    ]


def generate_boustrophedon_from_pose(x0, y0, yaw0, row_length, row_spacing,
                                     num_rows, buf, frame, clock):
    """
    Build the complete boustrophedon waypoint list starting from (x0, y0, yaw0).

    Returns list of PoseStamped waypoints suitable for FollowWaypoints.
    The caller is already at the very first row's start position; the first
    waypoint is therefore the END of row 0 (not the start).
    """
    now = clock.now().to_msg()
    waypoints = []

    # fwd = unit vector along row 0 direction
    # lat = unit vector perpendicular-left (direction rows stack)
    fwd_yaw = yaw0
    lat_yaw = yaw0 + math.pi / 2.0

    for i in range(num_rows):
        # Origin of row i (in map frame)
        # Rows stack laterally; even rows run forward (+fwd), odd rows run backward (-fwd)
        row_ox = x0 + i * row_spacing * math.cos(lat_yaw)
        row_oy = y0 + i * row_spacing * math.sin(lat_yaw)

        if i % 2 == 0:
            # Even row: start at row_ox/oy, drive forward
            heading = fwd_yaw
            end_x, end_y = _advance(row_ox, row_oy, heading, row_length)
        else:
            # Odd row: start at far end of row, drive backward
            start_x, start_y = _advance(row_ox, row_oy, fwd_yaw, row_length)
            heading = fwd_yaw + math.pi
            end_x, end_y = _advance(start_x, start_y, heading, row_length)

        # Waypoint: end of this row
        waypoints.append(_make_pose(end_x, end_y, heading, frame, now))

        # End-of-row maneuver (skip after the last row)
        if i < num_rows - 1:
            maneuver = _end_of_row_maneuver(
                end_x, end_y, heading, row_spacing, buf, frame, now
            )
            waypoints.extend(maneuver)

    return waypoints


# ---------------------------------------------------------------------------
# Node states
# ---------------------------------------------------------------------------

class State:
    IDLE = 'IDLE'
    LEARNING = 'LEARNING'    # driving first row to record row length
    NAVIGATING = 'NAVIGATING'
    DONE = 'DONE'


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class AutonomousRowCoverageNode(Node):

    def __init__(self):
        super().__init__('autonomous_row_coverage')

        # --- Parameters ---
        self.declare_parameter('row_length', 0.0)         # 0 = auto via mark_row_end
        self.declare_parameter('num_rows', 20)
        self.declare_parameter('row_spacing', 0.45)
        self.declare_parameter('buffer_distance', 1.5)
        self.declare_parameter('nav_frame', 'map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Nav2 action clients ---
        self._wp_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # --- Publishers ---
        self._path_pub = self.create_publisher(Path, '~/coverage_path', 10)
        self._row_pub = self.create_publisher(Int32, '~/current_row', 10)

        # --- Services ---
        self.create_service(Trigger, '~/start', self._start_cb)
        self.create_service(Trigger, '~/stop', self._stop_cb)
        self.create_service(Trigger, '~/mark_row_end', self._mark_row_end_cb)

        # --- Internal state ---
        self._state = State.IDLE
        self._goal_handle = None
        self._learn_start_pose = None   # (x, y, yaw) when ~/start was called in LEARNING mode
        self._nav_goal_handle = None    # NavigateToPose goal (used in LEARNING mode)

        self.get_logger().info(
            'AutonomousRowCoverage ready.\n'
            '  row_length=%.1f m  (0 = call ~/mark_row_end when you reach row end)\n'
            '  num_rows=%d\n'
            '  row_spacing=%.2f m\n'
            '  buffer_distance=%.1f m\n'
            'Call ~/start when robot is at the START of row 0, facing along the row.',
            self.get_parameter('row_length').value,
            self.get_parameter('num_rows').value,
            self.get_parameter('row_spacing').value,
            self.get_parameter('buffer_distance').value,
        )

    # ------------------------------------------------------------------
    # TF pose retrieval
    # ------------------------------------------------------------------

    def _get_current_pose(self):
        """Return (x, y, yaw) of base_frame in nav_frame, or None on failure."""
        nav_frame = self.get_parameter('nav_frame').value
        base_frame = self.get_parameter('base_frame').value
        try:
            tf = self.tf_buffer.lookup_transform(
                nav_frame, base_frame, rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = 2.0 * math.atan2(q.z, q.w)
            return x, y, yaw
        except TransformException as e:
            self.get_logger().error(f'TF lookup {nav_frame}→{base_frame} failed: {e}')
            return None

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _start_cb(self, request, response):
        if self._state != State.IDLE:
            response.success = False
            response.message = f'Already active (state={self._state}). Call ~/stop first.'
            return response

        pose = self._get_current_pose()
        if pose is None:
            response.success = False
            response.message = 'Cannot read current pose from TF. Is SLAM running?'
            return response

        x0, y0, yaw0 = pose
        row_length = self.get_parameter('row_length').value

        if row_length <= 0.0:
            # LEARNING mode: drive straight, wait for mark_row_end
            self._state = State.LEARNING
            self._learn_start_pose = (x0, y0, yaw0)
            self.get_logger().info(
                f'LEARNING mode — robot at ({x0:.2f}, {y0:.2f}), heading {math.degrees(yaw0):.1f}°. '
                'Drive to the end of row 0, then call ~/mark_row_end.'
            )
            # Start driving forward; let operator call mark_row_end to stop
            # We send a NavigateToPose goal far ahead along the row heading.
            # The operator cancels it (via mark_row_end) when they reach the row end.
            self._drive_straight_open_ended(x0, y0, yaw0)
            response.success = True
            response.message = (
                'LEARNING: driving row 0. Call ~/mark_row_end when you reach the row end.'
            )
        else:
            # Known row_length: generate all waypoints and start immediately
            self._state = State.NAVIGATING
            self.get_logger().info(
                f'Starting coverage: row_length={row_length} m, '
                f'num_rows={self.get_parameter("num_rows").value}, '
                f'start=({x0:.2f}, {y0:.2f}), heading={math.degrees(yaw0):.1f}°'
            )
            self._launch_coverage(x0, y0, yaw0, row_length)
            response.success = True
            response.message = (
                f'Coverage started from ({x0:.2f}, {y0:.2f}). '
                f'row_length={row_length:.1f} m, '
                f'{self.get_parameter("num_rows").value} rows.'
            )

        return response

    def _stop_cb(self, request, response):
        if self._state == State.IDLE:
            response.success = False
            response.message = 'Nothing active.'
            return response

        self.get_logger().info('Stop requested — cancelling active Nav2 goal.')
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

        self._state = State.IDLE
        response.success = True
        response.message = 'Coverage stopped.'
        return response

    def _mark_row_end_cb(self, request, response):
        if self._state != State.LEARNING:
            response.success = False
            response.message = 'Not in LEARNING mode (call ~/start with row_length:=0 first).'
            return response

        pose = self._get_current_pose()
        if pose is None:
            response.success = False
            response.message = 'Cannot read current pose.'
            return response

        # Cancel the open-ended forward drive
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

        x1, y1, _ = pose
        x0, y0, yaw0 = self._learn_start_pose

        # Compute learned row length from driven distance
        row_length = math.hypot(x1 - x0, y1 - y0)
        self.get_logger().info(
            f'Row end marked at ({x1:.2f}, {y1:.2f}). '
            f'Learned row_length = {row_length:.2f} m.'
        )

        if row_length < 0.5:
            response.success = False
            response.message = f'Row length {row_length:.2f} m is too short — move further first.'
            self._state = State.IDLE
            return response

        # Now generate the full coverage from the true row start
        self._state = State.NAVIGATING
        self._launch_coverage(x0, y0, yaw0, row_length)
        response.success = True
        response.message = f'Row length learned: {row_length:.2f} m. Coverage underway.'
        return response

    # ------------------------------------------------------------------
    # Navigation helpers
    # ------------------------------------------------------------------

    def _drive_straight_open_ended(self, x0, y0, yaw0):
        """
        Send a NavigateToPose goal 200 m ahead — effectively drives until cancelled.
        Used in LEARNING mode so the operator can stop the robot at the row end.
        """
        # 200 m is further than any realistic field row
        far_x, far_y = _advance(x0, y0, yaw0, 200.0)
        nav_frame = self.get_parameter('nav_frame').value
        now = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(far_x, far_y, yaw0, nav_frame, now)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/navigate_to_pose action server not available.')
            self._state = State.IDLE
            return

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._learning_goal_accepted_cb)

    def _learning_goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Learning drive goal rejected by Nav2.')
            self._state = State.IDLE
            return
        self._nav_goal_handle = handle
        self.get_logger().info('Learning drive accepted — call ~/mark_row_end when at row end.')

    def _launch_coverage(self, x0, y0, yaw0, row_length):
        """Generate all waypoints and dispatch to /follow_waypoints."""
        num_rows = self.get_parameter('num_rows').value
        row_spacing = self.get_parameter('row_spacing').value
        buf = self.get_parameter('buffer_distance').value
        nav_frame = self.get_parameter('nav_frame').value

        waypoints = generate_boustrophedon_from_pose(
            x0, y0, yaw0, row_length, row_spacing, num_rows, buf, nav_frame, self.get_clock()
        )

        n_rows = num_rows
        n_maneuvers = num_rows - 1
        total_wps = n_rows + n_maneuvers * 3   # 1 wp/row + 3 wp/maneuver
        self.get_logger().info(
            f'Coverage plan: {len(waypoints)} waypoints '
            f'({n_rows} rows, {n_maneuvers} end-of-row maneuvers)'
        )

        self._publish_path(waypoints, nav_frame)

        if not self._wp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/follow_waypoints action server not available.')
            self._state = State.IDLE
            return

        goal = FollowWaypoints.Goal()
        goal.poses = waypoints
        future = self._wp_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Coverage waypoint goal rejected by Nav2.')
            self._state = State.IDLE
            return
        self._goal_handle = handle
        self.get_logger().info('Coverage goal accepted by Nav2. Robot is driving.')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        wp_idx = feedback_msg.feedback.current_waypoint
        # Decode which row we're on from the waypoint index
        # Pattern: 1 wp per row end + 3 wp per maneuver = 4 wp per row (except last)
        row = wp_idx // 4
        msg = Int32()
        msg.data = row
        self._row_pub.publish(msg)
        self.get_logger().info(
            f'Row {row} — waypoint {wp_idx}',
            throttle_duration_sec=10.0,
        )

    def _result_cb(self, future):
        result = future.result().result
        missed = list(result.missed_waypoints)
        self._goal_handle = None
        self._state = State.DONE
        if missed:
            self.get_logger().warn(
                f'Coverage complete. '
                f'Missed {len(missed)} waypoints (indices: {missed}). '
                f'Consider reducing speed or increasing goal tolerance.'
            )
        else:
            self.get_logger().info(
                'Coverage complete — all rows finished. '
                'SLAM map is ready. Save it with: bash scripts/save_map.sh ~/maps/field'
            )

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def _publish_path(self, waypoints, frame):
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = waypoints
        self._path_pub.publish(path)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousRowCoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
