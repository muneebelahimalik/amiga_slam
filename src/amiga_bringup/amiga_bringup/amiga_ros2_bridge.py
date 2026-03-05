#!/usr/bin/env python3
"""
amiga_ros2_bridge.py — native ROS 2 / farm-ng gRPC bridge (no ROS 1 required).

Connects directly to the Amiga's gRPC services over Tailscale (or LAN) using
the farm-ng Python SDK — the same SDK used by filter_client.py.

This node REPLACES the ROS 1 amiga_ros_bridge + ros1_bridge combination.

Published topics
----------------
/amiga/vel   (geometry_msgs/TwistStamped)
    Measured wheel velocity from the Amiga canbus service.
    Consumed by amiga_odometry → /wheel_odom for SLAM.

/amiga/pose  (nav_msgs/Odometry, header.frame_id="world")
    Filter state from the Amiga's on-board state estimator (GPS + IMU).
    Only published when has_converged is true.  Coordinates are in the
    filter's "world" frame (typically UTM-based; origin at first fix).
    In the lab this may never publish due to weak GPS — that is expected.

Subscribed topics
-----------------
/cmd_vel  (geometry_msgs/Twist)
    Velocity commands from Nav2 or teleop.  Forwarded to the Amiga canbus
    service as AmigaRpdo1 (cmd_speed, cmd_angular_rate).

ROS 2 parameters
----------------
host          (str,   default 'camphor-clone.tail0be07.ts.net')
    Tailscale hostname or IP of the Amiga brain.

canbus_port   (int,   default 6001)
    gRPC port of the Amiga canbus service.

filter_port   (int,   default 20001)
    gRPC port of the Amiga filter (state estimation) service.

max_linear    (float, default 1.5)
    Hard clamp on commanded |linear.x| (m/s).

max_angular   (float, default 1.0)
    Hard clamp on commanded |angular.z| (rad/s).

publish_unconverged_filter (bool, default false)
    When true, also publish /amiga/pose when GPS has not converged.
    Useful for debugging; leave false in production.

Architecture
------------
asyncio runs in a daemon thread alongside rclpy.spin().

  background thread (asyncio)
    ├── _subscribe_canbus()  →  self._vel_pub.publish()
    ├── _subscribe_filter()  →  self._pose_pub.publish()
    └── _send_commands()     ←  self._cmd_queue (fed by /cmd_vel subscriber)

  main thread (rclpy.spin)
    └── /cmd_vel subscriber  →  self._cmd_queue

Velocity command note
---------------------
Commands are sent to the canbus gRPC service as AmigaRpdo1 messages.
The exact gRPC service stub method depends on the farm-ng SDK version
installed on this machine.  If command sending fails (logs a warning),
check the farm-ng canbus service proto and adjust _send_commands() below.
"""

from __future__ import annotations

import asyncio
import math
import queue
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

try:
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
    from google.protobuf import json_format
    _FARM_NG_CORE = True
except ImportError:
    _FARM_NG_CORE = False

try:
    from farm_ng.canbus.canbus_pb2 import AmigaTpdo1, AmigaRpdo1
    _FARM_NG_CANBUS = True
except ImportError:
    _FARM_NG_CANBUS = False

try:
    from farm_ng_core_pybind import Pose3F64
    _FARM_NG_PYBIND = True
except ImportError:
    _FARM_NG_PYBIND = False


def _make_event_service_config(name: str, host: str, port: int,
                                path: str, query: str) -> 'EventServiceConfig':
    """Build EventServiceConfig from parameters (same JSON structure as service_config.json)."""
    config_dict = {
        'name': name,
        'host': host,
        'port': port,
        'subscriptions': [
            {'uri': {'path': path, 'query': query}, 'every_n': 1}
        ],
    }
    return json_format.ParseDict(config_dict, EventServiceConfig())


class AmigaRos2Bridge(Node):

    def __init__(self):
        super().__init__('amiga_ros2_bridge')

        if not _FARM_NG_CORE:
            self.get_logger().fatal(
                'farm-ng Python SDK not found.  Install with:\n'
                '  pip install farm-ng-amiga\n'
                'or follow https://github.com/farm-ng/amiga-dev-kit instructions.'
            )
            raise RuntimeError('farm_ng not installed')

        # ---- parameters ----
        self.declare_parameter('host', 'camphor-clone.tail0be07.ts.net')
        self.declare_parameter('canbus_port', 6001)
        self.declare_parameter('filter_port', 20001)
        self.declare_parameter('max_linear', 1.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('publish_unconverged_filter', False)

        self._host = self.get_parameter('host').value
        self._canbus_port = int(self.get_parameter('canbus_port').value)
        self._filter_port = int(self.get_parameter('filter_port').value)
        self._max_lin = float(self.get_parameter('max_linear').value)
        self._max_ang = float(self.get_parameter('max_angular').value)
        self._pub_unconverged = self.get_parameter('publish_unconverged_filter').value

        # ---- publishers ----
        self._vel_pub = self.create_publisher(TwistStamped, '/amiga/vel', 10)
        self._pose_pub = self.create_publisher(Odometry, '/amiga/pose', 10)

        # ---- velocity command queue (thread-safe, ROS → asyncio) ----
        self._cmd_queue: queue.Queue[Optional[Twist]] = queue.Queue(maxsize=5)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        self.get_logger().info(
            f'amiga_ros2_bridge connecting to {self._host} '
            f'(canbus:{self._canbus_port}, filter:{self._filter_port})'
        )

        # ---- asyncio in background daemon thread ----
        self._loop = asyncio.new_event_loop()
        self._bg_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._bg_thread.start()

    # ------------------------------------------------------------------
    # ROS 2 callbacks (main thread)
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: Twist) -> None:
        """Put /cmd_vel in queue; async sender picks it up at 10 Hz."""
        try:
            self._cmd_queue.put_nowait(msg)
        except queue.Full:
            pass  # drop old command if queue is full

    # ------------------------------------------------------------------
    # Asyncio loop (background thread)
    # ------------------------------------------------------------------

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._async_main())

    async def _async_main(self) -> None:
        tasks = [
            asyncio.create_task(self._subscribe_canbus()),
            asyncio.create_task(self._subscribe_filter()),
            asyncio.create_task(self._send_commands()),
        ]
        done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
        for task in pending:
            task.cancel()
        for task in done:
            exc = task.exception()
            if exc:
                self.get_logger().error(f'Async task error: {exc}')

    # ------------------------------------------------------------------
    # Canbus subscriber → /amiga/vel
    # ------------------------------------------------------------------

    async def _subscribe_canbus(self) -> None:
        """Read AmigaTpdo1 from canbus service, publish /amiga/vel."""
        if not _FARM_NG_CANBUS:
            self.get_logger().warn(
                'farm_ng.canbus not available — /amiga/vel will NOT be published.'
            )
            return

        config = _make_event_service_config(
            'canbus', self._host, self._canbus_port,
            '/state', 'service_name=canbus',
        )

        while rclpy.ok():
            try:
                self.get_logger().info('Connecting to canbus service…')
                async for event, message in EventClient(config).subscribe(
                    config.subscriptions[0], decode=True
                ):
                    if not isinstance(message, AmigaTpdo1):
                        continue

                    msg = TwistStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.twist.linear.x = float(message.measured_speed)
                    msg.twist.angular.z = float(message.measured_angular_rate)
                    self._vel_pub.publish(msg)

            except Exception as exc:
                self.get_logger().warn(
                    f'canbus subscribe error: {exc!r}  — retrying in 3 s'
                )
                await asyncio.sleep(3.0)

    # ------------------------------------------------------------------
    # Filter subscriber → /amiga/pose
    # ------------------------------------------------------------------

    async def _subscribe_filter(self) -> None:
        """Read FilterState from filter service, publish /amiga/pose."""
        config = _make_event_service_config(
            'filter', self._host, self._filter_port,
            '/state', 'service_name=filter',
        )

        while rclpy.ok():
            try:
                self.get_logger().info('Connecting to filter service…')
                async for event, message in EventClient(config).subscribe(
                    config.subscriptions[0], decode=True
                ):
                    if not message.has_converged and not self._pub_unconverged:
                        continue  # skip unconverged estimates in production

                    odom = Odometry()
                    odom.header.stamp = self.get_clock().now().to_msg()
                    # "world" frame = filter's global reference (UTM-based)
                    odom.header.frame_id = 'world'
                    odom.child_frame_id = 'robot'

                    if _FARM_NG_PYBIND:
                        pose = Pose3F64.from_proto(message.pose)
                        odom.pose.pose.position.x = pose.translation[0]
                        odom.pose.pose.position.y = pose.translation[1]
                        odom.pose.pose.position.z = (
                            pose.translation[2] if len(pose.translation) > 2 else 0.0
                        )
                    else:
                        # Fallback: read pose fields directly from proto if pybind not installed
                        odom.pose.pose.position.x = message.pose.translation.x \
                            if hasattr(message.pose, 'translation') else 0.0
                        odom.pose.pose.position.y = message.pose.translation.y \
                            if hasattr(message.pose, 'translation') else 0.0

                    # Heading → quaternion (heading is yaw in radians)
                    h = message.heading
                    odom.pose.pose.orientation.z = math.sin(h * 0.5)
                    odom.pose.pose.orientation.w = math.cos(h * 0.5)

                    # Covariance from filter uncertainty diagonal [x, y, heading]
                    unc = message.uncertainty_diagonal
                    if unc and len(unc.data) >= 3:
                        odom.pose.covariance[0] = unc.data[0] ** 2   # xx
                        odom.pose.covariance[7] = unc.data[1] ** 2   # yy
                        odom.pose.covariance[35] = unc.data[2] ** 2  # yawyaw

                    self._pose_pub.publish(odom)

            except Exception as exc:
                self.get_logger().warn(
                    f'filter subscribe error: {exc!r}  — retrying in 3 s'
                )
                await asyncio.sleep(3.0)

    # ------------------------------------------------------------------
    # Velocity command sender: /cmd_vel → Amiga canbus
    # ------------------------------------------------------------------

    async def _send_commands(self) -> None:
        """
        Drain the cmd_queue at 10 Hz and forward commands to the Amiga
        canbus service as AmigaRpdo1 messages.

        Command sending uses grpc.aio directly since EventClient is designed
        primarily for subscribe streaming.  If the CanbusService stub method
        name differs in your farm-ng SDK version, adjust the call below.
        """
        if not _FARM_NG_CANBUS:
            self.get_logger().warn(
                'farm_ng.canbus not available — /cmd_vel commands will NOT be sent.'
            )
            return

        try:
            import grpc.aio as aio
            from farm_ng.canbus import canbus_service_pb2_grpc
            _grpc_available = True
        except ImportError:
            self.get_logger().warn(
                'grpc.aio or canbus service stub not available — '
                'velocity commands will be logged but not forwarded.'
            )
            _grpc_available = False

        channel = None
        stub = None
        last_cmd = Twist()          # default: zero velocity
        send_interval = 0.1         # 10 Hz command rate

        while rclpy.ok():
            await asyncio.sleep(send_interval)

            # Pull latest command from queue (skip stale ones)
            while not self._cmd_queue.empty():
                try:
                    last_cmd = self._cmd_queue.get_nowait()
                except queue.Empty:
                    break

            # Clamp velocities
            lin = max(-self._max_lin, min(self._max_lin, last_cmd.linear.x))
            ang = max(-self._max_ang, min(self._max_ang, last_cmd.angular.z))

            if not _grpc_available:
                if lin != 0.0 or ang != 0.0:
                    self.get_logger().debug(
                        f'cmd_vel (not forwarded): linear={lin:.2f} angular={ang:.2f}'
                    )
                continue

            # Lazy-init gRPC channel
            if channel is None:
                try:
                    channel = aio.insecure_channel(f'{self._host}:{self._canbus_port}')
                    stub = canbus_service_pb2_grpc.CanbusServiceStub(channel)
                    self.get_logger().info(
                        f'canbus command channel opened to {self._host}:{self._canbus_port}'
                    )
                except Exception as exc:
                    self.get_logger().warn(f'Failed to open canbus command channel: {exc!r}')
                    channel = None
                    continue

            # Send AmigaRpdo1 command
            try:
                rpdo = AmigaRpdo1(cmd_speed=lin, cmd_angular_rate=ang)
                await stub.send(rpdo)
            except Exception as exc:
                self.get_logger().warn(
                    f'canbus command send failed: {exc!r}  '
                    '(check canbus_service_pb2_grpc.CanbusServiceStub.send() API)'
                )
                # Reset channel so it reconnects on next iteration
                try:
                    await channel.close()
                except Exception:
                    pass
                channel = None
                stub = None


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = AmigaRos2Bridge()
        rclpy.spin(node)
    except RuntimeError as exc:
        # Node raised during __init__ (e.g. missing SDK)
        rclpy.logging.get_logger('amiga_ros2_bridge').fatal(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
