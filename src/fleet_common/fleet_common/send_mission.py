#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys

from fleet_interfaces.msg import MissionCommand, MissionState

TAKEOFF = MissionCommand.TAKEOFF
LAND = MissionCommand.LAND
WAYPOINT = MissionCommand.WAYPOINT
HOME = MissionCommand.HOME

MISSION_FINISHED = MissionState.MISSION_FINISHED


def build_plan_items(
    coords: list[tuple[float, float, float]],
    altitude: float,
    hold: float | None = None,
) -> list[dict]:
    items: list[dict] = [{"id": 1, "kind": TAKEOFF, "altitude_agl": altitude}]
    for index, (lat, lon, alt) in enumerate(coords, start=2):
        item = {
            "id": index,
            "kind": WAYPOINT,
            "latitude": lat,
            "longitude": lon,
            "altitude_agl": alt,
        }
        if hold is not None:
            item["hold_s"] = hold
        items.append(item)
    items.append({"id": len(items) + 1, "kind": HOME})
    items.append({"id": len(items) + 1, "kind": LAND})
    return items


def parse_coords(raw: list[str], altitude: float) -> list[tuple[float, float, float]]:
    coords: list[tuple[float, float, float]] = []
    for entry in raw:
        parts = entry.split(",")
        if len(parts) == 2:
            coords.append((float(parts[0]), float(parts[1]), altitude))
        elif len(parts) == 3:
            coords.append((float(parts[0]), float(parts[1]), float(parts[2])))
        else:
            raise ValueError(f"parse coordinate '{entry}' failed: cause: expected lat,lon[,alt]")
    return coords


def main() -> int:
    parser = argparse.ArgumentParser(description="Send a mission to a vehicle's mission server.")
    parser.add_argument(
        "--namespace", required=True, help="ROS namespace of the vehicle, such as aeroscout1"
    )
    parser.add_argument("waypoints", nargs="+", help="GPS coordinates as lat,lon[,alt_agl]")
    parser.add_argument(
        "--altitude",
        type=float,
        default=10.0,
        help="Altitude above home ground (m) for takeoff and waypoints without one",
    )
    parser.add_argument("--hold", type=float, default=None, help="Hold at each waypoint (s)")
    parser.add_argument(
        "--home",
        default=None,
        help="Home position as lat,lon; the vehicle's current position on navsat when omitted",
    )
    parser.add_argument("--mission-id", type=int, default=1)
    args = parser.parse_args()

    try:
        coords = parse_coords(args.waypoints, args.altitude)
        home = parse_coords([args.home], 0.0)[0] if args.home else None
    except ValueError as exc:
        parser.error(str(exc))

    import rclpy
    from fleet_interfaces.action import ExecuteMission
    from fleet_interfaces.msg import MissionCommand, MissionPlan
    from geographic_msgs.msg import GeoPoint
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    from fleet_common.qos import latched_qos

    plan = MissionPlan()
    plan.id = args.mission_id
    for item in build_plan_items(coords, args.altitude, args.hold):
        command = MissionCommand()
        command.id = item["id"]
        command.kind = item["kind"]
        command.latitude = item.get("latitude", 0.0)
        command.longitude = item.get("longitude", 0.0)
        command.altitude_agl = item.get("altitude_agl", 0.0)
        command.hold_s = item.get("hold_s", 0.0)
        command.yaw_deg = float("nan")
        plan.commands.append(command)

    rclpy.init()
    node = rclpy.create_node("send_mission", namespace=args.namespace)
    client = ActionClient(node, ExecuteMission, "execute_mission")
    outcome = {"status": None, "reason": ""}
    readiness = {"ready": False, "reason": None}
    fixes: list[NavSatFix] = []

    def on_state(message):
        if not message.ready and message.not_ready_reason != readiness["reason"]:
            print(f"waiting: mission server not ready: {message.not_ready_reason}")
        readiness["ready"] = message.ready
        readiness["reason"] = message.not_ready_reason

    node.create_subscription(
        MissionState,
        "mission_state",
        on_state,
        latched_qos(),
    )
    try:
        if home is None:
            position = node.create_subscription(
                NavSatFix, "navsat", fixes.append, qos_profile_sensor_data
            )
            print("waiting for the vehicle position on navsat for home")
            while not fixes:
                rclpy.spin_once(node)
            node.destroy_subscription(position)
            fix = fixes[0]
            if fix.status.status < NavSatStatus.STATUS_FIX:
                print(
                    f"read home position failed: cause: navsat status {fix.status.status} is not a fix",
                    file=sys.stderr,
                )
                return 1
            home = (fix.latitude, fix.longitude)
            print(f"home at the vehicle position {home[0]:.7f},{home[1]:.7f}")
        plan.home = GeoPoint(latitude=home[0], longitude=home[1], altitude=0.0)
        print("waiting for the mission server to be ready for missions")
        while not readiness["ready"]:
            rclpy.spin_once(node)
        client.wait_for_server()
        goal = ExecuteMission.Goal()
        goal.plan = plan

        def on_feedback(message):
            feedback = message.feedback
            print(f"command {feedback.command_id} status {feedback.status}")

        send = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            print(
                "send mission failed: cause: the mission server rejected the plan", file=sys.stderr
            )
            return 1
        print(f"mission {plan.id} accepted with {len(plan.commands)} commands")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result
        outcome["status"] = result.status
        outcome["reason"] = result.reason
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if outcome["status"] == MISSION_FINISHED:
        print(f"mission {plan.id} finished")
        return 0
    print(
        f"mission {plan.id} ended with status {outcome['status']}: {outcome['reason']}",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())



# #!/usr/bin/env python3

# from __future__ import annotations

# import argparse
# import sys

# # Import message definitions early because their enum-like constants are used
# # outside main(). The remaining ROS imports are deferred until after CLI parsing.
# from fleet_interfaces.msg import MissionCommand, MissionState

# # Mission command types.
# TAKEOFF = MissionCommand.TAKEOFF
# LAND = MissionCommand.LAND
# WAYPOINT = MissionCommand.WAYPOINT
# HOME = MissionCommand.HOME

# # Mission completion state used to determine the process exit status.
# MISSION_FINISHED = MissionState.MISSION_FINISHED


# def build_plan_items(
#     coords: list[tuple[float, float, float]],
#     altitude: float,
#     hold: float | None = None,
# ) -> list[dict]:
#     """Build the ordered list of commands that make up a mission.

#     The generated mission is:

#         TAKEOFF
#         WAYPOINT...
#         HOME
#         LAND

#     Args:
#         coords:
#             Waypoints represented as (latitude, longitude, altitude_agl).
#         altitude:
#             Takeoff altitude above ground level in meters.
#         hold:
#             Optional hold time at each waypoint in seconds.

#     Returns:
#         A list of dictionaries describing mission commands.
#     """

#     # Every mission starts by taking off to the requested default altitude.
#     items: list[dict] = [{"id": 1, "kind": TAKEOFF, "altitude_agl": altitude}]

#     # Command IDs start at 2 because command 1 is the TAKEOFF command.
#     for index, (lat, lon, alt) in enumerate(coords, start=2):
#         item = {
#             "id": index,
#             "kind": WAYPOINT,
#             "latitude": lat,
#             "longitude": lon,
#             "altitude_agl": alt,
#         }

#         # Only include a hold value if one was explicitly requested.
#         if hold is not None:
#             item["hold_s"] = hold

#         items.append(item)

#     # After visiting all waypoints, command the vehicle to return home.
#     # len(items) + 1 gives the next sequential command ID.
#     items.append({"id": len(items) + 1, "kind": HOME})

#     # Finish the mission by landing after returning home.
#     items.append({"id": len(items) + 1, "kind": LAND})

#     return items


# def parse_coords(raw: list[str], altitude: float) -> list[tuple[float, float, float]]:
#     """Parse command-line coordinate strings.

#     Accepted formats are:

#         latitude,longitude
#         latitude,longitude,altitude

#     If altitude is omitted, the supplied default altitude is used.
#     """

#     coords: list[tuple[float, float, float]] = []

#     for entry in raw:
#         parts = entry.split(",")

#         if len(parts) == 2:
#             # No waypoint-specific altitude was supplied, so use the
#             # command-line default.
#             coords.append((float(parts[0]), float(parts[1]), altitude))

#         elif len(parts) == 3:
#             # A waypoint-specific altitude overrides the default altitude.
#             coords.append((float(parts[0]), float(parts[1]), float(parts[2])))

#         else:
#             raise ValueError(
#                 f"parse coordinate '{entry}' failed: cause: expected lat,lon[,alt]"
#             )

#     return coords


# def main() -> int:
#     """Parse arguments, construct a mission, send it, and wait for completion."""

#     # -------------------------------------------------------------------------
#     # Command-line arguments
#     # -------------------------------------------------------------------------

#     parser = argparse.ArgumentParser(
#         description="Send a mission to a vehicle's mission server."
#     )

#     parser.add_argument(
#         "--namespace",
#         required=True,
#         help="ROS namespace of the vehicle, such as aeroscout1",
#     )

#     # At least one waypoint must be supplied.
#     parser.add_argument(
#         "waypoints",
#         nargs="+",
#         help="GPS coordinates as lat,lon[,alt_agl]",
#     )

#     parser.add_argument(
#         "--altitude",
#         type=float,
#         default=10.0,
#         help="Altitude above home ground (m) for takeoff and waypoints without one",
#     )

#     parser.add_argument(
#         "--hold",
#         type=float,
#         default=None,
#         help="Hold at each waypoint (s)",
#     )

#     parser.add_argument(
#         "--home",
#         default=None,
#         help=(
#             "Home position as lat,lon; "
#             "the vehicle's current position on navsat when omitted"
#         ),
#     )

#     parser.add_argument("--mission-id", type=int, default=1)

#     args = parser.parse_args()

#     # Parse and validate waypoint/home coordinate strings before starting ROS.
#     # parser.error() prints the error and terminates with argparse's normal
#     # command-line error handling.
#     try:
#         coords = parse_coords(args.waypoints, args.altitude)

#         # A supplied home position has no AGL altitude component, so pass 0.0
#         # as the default altitude and discard it below when assigning plan.home.
#         home = parse_coords([args.home], 0.0)[0] if args.home else None

#     except ValueError as exc:
#         parser.error(str(exc))

#     # -------------------------------------------------------------------------
#     # ROS imports
#     # -------------------------------------------------------------------------
#     #
#     # These imports are intentionally inside main(), allowing argument parsing
#     # and helper functions to be used before the ROS runtime is initialized.

#     import rclpy

#     from fleet_interfaces.action import ExecuteMission
#     from fleet_interfaces.msg import MissionCommand, MissionPlan
#     from geographic_msgs.msg import GeoPoint
#     from rclpy.action import ActionClient
#     from rclpy.qos import qos_profile_sensor_data
#     from sensor_msgs.msg import NavSatFix, NavSatStatus

#     from fleet_common.qos import latched_qos

#     # -------------------------------------------------------------------------
#     # Build the ROS MissionPlan message
#     # -------------------------------------------------------------------------

#     plan = MissionPlan()
#     plan.id = args.mission_id

#     # Convert the intermediate dictionary representation into ROS messages.
#     for item in build_plan_items(coords, args.altitude, args.hold):
#         command = MissionCommand()

#         command.id = item["id"]
#         command.kind = item["kind"]

#         # Fields that do not apply to a particular command default to zero.
#         # For example, TAKEOFF does not require latitude/longitude.
#         command.latitude = item.get("latitude", 0.0)
#         command.longitude = item.get("longitude", 0.0)
#         command.altitude_agl = item.get("altitude_agl", 0.0)
#         command.hold_s = item.get("hold_s", 0.0)

#         # NaN indicates that the mission does not request a particular yaw.
#         command.yaw_deg = float("nan")

#         plan.commands.append(command)

#     # -------------------------------------------------------------------------
#     # ROS node and action client setup
#     # -------------------------------------------------------------------------

#     rclpy.init()

#     # The namespace selects which vehicle this process communicates with.
#     node = rclpy.create_node("send_mission", namespace=args.namespace)

#     # Connect to the vehicle's ExecuteMission action server.
#     client = ActionClient(node, ExecuteMission, "execute_mission")

#     # Mutable containers are used so the subscription callbacks can update
#     # state visible to the enclosing main() function.
#     outcome = {"status": None, "reason": ""}
#     readiness = {"ready": False, "reason": None}

#     # Received NavSatFix messages are appended here while waiting for a home
#     # position.
#     fixes: list[NavSatFix] = []

#     def on_state(message):
#         """Handle mission-server readiness updates."""

#         # Avoid repeatedly printing the same not-ready reason.
#         if not message.ready and message.not_ready_reason != readiness["reason"]:
#             print(
#                 f"waiting: mission server not ready: "
#                 f"{message.not_ready_reason}"
#             )

#         readiness["ready"] = message.ready
#         readiness["reason"] = message.not_ready_reason

#     # Mission state is latched so a newly started client can receive the latest
#     # state immediately rather than waiting for the next state transition.
#     node.create_subscription(
#         MissionState,
#         "mission_state",
#         on_state,
#         latched_qos(),
#     )

#     try:
#         # ---------------------------------------------------------------------
#         # Determine the mission home position
#         # ---------------------------------------------------------------------

#         if home is None:
#             # No --home argument was supplied. Use the first GPS fix received
#             # from the vehicle instead.
#             position = node.create_subscription(
#                 NavSatFix,
#                 "navsat",
#                 fixes.append,
#                 qos_profile_sensor_data,
#             )

#             print("waiting for the vehicle position on navsat for home")

#             # Process ROS callbacks until at least one NavSatFix is received.
#             while not fixes:
#                 rclpy.spin_once(node)

#             # The temporary subscription is no longer needed once a fix arrives.
#             node.destroy_subscription(position)

#             fix = fixes[0]

#             # Reject the position if the GPS receiver does not consider it a
#             # valid fix.
#             if fix.status.status < NavSatStatus.STATUS_FIX:
#                 print(
#                     (
#                         "read home position failed: cause: "
#                         f"navsat status {fix.status.status} is not a fix"
#                     ),
#                     file=sys.stderr,
#                 )
#                 return 1

#             home = (fix.latitude, fix.longitude)

#             print(
#                 f"home at the vehicle position "
#                 f"{home[0]:.7f},{home[1]:.7f}"
#             )

#         # MissionPlan.home uses a geographic GeoPoint. Its altitude is set to
#         # zero because mission command altitudes are expressed as AGL.
#         plan.home = GeoPoint(
#             latitude=home[0],
#             longitude=home[1],
#             altitude=0.0,
#         )

#         # ---------------------------------------------------------------------
#         # Wait until the mission server declares itself ready
#         # ---------------------------------------------------------------------

#         print("waiting for the mission server to be ready for missions")

#         while not readiness["ready"]:
#             rclpy.spin_once(node)

#         # Readiness comes from MissionState, while wait_for_server() confirms
#         # that the ROS action server itself is available.
#         client.wait_for_server()

#         # ---------------------------------------------------------------------
#         # Send the mission
#         # ---------------------------------------------------------------------

#         goal = ExecuteMission.Goal()
#         goal.plan = plan

#         def on_feedback(message):
#             """Print progress feedback from the running mission."""

#             feedback = message.feedback
#             print(
#                 f"command {feedback.command_id} "
#                 f"status {feedback.status}"
#             )

#         # Sending an action goal is asynchronous.
#         send = client.send_goal_async(
#             goal,
#             feedback_callback=on_feedback,
#         )

#         # Spin the node until the server has accepted or rejected the goal.
#         rclpy.spin_until_future_complete(node, send)

#         handle = send.result()

#         # An absent goal handle or accepted=False means the server rejected
#         # the mission before execution began.
#         if handle is None or not handle.accepted:
#             print(
#                 (
#                     "send mission failed: cause: "
#                     "the mission server rejected the plan"
#                 ),
#                 file=sys.stderr,
#             )
#             return 1

#         print(
#             f"mission {plan.id} accepted "
#             f"with {len(plan.commands)} commands"
#         )

#         # ---------------------------------------------------------------------
#         # Wait for mission completion
#         # ---------------------------------------------------------------------

#         result_future = handle.get_result_async()

#         # Continue processing ROS callbacks until the action finishes.
#         rclpy.spin_until_future_complete(node, result_future)

#         result = result_future.result().result

#         # Save the result so it can be evaluated after ROS cleanup.
#         outcome["status"] = result.status
#         outcome["reason"] = result.reason

#     except KeyboardInterrupt:
#         # Treat Ctrl+C as an unsuccessful/incomplete mission execution.
#         print("interrupted", file=sys.stderr)
#         return 1

#     finally:
#         # Always release ROS resources, including on errors or Ctrl+C.
#         node.destroy_node()
#         rclpy.try_shutdown()

#     # -------------------------------------------------------------------------
#     # Translate mission result into the program's exit status
#     # -------------------------------------------------------------------------

#     if outcome["status"] == MISSION_FINISHED:
#         print(f"mission {plan.id} finished")
#         return 0

#     print(
#         (
#             f"mission {plan.id} ended with status "
#             f"{outcome['status']}: {outcome['reason']}"
#         ),
#         file=sys.stderr,
#     )

#     return 1


# if __name__ == "__main__":
#     # Propagate main()'s success/failure status to the shell.
#     sys.exit(main())