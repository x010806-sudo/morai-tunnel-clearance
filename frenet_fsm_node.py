#!/usr/bin/env python3
"""MORAI ROS1 Pure Pursuit + Frenet obstacle-avoidance FSM.

Only this node may publish the final ``/ctrl_cmd`` while control is enabled.
Run the existing ``purepursuit_mgeo`` node with ``enable_control:=false``.
"""

from __future__ import annotations

import json
import math
import time
import threading
from dataclasses import replace
from typing import List, Optional, Sequence, Tuple

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64, String

from purepursuit_mgeo.frenet_planner import (
    FrenetCandidate,
    FrenetPlanner,
    ReferencePath,
    TrackedObstacle,
)
from purepursuit_mgeo.path import MgeoPurePursuit, PathPoint, load_mgeo_path


NORMAL = "NORMAL"
AVOIDANCE = "AVOIDANCE"
RETURN = "RETURN"


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def yaw_to_quaternion_z_w(yaw: float) -> Tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def float_list_parameter(name: str, default: Sequence[float]) -> List[float]:
    value = rospy.get_param("~" + name, list(default))
    if isinstance(value, str):
        value = value.strip().strip("[]")
        value = [] if not value else [part.strip() for part in value.split(",")]
    result = [float(item) for item in value]
    if not result:
        raise ValueError("{} must contain at least one number".format(name))
    return result


def stabilize_long_box_yaw(obstacle: TrackedObstacle, cache: dict, now: float) -> TrackedObstacle:
    """Ignore abrupt axis changes on long, thin BBOX tracks.

    A rectangle's yaw and yaw+pi describe the same shape, so compare the
    undirected box axis (modulo pi) before deciding that the tracker flipped.
    """
    if not (obstacle.yaw_valid and math.isfinite(obstacle.yaw)
            and obstacle.yaw_source.upper() == "BBOX"
            and obstacle.length >= 8.0 and obstacle.width <= 1.5):
        return obstacle
    previous = cache.get(obstacle.track_id)
    if previous is not None and now-previous["seen"] > 1.5:
        previous = None
    raw_yaw = obstacle.yaw
    if previous is None:
        cache[obstacle.track_id] = {"yaw": raw_yaw, "seen": now, "locked": False}
        return obstacle
    difference = raw_yaw-previous["yaw"]
    delta = 0.5*math.atan2(math.sin(2.0*difference),math.cos(2.0*difference))
    locked = previous["locked"] or abs(delta) > math.radians(35.0)
    used_yaw = previous["yaw"] if locked else raw_yaw
    cache[obstacle.track_id] = {"yaw": used_yaw, "seen": now, "locked": locked}
    if locked:
        return replace(obstacle,yaw=used_yaw,yaw_source="BBOX_STABILIZED")
    return obstacle


def planned_lateral_acceleration(candidate, vehicle, max_tracking_error):
    if candidate is None or len(candidate.points)<3:
        return 0.0
    x,y=vehicle[:2]
    i=min(range(len(candidate.points)),key=lambda j:math.hypot(candidate.points[j].x-x,candidate.points[j].y-y))
    if math.hypot(candidate.points[i].x-x,candidate.points[i].y-y)>max_tracking_error:
        return 0.0
    i=max(1,min(i,len(candidate.points)-2))
    t=candidate.times;d=candidate.lateral_offsets
    dt0=t[i]-t[i-1];dt1=t[i+1]-t[i]
    if min(dt0,dt1)<=0:
        return 0.0
    value=2.0*((d[i+1]-d[i])/dt1-(d[i]-d[i-1])/dt0)/(dt0+dt1)
    return value if math.isfinite(value) else 0.0


class GpsFreshnessGate:
    """Receipt gap of fresh converted GPS; repeated cached stamps do not count."""

    def __init__(self, now, timeout=1.0, recovery=1.0):
        self.started_at = now
        self.timeout = timeout
        self.recovery = recovery
        self.last_valid_at = None
        self.last_stamp = None
        self.recovery_since = None
        self.blackout = False

    def observe(self, now, stamp, x, y):
        if (not all(math.isfinite(v) for v in (now,stamp,x,y))
                or stamp <= 0.0 or abs(now-stamp) > self.timeout
                or stamp == self.last_stamp):
            return False
        if self.last_valid_at is None or now-self.last_valid_at >= self.timeout:
            self.recovery_since = now
        self.last_valid_at = now
        self.last_stamp = stamp
        return True

    def update(self, now):
        origin = self.started_at if self.last_valid_at is None else self.last_valid_at
        if now-origin >= self.timeout:
            self.blackout = True
            self.recovery_since = None
        elif (self.blackout and self.recovery_since is not None
              and now-self.recovery_since >= self.recovery):
            self.blackout = False
        return self.blackout


class RecentThreatMemory:
    """Briefly keep previously relevant objects through a detector dropout."""

    def __init__(self, hold_seconds=1.5):
        self.hold_seconds = hold_seconds
        self.records = {}

    def observe(self, obstacles, now):
        self.records = {k:v for k,v in self.records.items() if 0 <= now-v[0] <= self.hold_seconds}
        for o in obstacles:
            previous = self.records.get(o.track_id)
            self.records[o.track_id] = (now,o,False if previous is None else previous[2])

    def protect(self, track_ids):
        for track_id in track_ids:
            if track_id in self.records:
                t,o,_ = self.records[track_id]
                self.records[track_id] = (t,o,True)

    def supplement(self, obstacles, now, assume_static):
        result=list(obstacles);present={o.track_id for o in obstacles};held=[]
        for track_id,(seen,o,protected) in self.records.items():
            age=now-seen
            if protected and track_id not in present and 0 <= age <= self.hold_seconds:
                if not assume_static:
                    x,y=o.predicted(age);o=replace(o,center_x=x,center_y=y)
                result.append(o);held.append((track_id,round(age,3)))
        return result,held


class FrenetFsmNode:
    def __init__(self) -> None:
        rospy.init_node("frenet_fsm_controller", anonymous=False)
        self.lock = threading.RLock()

        path_file = rospy.get_param("~path_file")
        self.global_points = load_mgeo_path(path_file)
        self.reference = ReferencePath(self.global_points)

        self.enable_control = bool(rospy.get_param("~enable_control", False))
        self.command_topic = rospy.get_param("~command_topic", "/ctrl_cmd")
        self.odom_topic = rospy.get_param("~odometry_topic", "/localization/odometry")
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic", "/detection/obstacle_states"
        )
        self.pedestrian_stop_topic = rospy.get_param(
            "~pedestrian_stop_topic",
            "/perception/pedestrian_crossing/stop_required",
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.longitudinal_command_type = int(rospy.get_param("~longl_cmd_type", 2))

        self.normal_speed = float(rospy.get_param("~normal_speed_mps", 6.0))
        self.avoidance_speed = float(rospy.get_param("~avoidance_speed_mps", 3.0))
        self.return_speed = float(rospy.get_param("~return_speed_mps", 3.0))
        self.control_rate = float(rospy.get_param("~control_rate_hz", 20.0))
        self.planning_rate = float(rospy.get_param("~planning_rate_hz", 5.0))
        self.max_steering = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0))
        )
        self.steering_sign = float(rospy.get_param("~steering_sign", 1.0))
        self.wheelbase = float(rospy.get_param("~wheelbase_m", 3.0))
        self.lookahead_min = float(rospy.get_param("~lookahead_min_m", 4.0))
        self.lookahead_gain = float(rospy.get_param("~lookahead_gain", 0.35))
        self.return_lookahead_min = float(
            rospy.get_param("~return_lookahead_min_m", 8.0)
        )
        self.return_lookahead_gain = float(
            rospy.get_param("~return_lookahead_gain", 0.50)
        )
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance_m", 1.5))

        self.vehicle_length = float(rospy.get_param("~vehicle_length_m", 4.635))
        self.vehicle_width = float(rospy.get_param("~vehicle_width_m", 1.892))
        self.rear_axle_to_center = float(
            rospy.get_param("~rear_axle_to_center_m", 1.35)
        )
        self.lane_width = float(rospy.get_param("~lane_width_m", 3.5))
        self.maximum_prediction_time = float(rospy.get_param("~maximum_prediction_time_s", 4.0))
        # These limits must be read here and passed into FrenetPlanner.
        # Otherwise rosparam changes appear to succeed but the planner keeps
        # using FrenetPlanner's constructor defaults (3.0 m/s^2 and 0.28 1/m).
        self.maximum_lateral_acceleration = float(
            rospy.get_param("~maximum_lateral_acceleration_mps2", 100.0)
        )
        self.maximum_curvature = float(
            rospy.get_param("~maximum_curvature_1pm", 3.0)
        )
        if (not math.isfinite(self.maximum_lateral_acceleration)
                or self.maximum_lateral_acceleration <= 0.0
                or not math.isfinite(self.maximum_curvature)
                or self.maximum_curvature <= 0.0):
            raise ValueError("planner dynamics limits must be finite and positive")
        self.path_sequence = 0
        self.path_stamp = 0.0
        self.safety_margin = float(rospy.get_param("~safety_margin_m", 0.45))
        self.route_trigger_margin = float(
            rospy.get_param("~route_trigger_margin_m", 0.0)
        )
        self.assume_static_obstacles = bool(
            rospy.get_param("~assume_static_obstacles", False)
        )
        self.trigger_distance = float(
            rospy.get_param("~obstacle_trigger_distance_m", 30.0)
        )
        self.rear_ignore_distance = float(
            rospy.get_param("~obstacle_rear_distance_m", 3.0)
        )
        self.return_tolerance = float(rospy.get_param("~return_tolerance_m", 0.30))
        self.clear_hold_seconds = float(rospy.get_param("~clear_hold_s", 1.5))
        # No-stop policy: always continue on the map reference if Frenet fails.
        self.fallback_to_reference_on_avoidance_failure = True
        self.reference_fallback_active = False
        self.odom_stale_timeout = float(rospy.get_param("~odom_stale_timeout_s", 0.5))
        self.obstacle_stale_timeout = float(
            rospy.get_param("~obstacle_stale_timeout_s", 0.6)
        )
        self.require_obstacle_topic = bool(
            rospy.get_param("~require_obstacle_topic", True)
        )
        self.preferred_side = str(
            rospy.get_param("~preferred_avoidance_side", "left")
        )

        self.avoidance_offsets = float_list_parameter(
            "avoidance_offsets_m",
            [0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0, 2.5, -2.5, 3.0, -3.0, 3.5, -3.5, 3.7, -3.7, 3.8, -3.8, 4.0, -4.0, 4.5, -4.5, 5.0, -5.0, 5.5, -5.5, 6.0, -6.0],
        )
        self.planning_times = float_list_parameter(
            "planning_times_s", [2.0, 3.0, 4.0]
        )

        self.global_controller = self._new_controller(self.global_points)
        # Reference reacquisition uses Pure Pursuit directly.  Its longer
        # lookahead prevents the sharp steering that would result from aiming
        # at the geometrically nearest point itself.
        self.return_controller = self._new_controller(
            self.global_points,
            self.return_lookahead_min,
            self.return_lookahead_gain,
        )
        self.planner = FrenetPlanner(
            self.reference,
            vehicle_length=self.vehicle_length,
            vehicle_width=self.vehicle_width,
            rear_axle_to_center=self.rear_axle_to_center,
            safety_margin=self.safety_margin,
            maximum_lateral_acceleration=self.maximum_lateral_acceleration,
            maximum_curvature=self.maximum_curvature,
            preferred_side=self.preferred_side,
            maximum_prediction_time=self.maximum_prediction_time,
            route_trigger_margin=self.route_trigger_margin,
            assume_static_obstacles=self.assume_static_obstacles,
        )
        self.tunnel_mode = False
        self._profile_replan_required = False
        self.gps_health_topic = rospy.get_param("~gps_health_topic", "/localization/gps")
        self.gps_gate = GpsFreshnessGate(rospy.Time.now().to_sec(),
            float(rospy.get_param("~tunnel_gps_timeout_s",1.0)),
            float(rospy.get_param("~tunnel_gps_recovery_s",1.0)))
        self.tunnel_weight_offset=float(rospy.get_param("~tunnel_weight_offset",5.0))
        self.tunnel_weight_safety=float(rospy.get_param("~tunnel_weight_safety",40.0))
        self.tunnel_weight_clearance=float(rospy.get_param("~tunnel_weight_clearance",100.0))
        self.tunnel_clearance=float(rospy.get_param("~tunnel_preferred_clearance_m",1.0))
        self.tunnel_weight_completion=float(rospy.get_param("~tunnel_weight_completion",10.0))
        hold=float(rospy.get_param("~tunnel_obstacle_hold_s",1.5))
        if (not all(math.isfinite(v) and v >= 0 for v in
                    (self.gps_gate.timeout,self.gps_gate.recovery,self.tunnel_weight_offset,
                     self.tunnel_weight_safety,self.tunnel_weight_clearance,
                     self.tunnel_clearance,self.tunnel_weight_completion,hold))
                or self.gps_gate.timeout<=0 or self.tunnel_clearance<=0):
            raise ValueError("Invalid tunnel clearance parameters")
        self._normal_costs=(self.planner.weight_offset,self.planner.weight_safety,
                            self.planner.weight_clearance,self.planner.preferred_clearance,
                            self.planner.weight_completion)
        self.threat_memory=RecentThreatMemory(hold)
        self._memory_last_stamp=None
        self._held_obstacles=[]
        self._maneuver_finish_s=None
        self._maneuver_direction=0
        rospy.loginfo(
            "Frenet planner limits: max_lateral_accel=%.2f m/s^2 max_curvature=%.2f 1/m",
            self.maximum_lateral_acceleration,
            self.maximum_curvature,
        )
        rospy.loginfo(
            "Frenet obstacle prediction: assume_static_obstacles=%s",
            self.assume_static_obstacles,
        )

        self.latest_odom: Optional[Odometry] = None
        self.odom_received_at = 0.0
        self.obstacles: List[TrackedObstacle] = []
        self._bbox_yaw_cache = {}
        self.obstacles_received_at = 0.0
        self.obstacles_source_timestamp = 0.0
        self.pedestrian_stop_required = False

        self.state = NORMAL
        self.state_reason = "startup"
        self.active_candidate: Optional[FrenetCandidate] = None
        self.local_controller: Optional[MgeoPurePursuit] = None
        self.active_avoidance_offset: Optional[float] = None
        # RETURN is meaningful only after the vehicle committed to avoidance.
        self.return_required = False
        self._last_perception_log = 0.0
        self._last_published_path = None
        self.last_plan_at = 0.0
        self.clear_since: Optional[float] = None
        self._last_control_command = None

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(self.gps_health_topic, Odometry, self._gps_callback, queue_size=20)
        rospy.Subscriber(
            self.obstacle_topic, String, self._obstacle_callback, queue_size=1
        )
        rospy.Subscriber(
            self.pedestrian_stop_topic,
            Bool,
            self._pedestrian_stop_callback,
            queue_size=1,
        )
        self.command_publisher = None
        if self.enable_control:
            self.command_publisher = rospy.Publisher(
                self.command_topic, CtrlCmd, queue_size=1
            )
        self.state_publisher = rospy.Publisher(
            "/control/frenet_fsm/state", String, queue_size=1, latch=True
        )
        self.path_publisher = rospy.Publisher(
            "/control/frenet_fsm/local_path", Path, queue_size=1, latch=True
        )
        self.lookahead_publisher = rospy.Publisher(
            "/control/frenet_fsm/lookahead_point", PointStamped, queue_size=1
        )
        self.steering_publisher = rospy.Publisher(
            "/control/frenet_fsm/steering_preview", Float64, queue_size=1
        )
        rospy.logwarn("Frenet version: TUNNEL_CLEARANCE_V1; GPS-shadow cost/hold/continuation; planning=%.2fHz times=%s",
                      self.planning_rate,self.planning_times)
        rospy.loginfo("Frenet cost weights: jerk=%.2f offset=%.2f time=%.2f",
                      self.planner.weight_jerk,self.planner.weight_offset,self.planner.weight_time)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.control_rate, 1.0)),
            self._control_callback,
        )

        rospy.logwarn(
            "Frenet FSM ready: control=%s path_points=%d obstacle=%s cmd=%s "
            "offsets=%s reference_fallback_on_avoidance_failure=%s. "
            "The old Pure Pursuit publisher MUST use enable_control=false.",
            self.enable_control,
            len(self.global_points),
            self.obstacle_topic,
            self.command_topic,
            self.avoidance_offsets,
            self.fallback_to_reference_on_avoidance_failure,
        )

    def _new_controller(
        self,
        points: Sequence[PathPoint],
        lookahead_min: Optional[float] = None,
        lookahead_gain: Optional[float] = None,
    ) -> MgeoPurePursuit:
        return MgeoPurePursuit(
            points,
            self.wheelbase,
            self.lookahead_min if lookahead_min is None else lookahead_min,
            self.lookahead_gain if lookahead_gain is None else lookahead_gain,
            self.goal_tolerance,
            self.steering_sign,
        )

    def _odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.latest_odom = message
            self.odom_received_at = rospy.Time.now().to_sec()

    def _gps_callback(self, message: Odometry):
        # Converted GPS suppresses raw lat/lon=(0,0). EKF prediction is not GPS.
        with self.lock:
            self.gps_gate.observe(rospy.Time.now().to_sec(),message.header.stamp.to_sec(),
                                  message.pose.pose.position.x,message.pose.pose.position.y)

    def _update_tunnel_mode(self, now):
        with self.lock:
            tunnel=self.gps_gate.update(now)
        if tunnel==self.tunnel_mode:
            return
        self.tunnel_mode=tunnel
        p=self.planner
        values=(self.tunnel_weight_offset,self.tunnel_weight_safety,
                self.tunnel_weight_clearance,self.tunnel_clearance,self.tunnel_weight_completion) if tunnel else self._normal_costs
        p.weight_offset,p.weight_safety,p.weight_clearance,p.preferred_clearance,p.weight_completion=values
        self._profile_replan_required=True
        self._maneuver_finish_s=None;self._maneuver_direction=0
        self.last_plan_at=float('-inf')
        rospy.logwarn("Frenet tunnel clearance: active=%s offset=%.1f safety=%.1f clearance_weight=%.1f preferred_gap=%.2fm hold=%.2fs planning=%.2fHz times=%s",
            tunnel,p.weight_offset,p.weight_safety,p.weight_clearance,p.preferred_clearance,
            self.threat_memory.hold_seconds,self.planning_rate,self.planning_times)

    def _obstacle_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                raw_obstacles = payload.get("obstacles", [])
                source_timestamp = float(payload.get("timestamp", 0.0))
            elif isinstance(payload, list):
                raw_obstacles = payload
                source_timestamp = 0.0
            else:
                raise ValueError("top-level JSON must be an object or list")
            parsed = [TrackedObstacle.from_dict(item) for item in raw_obstacles]
        except (TypeError, ValueError, KeyError, json.JSONDecodeError) as error:
            rospy.logerr_throttle(1.0, "Invalid obstacle state JSON: %s", error)
            return
        received_at = rospy.Time.now().to_sec()
        with self.lock:
            for track_id in list(self._bbox_yaw_cache):
                if received_at-self._bbox_yaw_cache[track_id]["seen"] > 1.5:
                    del self._bbox_yaw_cache[track_id]
            stabilized = []
            for item in parsed:
                corrected = stabilize_long_box_yaw(item,self._bbox_yaw_cache,received_at)
                if corrected.yaw_source == "BBOX_STABILIZED":
                    delta = corrected.yaw-item.yaw
                    if abs(math.atan2(math.sin(delta),math.cos(delta))) > 1.e-6:
                        rospy.logwarn_throttle(1.0,
                            "Frenet obstacle BBOX yaw held: id=%d raw=%.3f used=%.3f L/W=%.2f/%.2f",
                            item.track_id,item.yaw,corrected.yaw,item.length,item.width)
                stabilized.append(corrected)
            self.obstacles = stabilized
            if source_timestamp<=0 or source_timestamp!=self._memory_last_stamp:
                self.threat_memory.observe(stabilized,received_at)
                self._memory_last_stamp=source_timestamp
            self.obstacles_received_at = received_at
            self.obstacles_source_timestamp = source_timestamp

    def _pedestrian_stop_callback(self, message: Bool) -> None:
        with self.lock:
            self.pedestrian_stop_required = bool(message.data)

    def _snapshot(self):
        with self.lock:
            return (
                self.latest_odom,
                self.odom_received_at,
                list(self.obstacles),
                self.obstacles_received_at,
                self.obstacles_source_timestamp,
                self.pedestrian_stop_required,
            )

    @staticmethod
    def _vehicle_state(message: Odometry) -> Tuple[float, float, float, float]:
        pose = message.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        speed = math.hypot(
            message.twist.twist.linear.x, message.twist.twist.linear.y
        )
        return pose.position.x, pose.position.y, yaw, speed

    def _transition(self, state: str, reason: str) -> None:
        if state != self.state:
            rospy.logwarn("Frenet FSM: %s -> %s (%s)", self.state, state, reason)
        self.state = state
        if state in (NORMAL,RETURN):
            self._maneuver_finish_s=None;self._maneuver_direction=0
        self.state_reason = reason
        if state == NORMAL:
            self.active_candidate = None
            self.local_controller = None
            self.active_avoidance_offset = None
            self.return_required = False
            self.clear_since = None

    def _obstacle_age(self, now: float, source_timestamp: float) -> float:
        if source_timestamp <= 0.0:
            return 0.0
        return max(0.0, min(now - source_timestamp, self.obstacle_stale_timeout))

    def _planning_due(self, now: float) -> bool:
        return now - self.last_plan_at >= 1.0 / max(self.planning_rate, 0.1)

    def _plan(
        self,
        now: float,
        vehicle: Tuple[float, float, float, float],
        obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
        target_offsets: Sequence[float],
        target_speed: float,
        preserve_on_failure: bool = False,
    ) -> bool:
        x_value, y_value, yaw, speed = vehicle
        plan_started = time.monotonic()
        # Only latch AVOIDANCE. RETURN retains the baseline reactive replanning.
        # A fixed valid path completes its original transition instead of
        # starting another 4-second transition at every planning tick.
        if (self.state == AVOIDANCE and self.active_candidate is not None
                and not self._profile_replan_required):
            valid, validation = self.planner.validate_remaining(
                self.active_candidate,vehicle,obstacles,obstacle_age,
                self.planner.vehicle_width/2.,target_speed)
            if valid:
                self.last_plan_at = rospy.Time.now().to_sec()
                if self._last_published_path is not None:
                    self.path_publisher.publish(self._last_published_path)
                rospy.loginfo_throttle(1.0,"Frenet kept avoidance path: d=%.2f T=%.2f validation=%s",
                    self.active_candidate.target_offset,self.active_candidate.planning_time,
                    json.dumps(validation,separators=(",", ":")))
                return True
            rospy.logwarn("Frenet avoidance replacement needed: %s",
                          json.dumps(validation,separators=(",", ":")))
            if validation.get('reason') in ('path_tracking_error','path_horizon_exhausted'):
                self._maneuver_finish_s=None;self._maneuver_direction=0
            if validation.get('track_id') is not None:
                with self.lock:self.threat_memory.protect([validation['track_id']])
        times = list(self.planning_times)
        projection=self.reference.project(x_value,y_value)
        budget=None;acceleration=0.0
        if self.tunnel_mode and self.state==AVOIDANCE:
            acceleration=planned_lateral_acceleration(self.active_candidate,vehicle,self.planner.vehicle_width/2.)
            if self._maneuver_finish_s is not None:
                budget=(self._maneuver_finish_s-projection.s)/max(1.0,speed,target_speed)
        candidate = self.planner.plan(
            x_value,
            y_value,
            yaw,
            speed,
            target_speed,
            obstacles,
            obstacle_age,
            target_offsets,
            times,
            initial_lateral_acceleration=acceleration,
            completion_budget=budget,
            continuation_direction=self._maneuver_direction if self.tunnel_mode else 0,
        )
        self.last_plan_at = rospy.Time.now().to_sec()
        duration_ms = (time.monotonic() - plan_started) * 1000.0
        diagnostics = getattr(self.planner, "last_diagnostics", {})
        diagnostics.update({"tunnel_mode":self.tunnel_mode,"held_obstacles":self._held_obstacles,
            "initial_lateral_acceleration":acceleration,"completion_budget_s":budget,
            "continuation_direction":self._maneuver_direction,
            "weights":{"offset":self.planner.weight_offset,"safety":self.planner.weight_safety,
                       "clearance":self.planner.weight_clearance,
                       "preferred_clearance_m":self.planner.preferred_clearance,
                       "completion":self.planner.weight_completion}})
        rospy.loginfo("Frenet plan summary: state=%s elapsed_ms=%.1f targets=%s times=%s diagnostics=%s",
                      self.state, duration_ms, list(target_offsets),times,
                      json.dumps(diagnostics, separators=(",", ":"), allow_nan=False))
        if candidate is None and self.active_candidate is not None:
            valid, validation = self.planner.validate_remaining(
                self.active_candidate,vehicle,obstacles,obstacle_age,
                self.planner.vehicle_width/2.,target_speed)
            if valid:
                # Keep only a currently checked path, never ignore new collisions.
                if self._last_published_path is not None:
                    self.path_publisher.publish(self._last_published_path)
                rospy.loginfo("Frenet retained previous path: d=%.2f validation=%s",
                    self.active_candidate.target_offset,json.dumps(validation,separators=(",", ":")))
                return True
            rospy.logwarn("Frenet previous path rejected: %s",
                          json.dumps(validation,separators=(",", ":")))
        if candidate is None:
            rospy.logwarn_throttle(2.0, "Frenet rejected candidates: %s",
                json.dumps(getattr(self.planner, "last_rejections", []), separators=(",", ":"), allow_nan=False))
            if not preserve_on_failure:
                self.active_candidate = None
                self.local_controller = None
            return False
        self.active_candidate = candidate
        self._profile_replan_required=False
        if self.tunnel_mode and self.state==AVOIDANCE:
            movement=candidate.target_offset-projection.d
            if self._maneuver_finish_s is None or movement*self._maneuver_direction < -0.15:
                self._maneuver_direction=1 if movement>0 else (-1 if movement<0 else 0)
                self._maneuver_finish_s=projection.s+max(1.0,speed,target_speed)*candidate.planning_time
        self.local_controller = self._new_controller(candidate.points)
        self._publish_candidate_path(candidate)
        rospy.loginfo(
            "Frenet selected: state=%s d=%.2f T=%.2f cost=%.3f clearance=%.2f",
            self.state,
            candidate.target_offset,
            candidate.planning_time,
            candidate.total_cost,
            candidate.minimum_clearance,
        )
        return True

    def _publish_candidate_path(self, candidate: FrenetCandidate) -> None:
        message = Path()
        message.header.stamp = rospy.Time.now()
        self.path_sequence += 1
        message.header.seq = self.path_sequence
        self.path_stamp = message.header.stamp.to_sec()
        message.header.frame_id = self.map_frame
        for point, yaw in zip(candidate.points, candidate.yaws):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion_z_w(yaw)
            message.poses.append(pose)
        self._last_published_path = message
        self.path_publisher.publish(message)

    def _follow(
        self,
        controller: MgeoPurePursuit,
        vehicle: Tuple[float, float, float, float],
        desired_speed: float,
        blocker_count: int,
    ) -> bool:
        x_value, y_value, yaw, speed = vehicle
        steering, path_stop, target, target_index, lookahead = controller.compute(
            x_value, y_value, yaw, speed
        )
        steering = max(-self.max_steering, min(self.max_steering, steering))
        target_message = PointStamped()
        target_message.header.stamp = rospy.Time.now()
        target_message.header.frame_id = self.map_frame
        target_message.point.x = target.x
        target_message.point.y = target.y
        target_message.point.z = target.z
        self.lookahead_publisher.publish(target_message)
        self.steering_publisher.publish(Float64(steering))
        if self.enable_control:
            self._last_control_command = self._make_command(steering, desired_speed)
            self.command_publisher.publish(self._last_control_command)
        self._publish_state(
            "local path ended" if path_stop else self.state_reason,
            blocker_count,
            steering,
            target_index,
            lookahead,
            stop_required=False,
        )
        return path_stop

    def _make_command(self, steering: float, speed: float) -> CtrlCmd:
        message = CtrlCmd()
        if hasattr(message, "longlCmdType"):
            message.longlCmdType = self.longitudinal_command_type
        if hasattr(message, "steering"):
            message.steering = steering
        if hasattr(message, "brake"):
            message.brake = 0.0
        if hasattr(message, "accel"):
            message.accel = 0.0
        if hasattr(message, "acceleration"):
            message.acceleration = 0.0
        if hasattr(message, "velocity"):
            message.velocity = max(0.0, speed)
        return message

    def _publish_state(
        self,
        reason: str,
        blocker_count: int,
        steering: float,
        target_index: int = -1,
        lookahead: float = 0.0,
        stop_required: bool = False,
    ) -> None:
        candidate = self.active_candidate
        payload = {
            "state": self.state,
            "reason": reason,
            "stop_required": False,
            "tunnel_mode": self.tunnel_mode,
            "held_obstacles": self._held_obstacles,
            "timestamp": rospy.Time.now().to_sec(),
            "path_sequence": self.path_sequence,
            "path_stamp": self.path_stamp,
            "corridor_width_m": self.lane_width,
            "blockers": self.planner.last_blocker_diagnostics,

            "control_enabled": self.enable_control,
            "blocker_count": blocker_count,
            "steering_rad": steering,
            "target_index": target_index,
            "lookahead_m": lookahead,
            "avoidance_offset_m": self.active_avoidance_offset,
            "candidate_cost": None if candidate is None else candidate.total_cost,
            "candidate_clearance_m": (
                None
                if candidate is None or not math.isfinite(candidate.minimum_clearance)
                else candidate.minimum_clearance
            ),
        }
        self.state_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def _enter_avoidance(
        self,
        now: float,
        vehicle: Tuple[float, float, float, float],
        obstacles: Sequence[TrackedObstacle],
        blocking_obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
    ) -> bool:
        self._transition(AVOIDANCE, "global-lane obstacle detected")
        target_offsets = self.planner.clearance_target_offsets(
            blocking_obstacles,
            obstacle_age,
            self.avoidance_offsets,
        )
        if not target_offsets:
            self._activate_reference_fallback("no configured offset clears obstacle BBox")
            return True
        if not self._plan(
            now,
            vehicle,
            obstacles,
            obstacle_age,
            target_offsets,
            self.avoidance_speed,
        ):
            self._activate_reference_fallback("no collision-free avoidance candidate")
            return True
        self.active_avoidance_offset = self.active_candidate.target_offset
        self.return_required = True
        self.clear_since = None
        return True

    def _activate_reference_fallback(self, reason: str) -> bool:
        """Continue on the map route if no Frenet path is available."""
        self.active_candidate = None
        self.local_controller = None
        self.active_avoidance_offset = None
        self.return_required = False
        self.reference_fallback_active = True
        self._transition(NORMAL, "avoidance failed; following map reference: " + reason)
        rospy.logwarn("Frenet fallback: following map reference despite blocker(s): %s", reason)
        return True

    def _plan_alternatives(self, now, vehicle, obstacles, blockers, obstacle_age, desired_speed):
        targets = self.planner.clearance_target_offsets(blockers,obstacle_age,self.avoidance_offsets)
        previous = self.active_avoidance_offset
        same_side = ([d for d in targets if d*previous>0.0]
                     if previous is not None and abs(previous)>1e-6 else targets)
        if same_side and self._plan(now,vehicle,obstacles,obstacle_age,same_side,desired_speed,
                                   preserve_on_failure=self.tunnel_mode):
            return True
        opposite = [d for d in targets if d not in same_side]
        if opposite:
            rospy.logwarn("Frenet same-side candidates unavailable; evaluating opposite side from current motion")
            return self._plan(now,vehicle,obstacles,obstacle_age,opposite,desired_speed,
                              preserve_on_failure=self.tunnel_mode)
        return False

    def _control_callback(self, _event: rospy.timer.TimerEvent) -> None:
        now = rospy.Time.now().to_sec()
        self._update_tunnel_mode(now)
        (
            odometry,
            odometry_received_at,
            obstacles,
            obstacles_received_at,
            source_timestamp,
            pedestrian_stop,
        ) = self._snapshot()

        if odometry is None:
            rospy.logwarn_throttle(1.0,
                "Frenet odometry missing; repeating last command without braking")
            if self.enable_control and self._last_control_command is not None:
                self.command_publisher.publish(self._last_control_command)
            return
        if now-odometry_received_at > self.odom_stale_timeout:
            rospy.logwarn_throttle(1.0,
                "Frenet odometry stale; continuing with latest pose")
        obstacles_fresh = (obstacles_received_at > 0.0
            and now-obstacles_received_at <= self.obstacle_stale_timeout)
        if self.require_obstacle_topic and not obstacles_fresh:
            rospy.logwarn_throttle(1.0,
                "Frenet obstacle input missing/stale; continuing reference tracking without braking")
            obstacles = []
        if pedestrian_stop:
            rospy.logwarn_throttle(1.0,
                "Frenet pedestrian stop request ignored by no-stop driving policy")

        self._held_obstacles=[]
        if self.tunnel_mode:
            with self.lock:
                obstacles,self._held_obstacles=self.threat_memory.supplement(
                    obstacles,now,self.assume_static_obstacles)
            if self._held_obstacles:
                rospy.logwarn_throttle(1.0,"Frenet tunnel held threats (id,age_s): %s",self._held_obstacles)

        vehicle = self._vehicle_state(odometry)
        x_value, y_value, _yaw, speed = vehicle
        ego_projection = self.reference.project(x_value, y_value)
        obstacle_age = self._obstacle_age(now, source_timestamp)
        blockers = self.planner.relevant_obstacles(
            obstacles,
            ego_projection.s,
            speed,
            self.vehicle_width,
            self.trigger_distance,
            self.rear_ignore_distance,
            obstacle_age,
        )
        with self.lock:self.threat_memory.protect([o.track_id for o in blockers])

        if now-self._last_perception_log >= 1.0:
            c, sn = math.cos(vehicle[2]),math.sin(vehicle[2])
            blocker_ids = {o.track_id for o in blockers}
            preview = []
            for o in obstacles:
                dx,dy=o.center_x-x_value,o.center_y-y_value
                forward,left=dx*c+dy*sn,-dx*sn+dy*c
                if -self.rear_ignore_distance <= forward <= self.trigger_distance:
                    preview.append({"id":o.track_id,"forward_m":round(forward,2),
                        "left_m":round(left,2),"length":round(o.length,2),
                        "width":round(o.width,2),"blocker":o.track_id in blocker_ids})
            rospy.loginfo("Frenet perception preview: state=%s obstacle_age=%.3f obstacles=%s",
                self.state,obstacle_age,json.dumps(preview,separators=(",", ":")))
            self._last_perception_log=now

        if self.reference_fallback_active:
            if blockers and not self._planning_due(now):
                self.state_reason = "avoidance unavailable; awaiting scheduled retry"
                self._follow(self.global_controller, vehicle, self.normal_speed, len(blockers))
                return
            if blockers:
                rospy.logwarn_throttle(1.0, "Frenet fallback retry: blockers=%d", len(blockers))
            # NORMAL below retries against the current input snapshot.
            self.reference_fallback_active = False

        if self.state == NORMAL:
            if blockers:
                if not self._enter_avoidance(now, vehicle, obstacles, blockers, obstacle_age):
                    self._activate_reference_fallback("avoidance planning failed")
                    self._follow(self.global_controller, vehicle, self.normal_speed, len(blockers))
                    return
                if self.reference_fallback_active:
                    self.state_reason = "avoidance unavailable; following map reference"
                    self._follow(self.global_controller, vehicle, self.normal_speed, len(blockers))
                    return
            else:
                self.state_reason = "global path tracking"
                self._follow(self.global_controller, vehicle, self.normal_speed, 0)
                return

        if self.state == AVOIDANCE:
            if blockers:
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = now
            elif (now - self.clear_since >= self.clear_hold_seconds
                  and self.active_avoidance_offset is not None
                  and abs(ego_projection.d-self.active_avoidance_offset) <= self.return_tolerance):
                self.active_candidate = None
                self.local_controller = None
                self.active_avoidance_offset = None
                self._transition(RETURN, "path clear; attempting return plan")

        if self.state == RETURN:
            if blockers:
                if not self._enter_avoidance(now, vehicle, obstacles, blockers, obstacle_age):
                    self._activate_reference_fallback("avoidance planning failed during return")
                    self._follow(self.global_controller, vehicle, self.normal_speed, len(blockers))
                    return
                if self.reference_fallback_active:
                    self.state_reason = "avoidance unavailable; following map reference"
                    self._follow(self.global_controller, vehicle, self.normal_speed, len(blockers))
                    return
            else:
                heading_error = math.atan2(math.sin(vehicle[2]-ego_projection.yaw),
                                           math.cos(vehicle[2]-ego_projection.yaw))
                if abs(ego_projection.d) <= self.return_tolerance and abs(heading_error) <= 0.08:
                    self._transition(NORMAL, "reference path reacquired")
                    self._follow(self.global_controller, vehicle, self.normal_speed, 0)
                    return

        # Both AVOIDANCE and RETURN must have a current, checked local path.
        desired_speed = self.return_speed if self.state == RETURN else self.avoidance_speed
        if self.local_controller is None or self._planning_due(now):
            targets = [0.0] if self.state == RETURN else (
                [self.active_avoidance_offset] if (self.active_avoidance_offset is not None
                                                 and not self._profile_replan_required)
                else self.avoidance_offsets)
            if self.tunnel_mode and self.state==AVOIDANCE and not self._profile_replan_required:
                planned = self._plan_alternatives(now,vehicle,obstacles,blockers,obstacle_age,desired_speed)
            else:
                planned = self._plan(now, vehicle, obstacles, obstacle_age, targets, desired_speed)
                if not planned and self.state == AVOIDANCE:
                    planned = self._plan_alternatives(now,vehicle,obstacles,blockers,obstacle_age,desired_speed)
            if not planned:
                self._activate_reference_fallback("no collision-free path inside route corridor")
                self.state_reason = "avoidance unavailable; following map reference"
                self._follow(self.global_controller, vehicle, self.normal_speed, len(blockers))
                return
            self.active_avoidance_offset = self.active_candidate.target_offset
        self.state_reason = "following validated " + self.state.lower() + " path"
        self._follow(self.local_controller, vehicle, desired_speed, len(blockers))


if __name__ == "__main__":
    try:
        FrenetFsmNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
