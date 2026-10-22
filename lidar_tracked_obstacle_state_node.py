#!/usr/bin/env python3
"""Publish driving-oriented obstacle states from the existing map tracker.

This node deliberately does not cluster point clouds or run another tracker.
It consumes the IDs, map positions, boxes, yaw, and velocities already
published by ``KalmanHungarianNode`` and only adds output smoothing and motion
state classification.
"""

import json
import math
import statistics
from collections import deque

import rospy
from std_msgs.msg import String

from lidar_perception.msg import LidarObstacleArray


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _limit_vector(x_value, y_value, maximum):
    magnitude = math.hypot(x_value, y_value)
    if magnitude <= maximum or magnitude < 1.0e-9:
        return x_value, y_value
    scale = maximum / magnitude
    return x_value * scale, y_value * scale


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _smooth_angle(previous, current, alpha):
    x_value = (1.0 - alpha) * math.cos(previous) + alpha * math.cos(current)
    y_value = (1.0 - alpha) * math.sin(previous) + alpha * math.sin(current)
    if math.hypot(x_value, y_value) < 1.0e-9:
        return previous
    return math.atan2(y_value, x_value)


class TrackedObstacleStatePublisher:
    """Add state labels to the existing map-frame tracking output."""

    def __init__(self):
        self.input_topic = _param(
            "input_topic", "/perception/lidar/tracked_obstacles_map"
        )
        self.output_topic = _param(
            "output_topic", "/detection/obstacle_states"
        )
        self.static_topic = _param(
            "static_topic", "/detection/static_obstacles"
        )
        self.dynamic_topic = _param(
            "dynamic_topic", "/detection/dynamic_obstacles"
        )

        self.velocity_alpha = float(_param("velocity_alpha", 0.40))
        self.max_output_accel_mps2 = float(
            _param("max_output_accel_mps2", 4.0)
        )
        self.max_output_speed_mps = float(_param("max_output_speed_mps", 12.0))

        self.bbox_history_size = int(_param("bbox_history_size", 5))
        self.bbox_alpha = float(_param("bbox_alpha", 0.30))
        self.bbox_max_relative_jump = float(
            _param("bbox_max_relative_jump", 0.40)
        )

        self.moving_enter_speed = float(_param("moving_enter_speed", 0.35))
        self.moving_exit_speed = float(_param("moving_exit_speed", 0.15))
        self.static_confirm_sec = float(_param("static_confirm_sec", 2.0))
        self.stopped_confirm_sec = float(_param("stopped_confirm_sec", 1.0))

        self.yaw_min_speed_mps = float(_param("yaw_min_speed_mps", 0.30))
        self.yaw_alpha = float(_param("yaw_alpha", 0.35))
        self.state_timeout_sec = float(_param("state_timeout_sec", 5.0))

        if self.bbox_history_size < 1:
            raise ValueError("bbox_history_size must be at least 1")
        if not 0.0 < self.velocity_alpha <= 1.0:
            raise ValueError("velocity_alpha must be in (0, 1]")
        if not 0.0 < self.bbox_alpha <= 1.0:
            raise ValueError("bbox_alpha must be in (0, 1]")
        if not 0.0 < self.yaw_alpha <= 1.0:
            raise ValueError("yaw_alpha must be in (0, 1]")
        if self.moving_exit_speed > self.moving_enter_speed:
            raise ValueError("moving_exit_speed cannot exceed moving_enter_speed")

        self.velocity_states = {}
        self.bbox_histories = {}
        self.filtered_bbox = {}
        self.motion_states = {}
        self.yaw_states = {}
        self.last_seen = {}

        self.output_publisher = rospy.Publisher(
            self.output_topic, String, queue_size=1
        )
        self.static_publisher = rospy.Publisher(
            self.static_topic, String, queue_size=1
        )
        self.dynamic_publisher = rospy.Publisher(
            self.dynamic_topic, String, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            LidarObstacleArray,
            self._callback,
            queue_size=1,
        )

        rospy.logwarn(
            "Tracked obstacle state publisher: input=%s outputs=%s,%s,%s; "
            "no point-cloud clustering or additional Hungarian tracker is run",
            self.input_topic,
            self.output_topic,
            self.static_topic,
            self.dynamic_topic,
        )

    def _stabilize_velocity(self, track_id, raw_vx, raw_vy, timestamp):
        raw_vx, raw_vy = _limit_vector(
            raw_vx, raw_vy, self.max_output_speed_mps
        )
        previous = self.velocity_states.get(track_id)
        if previous is None:
            self.velocity_states[track_id] = {
                "vx": raw_vx,
                "vy": raw_vy,
                "timestamp": timestamp,
            }
            return raw_vx, raw_vy

        dt = max(0.01, min(timestamp - previous["timestamp"], 1.0))
        delta_vx, delta_vy = _limit_vector(
            raw_vx - previous["vx"],
            raw_vy - previous["vy"],
            self.max_output_accel_mps2 * dt,
        )
        gated_vx = previous["vx"] + delta_vx
        gated_vy = previous["vy"] + delta_vy
        alpha = self.velocity_alpha
        filtered_vx = alpha * gated_vx + (1.0 - alpha) * previous["vx"]
        filtered_vy = alpha * gated_vy + (1.0 - alpha) * previous["vy"]
        self.velocity_states[track_id] = {
            "vx": filtered_vx,
            "vy": filtered_vy,
            "timestamp": timestamp,
        }
        return filtered_vx, filtered_vy

    def _stabilize_bbox(self, track_id, raw_length, raw_width):
        history = self.bbox_histories.get(track_id)
        if history is None:
            history = {
                "length": deque(maxlen=self.bbox_history_size),
                "width": deque(maxlen=self.bbox_history_size),
            }
            self.bbox_histories[track_id] = history
        history["length"].append(max(0.0, raw_length))
        history["width"].append(max(0.0, raw_width))
        median_length = statistics.median(history["length"])
        median_width = statistics.median(history["width"])

        previous = self.filtered_bbox.get(track_id)
        if previous is None:
            self.filtered_bbox[track_id] = {
                "length": median_length,
                "width": median_width,
            }
            return median_length, median_width

        def limit_change(old_value, new_value):
            if old_value <= 1.0e-6:
                return new_value
            ratio = self.bbox_max_relative_jump
            return max(
                old_value * (1.0 - ratio),
                min(new_value, old_value * (1.0 + ratio)),
            )

        limited_length = limit_change(previous["length"], median_length)
        limited_width = limit_change(previous["width"], median_width)
        alpha = self.bbox_alpha
        length = alpha * limited_length + (1.0 - alpha) * previous["length"]
        width = alpha * limited_width + (1.0 - alpha) * previous["width"]
        self.filtered_bbox[track_id] = {"length": length, "width": width}
        return length, width

    def _update_motion_state(self, track_id, speed, timestamp):
        state = self.motion_states.get(track_id)
        if state is None:
            state = {
                "state": "UNKNOWN",
                "ever_moved": False,
                "first_seen": timestamp,
                "low_speed_since": timestamp,
            }
            self.motion_states[track_id] = state

        if speed >= self.moving_enter_speed:
            state["state"] = "MOVING"
            state["ever_moved"] = True
            state["low_speed_since"] = None
            return state["state"]

        if state["state"] == "MOVING" and speed > self.moving_exit_speed:
            state["low_speed_since"] = None
            return state["state"]

        if speed <= self.moving_exit_speed:
            if state["low_speed_since"] is None:
                state["low_speed_since"] = timestamp
            low_duration = timestamp - state["low_speed_since"]
            if state["ever_moved"]:
                if low_duration >= self.stopped_confirm_sec:
                    state["state"] = "STOPPED"
            elif timestamp - state["first_seen"] >= self.static_confirm_sec:
                state["state"] = "STATIC"
        return state["state"]

    def _update_yaw(self, track_id, box_yaw, vx, vy):
        speed = math.hypot(vx, vy)
        previous = self.yaw_states.get(track_id)

        # Shape orientation comes from the detected OBB, not motion heading.
        # A noisy tracked velocity must not rotate a long wall's collision box.
        if math.isfinite(box_yaw):
            state = {
                "yaw": _normalize_angle(box_yaw),
                "valid": True,
                "source": "BBOX",
            }
        elif speed >= self.yaw_min_speed_mps:
            state = {
                "yaw": math.atan2(vy, vx),
                "valid": True,
                "source": "VELOCITY",
            }
        elif previous is not None and previous["valid"]:
            state = {"yaw": previous["yaw"], "valid": True, "source": "HOLD"}
        else:
            state = {"yaw": 0.0, "valid": False, "source": "UNKNOWN"}

        self.yaw_states[track_id] = state
        return state["yaw"], state["valid"], state["source"]

    def _cleanup(self, timestamp):
        stale_ids = [
            track_id
            for track_id, last_seen in self.last_seen.items()
            if timestamp - last_seen > self.state_timeout_sec
        ]
        dictionaries = (
            self.velocity_states,
            self.bbox_histories,
            self.filtered_bbox,
            self.motion_states,
            self.yaw_states,
            self.last_seen,
        )
        for track_id in stale_ids:
            for dictionary in dictionaries:
                dictionary.pop(track_id, None)

    @staticmethod
    def _publish(publisher, timestamp, obstacles):
        payload = {
            "timestamp": float(timestamp),
            "obstacle_count": len(obstacles),
            "obstacles": obstacles,
        }
        publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def _callback(self, message):
        stamp = message.header.stamp
        timestamp = stamp.to_sec() if stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()

        all_obstacles = []
        static_obstacles = []
        dynamic_obstacles = []
        for tracked in message.obstacles:
            track_id = int(tracked.id)
            self.last_seen[track_id] = timestamp
            vx, vy = self._stabilize_velocity(
                track_id,
                float(tracked.velocity_x_map),
                float(tracked.velocity_y_map),
                timestamp,
            )
            speed = math.hypot(vx, vy)
            length, width = self._stabilize_bbox(
                track_id, float(tracked.length), float(tracked.width)
            )
            motion_state = self._update_motion_state(track_id, speed, timestamp)
            yaw, yaw_valid, yaw_source = self._update_yaw(
                track_id, float(tracked.yaw), vx, vy
            )

            obstacle = {
                "id": track_id,
                "center_x_map": float(tracked.center_x_map),
                "center_y_map": float(tracked.center_y_map),
                "length": float(length),
                "width": float(width),
                "velocity_x_map": float(vx),
                "velocity_y_map": float(vy),
                "speed_mps": float(speed),
                "motion_state": motion_state,
                "yaw": float(yaw),
                "yaw_deg": float(math.degrees(yaw)),
                "yaw_valid": bool(yaw_valid),
                "yaw_source": yaw_source,
            }
            all_obstacles.append(obstacle)
            if motion_state == "STATIC":
                static_obstacles.append(obstacle)
            else:
                # UNKNOWN is a conservative dynamic candidate until enough
                # history exists. STOPPED remains dynamic because an actor
                # such as a pedestrian may pause and move again.
                dynamic_obstacles.append(obstacle)

        self._cleanup(timestamp)
        self._publish(self.output_publisher, timestamp, all_obstacles)
        self._publish(self.static_publisher, timestamp, static_obstacles)
        self._publish(self.dynamic_publisher, timestamp, dynamic_obstacles)
        rospy.loginfo_throttle(
            1.0,
            "Obstacle states: input=%d all=%d static=%d dynamic=%d",
            message.obstacle_count,
            len(all_obstacles),
            len(static_obstacles),
            len(dynamic_obstacles),
        )


def main():
    rospy.init_node("lidar_tracked_obstacle_state")
    TrackedObstacleStatePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
