"""Frenet candidate generation and collision checking for MORAI ROS1.

The reference path and every obstacle are expressed in the same ``map``
frame.  Path points represent the vehicle rear-axle centre, matching the
existing Pure Pursuit controller.
"""

from __future__ import annotations

import bisect
import math
import numpy as np
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from purepursuit_mgeo.path import PathPoint


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class FrenetProjection:
    s: float
    d: float
    yaw: float
    segment_index: int


@dataclass(frozen=True)
class TrackedObstacle:
    track_id: int
    center_x: float
    center_y: float
    length: float
    width: float
    velocity_x: float
    velocity_y: float
    speed: float
    motion_state: str
    yaw: float
    yaw_valid: bool
    yaw_source: str = "UNKNOWN"

    @classmethod
    def from_dict(cls, values: dict) -> "TrackedObstacle":
        velocity_x = float(values.get("velocity_x_map", 0.0))
        velocity_y = float(values.get("velocity_y_map", 0.0))
        return cls(
            track_id=int(values.get("id", -1)),
            center_x=float(values["center_x_map"]),
            center_y=float(values["center_y_map"]),
            length=max(0.0, float(values.get("length", 0.0))),
            width=max(0.0, float(values.get("width", 0.0))),
            velocity_x=velocity_x,
            velocity_y=velocity_y,
            speed=float(values.get("speed_mps", math.hypot(velocity_x, velocity_y))),
            motion_state=str(values.get("motion_state", "UNKNOWN")),
            yaw=float(values.get("yaw", 0.0)),
            yaw_valid=bool(values.get("yaw_valid", False)),
            yaw_source=str(values.get("yaw_source", "UNKNOWN")),
        )

    def predicted(self, seconds: float) -> Tuple[float, float]:
        # Confirmed static tracks may retain small filtered velocity noise.
        # UNKNOWN/MOVING/STOPPED keep their measured prediction.
        seconds = max(0.0, seconds)
        if self.motion_state.upper() == "STATIC" and math.hypot(self.velocity_x, self.velocity_y) <= 0.15:
            return self.center_x, self.center_y
        return (
            self.center_x + self.velocity_x * seconds,
            self.center_y + self.velocity_y * seconds,
        )


@dataclass
class FrenetCandidate:
    points: List[PathPoint]
    times: List[float]
    lateral_offsets: List[float]
    yaws: List[float]
    target_offset: float
    planning_time: float
    jerk_cost: float
    safety_cost: float
    offset_cost: float
    final_offset_cost: float
    time_cost: float
    side_cost: float
    total_cost: float
    minimum_clearance: float


class ReferencePath:
    """Polyline reference path with Cartesian/Frenet projection helpers."""

    def __init__(self, points: Sequence[PathPoint]) -> None:
        filtered: List[PathPoint] = []
        for point in points:
            if not filtered or math.hypot(
                point.x - filtered[-1].x, point.y - filtered[-1].y
            ) > 1.0e-6:
                filtered.append(point)
        if len(filtered) < 2:
            raise ValueError("reference path needs at least two distinct points")

        self.points = filtered
        self.cumulative_s = [0.0]
        self.segment_lengths: List[float] = []
        self.segment_yaws: List[float] = []
        for first, second in zip(filtered[:-1], filtered[1:]):
            dx = second.x - first.x
            dy = second.y - first.y
            length = math.hypot(dx, dy)
            self.segment_lengths.append(length)
            self.segment_yaws.append(math.atan2(dy, dx))
            self.cumulative_s.append(self.cumulative_s[-1] + length)
        self.length = self.cumulative_s[-1]
        self._xy = np.asarray([(p.x, p.y) for p in self.points[:-1]], dtype=float)
        self._lengths = np.asarray(self.segment_lengths)
        self._yaws = np.asarray(self.segment_yaws)
        self._tangents = np.column_stack((np.cos(self._yaws), np.sin(self._yaws)))
        self._starts = np.asarray(self.cumulative_s[:-1])

    def project(self, x: float, y: float, s_hint: Optional[float] = None, window: float = 12.0) -> FrenetProjection:
        ss, dd, indices = self.project_many([(x, y)], s_hint, window)
        index = int(indices[0])
        return FrenetProjection(float(ss[0]), float(dd[0]), self.segment_yaws[index], index)

    def project_many(self, xy, s_hint=None, window=12.0):
        start, end = 0, len(self.segment_lengths)
        if s_hint is not None:
            start = max(0, bisect.bisect_right(self.cumulative_s, s_hint-window)-1)
            end = min(end, bisect.bisect_right(self.cumulative_s, s_hint+window))
        relative = np.asarray(xy)[:, None, :] - self._xy[None, start:end, :]
        tangent = self._tangents[start:end]
        along = np.sum(relative * tangent[None, :, :], axis=2)
        along = np.clip(along, 0.0, self._lengths[None, start:end])
        error = relative - along[:, :, None]*tangent[None, :, :]
        chosen = np.argmin(np.sum(error*error, axis=2), axis=1)
        rows = np.arange(len(chosen))
        indices = chosen + start
        delta = error[rows, chosen]
        d = -self._tangents[indices, 1]*delta[:, 0] + self._tangents[indices, 0]*delta[:, 1]
        return self._starts[indices]+along[rows, chosen], d, indices

    def sample(self, s_value: float, lateral_offset: float = 0.0) -> PathPoint:
        s_value = _clamp(s_value, 0.0, self.length)
        index = bisect.bisect_right(self.cumulative_s, s_value) - 1
        index = max(0, min(index, len(self.segment_lengths) - 1))
        segment_s = s_value - self.cumulative_s[index]
        length = self.segment_lengths[index]
        ratio = _clamp(segment_s / length, 0.0, 1.0)
        first = self.points[index]
        second = self.points[index + 1]
        reference_x = first.x + ratio * (second.x - first.x)
        reference_y = first.y + ratio * (second.y - first.y)
        reference_z = first.z + ratio * (second.z - first.z)
        yaw = self.segment_yaws[index]
        return PathPoint(
            reference_x - math.sin(yaw) * lateral_offset,
            reference_y + math.cos(yaw) * lateral_offset,
            reference_z,
        )

    def yaw_at(self, s_value: float) -> float:
        """Return the reference segment heading at arc length ``s_value``."""
        s_value = _clamp(s_value, 0.0, self.length)
        index = bisect.bisect_right(self.cumulative_s, s_value) - 1
        index = max(0, min(index, len(self.segment_yaws) - 1))
        return self.segment_yaws[index]


def _quintic_coefficients(
    start_position: float,
    start_velocity: float,
    start_acceleration: float,
    end_position: float,
    end_velocity: float,
    end_acceleration: float,
    duration: float,
) -> Tuple[float, float, float, float, float, float]:
    if duration <= 0.0:
        raise ValueError("duration must be positive")
    t = duration
    c0 = start_position
    c1 = start_velocity
    c2 = 0.5 * start_acceleration
    c3 = (
        20.0 * (end_position - start_position)
        - (8.0 * end_velocity + 12.0 * start_velocity) * t
        - (3.0 * start_acceleration - end_acceleration) * t * t
    ) / (2.0 * t ** 3)
    c4 = (
        30.0 * (start_position - end_position)
        + (14.0 * end_velocity + 16.0 * start_velocity) * t
        + (3.0 * start_acceleration - 2.0 * end_acceleration) * t * t
    ) / (2.0 * t ** 4)
    c5 = (
        12.0 * (end_position - start_position)
        - (6.0 * end_velocity + 6.0 * start_velocity) * t
        - (start_acceleration - end_acceleration) * t * t
    ) / (2.0 * t ** 5)
    return c0, c1, c2, c3, c4, c5


def _evaluate_quintic(
    coefficients: Sequence[float], time_value: float
) -> Tuple[float, float, float, float]:
    c0, c1, c2, c3, c4, c5 = coefficients
    t = time_value
    position = c0 + c1 * t + c2 * t ** 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5
    velocity = c1 + 2.0 * c2 * t + 3.0 * c3 * t ** 2 + 4.0 * c4 * t ** 3 + 5.0 * c5 * t ** 4
    acceleration = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t ** 2 + 20.0 * c5 * t ** 3
    jerk = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * t ** 2
    return position, velocity, acceleration, jerk


def _projection_radius(
    axis_x: float,
    axis_y: float,
    rectangle_yaw: float,
    half_length: float,
    half_width: float,
) -> float:
    forward_x = math.cos(rectangle_yaw)
    forward_y = math.sin(rectangle_yaw)
    left_x = -forward_y
    left_y = forward_x
    return (
        half_length * abs(axis_x * forward_x + axis_y * forward_y)
        + half_width * abs(axis_x * left_x + axis_y * left_y)
    )


def _rectangles_overlap(
    first_x: float,
    first_y: float,
    first_yaw: float,
    first_half_length: float,
    first_half_width: float,
    second_x: float,
    second_y: float,
    second_yaw: float,
    second_half_length: float,
    second_half_width: float,
) -> bool:
    delta_x = second_x - first_x
    delta_y = second_y - first_y
    axes = (
        (math.cos(first_yaw), math.sin(first_yaw)),
        (-math.sin(first_yaw), math.cos(first_yaw)),
        (math.cos(second_yaw), math.sin(second_yaw)),
        (-math.sin(second_yaw), math.cos(second_yaw)),
    )
    for axis_x, axis_y in axes:
        centre_distance = abs(delta_x * axis_x + delta_y * axis_y)
        first_radius = _projection_radius(
            axis_x,
            axis_y,
            first_yaw,
            first_half_length,
            first_half_width,
        )
        second_radius = _projection_radius(
            axis_x,
            axis_y,
            second_yaw,
            second_half_length,
            second_half_width,
        )
        if centre_distance > first_radius + second_radius:
            return False
    return True


def _rectangle_corners(x, y, yaw, half_length, half_width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + a*c - b*s, y + a*s + b*c)
            for a, b in ((half_length, half_width), (half_length, -half_width),
                         (-half_length, -half_width), (-half_length, half_width))]


def _point_segment_distance(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    denom = dx*dx + dy*dy
    t = 0.0 if denom <= 1e-12 else _clamp(((p[0]-a[0])*dx + (p[1]-a[1])*dy)/denom, 0.0, 1.0)
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def _rectangle_distance(first, second):
    """Exact polygon separation for two rectangles; zero for overlap."""
    if _rectangles_overlap(*first, *second):
        return 0.0
    a, b = _rectangle_corners(*first), _rectangle_corners(*second)
    return min(_point_segment_distance(p, poly[i], poly[(i+1) % 4])
               for vertices, poly in ((a, b), (b, a)) for p in vertices for i in range(4))


def _batch_overlap(first, second):
    """Exact SAT, first[N,5], second[N,M,5] (center,yaw,half sizes)."""
    ax,ay,ayaw,ahl,ahw = [first[:,i,None] for i in range(5)]
    bx,by,byaw,bhl,bhw = [second[:,:,i] for i in range(5)]
    ca,sa,cb,sb = np.cos(ayaw),np.sin(ayaw),np.cos(byaw),np.sin(byaw)
    dx,dy = bx-ax,by-ay
    cosine = np.abs(ca*cb+sa*sb)
    sine = np.abs(ca*sb-sa*cb)
    return ((np.abs(dx*ca+dy*sa) <= ahl+bhl*cosine+bhw*sine)
          & (np.abs(-dx*sa+dy*ca) <= ahw+bhl*sine+bhw*cosine)
          & (np.abs(dx*cb+dy*sb) <= bhl+ahl*cosine+ahw*sine)
          & (np.abs(-dx*sb+dy*cb) <= bhw+ahl*sine+ahw*cosine))


def _batch_corners(boxes):
    x,y,yaw,hl,hw = [boxes[...,i,None] for i in range(5)]
    c,s = np.cos(yaw),np.sin(yaw)
    u=np.array([1.,1.,-1.,-1.]);v=np.array([1.,-1.,-1.,1.])
    return np.stack((x+hl*u*c-hw*v*s,y+hl*u*s+hw*v*c),axis=-1)


def _batch_point_box_distances(vertices, boxes):
    delta=vertices-boxes[...,None,:2]
    yaw=boxes[...,2,None]
    c,s=np.cos(yaw),np.sin(yaw)
    u=delta[...,0]*c+delta[...,1]*s
    v=-delta[...,0]*s+delta[...,1]*c
    du=np.maximum(np.abs(u)-boxes[...,3,None],0.)
    dv=np.maximum(np.abs(v)-boxes[...,4,None],0.)
    return np.sqrt(np.min(du*du+dv*dv,axis=-1))


def _batch_distances(first, second, assume_disjoint=False):
    a=_batch_corners(first[:,None,:]);b=_batch_corners(second)
    distance=np.minimum(_batch_point_box_distances(a,second),
                        _batch_point_box_distances(b,first[:,None,:]))
    if assume_disjoint:
        return distance
    return np.where(_batch_overlap(first,second),0.,distance)


class FrenetPlanner:
    def __init__(
        self,
        reference: ReferencePath,
        vehicle_length: float = 4.635,
        vehicle_width: float = 1.892,
        rear_axle_to_center: float = 1.35,
        safety_margin: float = 0.45,
        minimum_obstacle_length: float = 0.50,
        minimum_obstacle_width: float = 0.35,
        sample_time: float = 0.10,
        safety_sigma: float = 2.0,
        maximum_lateral_acceleration: float = 3.0,
        maximum_curvature: float = 0.28,
        preferred_side: str = "none",
        maximum_prediction_time: float = 4.0,
        route_trigger_margin: float = 0.0,
        assume_static_obstacles: bool = False,
    ) -> None:
        self.reference = reference
        if maximum_prediction_time <= 0.0:
            raise ValueError("invalid prediction horizon")
        self.maximum_prediction_time = maximum_prediction_time
        if not math.isfinite(route_trigger_margin) or route_trigger_margin < 0.0:
            raise ValueError("route trigger margin must be finite and non-negative")
        self.route_trigger_margin = route_trigger_margin
        self.assume_static_obstacles = bool(assume_static_obstacles)
        self.last_blocker_diagnostics = []
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.rear_axle_to_center = rear_axle_to_center
        self.safety_margin = safety_margin
        self.minimum_obstacle_length = minimum_obstacle_length
        self.minimum_obstacle_width = minimum_obstacle_width
        self.sample_time = sample_time
        self.safety_sigma = safety_sigma
        self.maximum_lateral_acceleration = maximum_lateral_acceleration
        self.maximum_curvature = maximum_curvature
        self.preferred_side = preferred_side.lower()

        self.weight_jerk = 30.0
        self.weight_safety = 4.00
        self.weight_clearance = 0.0
        self.preferred_clearance = 1.0
        self.weight_completion = 0.0
        # Strongly prioritize small normalized squared offsets. Hard OBB
        # collision rejection remains separate from the cost calculation.
        self.weight_offset = 100.0
        self.weight_final_offset = 0.20
        self.weight_time = 0.05
        self.side_preference_cost = 0.15

    def obstacle_box(self, obstacle, prediction_time):
        if self.assume_static_obstacles:
            # This MORAI test scene is known to contain fixed obstacles, but
            # the tracker reports noisy nonzero velocities for them. Keep the
            # measured obstacle position fixed through the planning horizon.
            x, y = obstacle.center_x, obstacle.center_y
        else:
            x, y = obstacle.predicted(prediction_time)
        half_length = 0.5 * max(obstacle.length, self.minimum_obstacle_length)
        half_width = 0.5 * max(obstacle.width, self.minimum_obstacle_width)
        # VELOCITY yaw is a motion heading, not the detected object's box
        # orientation. HOLD does not identify a reliable geometry orientation
        # either. Use an enclosing square for these uncertain orientations.
        yaw_source = obstacle.yaw_source.upper()
        if (obstacle.yaw_valid and math.isfinite(obstacle.yaw)
                and yaw_source not in ("VELOCITY", "HOLD", "UNKNOWN", "")):
            yaw = obstacle.yaw
        else:
            # Unknown geometry orientation: conservative enclosing square.
            # Do not invent a road-aligned orientation for a crossing object.
            yaw = 0.0
            half_length = half_width = math.hypot(half_length, half_width)
        return x, y, yaw, half_length, half_width

    def obstacle_intervals(self, obstacle, prediction_time):
        box = self.obstacle_box(obstacle, prediction_time)
        center = self.reference.project(box[0], box[1])
        window = max(12.0, 2.0 * math.hypot(box[3], box[4]))
        projections = [self.reference.project(x, y, center.s, window)
                       for x, y in _rectangle_corners(*box)]
        return (min(p.s for p in projections), max(p.s for p in projections),
                min(p.d for p in projections), max(p.d for p in projections))

    def obstacle_lateral_interval(self, obstacle, prediction_time):
        return self.obstacle_intervals(obstacle, prediction_time)[2:]

    @staticmethod
    def nearest_lateral_edge(d_min: float, d_max: float) -> float:
        """Signed BBox edge nearest d=0; return zero when it straddles it."""
        if d_min <= 0.0 <= d_max:
            return 0.0
        return d_min if d_min > 0.0 else d_max

    def clearance_target_offsets(
        self,
        obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
        configured_offsets: Sequence[float],
    ) -> List[float]:
        """Return finite candidate offsets without collapsing staggered obstacles.

        Obstacles at different longitudinal positions must not be combined into
        one lateral interval: that can force a pass around the first obstacle
        and directly into a later obstacle on the other side.  The planner's
        time-indexed OBB test checks every candidate against every detected
        obstacle.  Include d=0 when configured so a safe path through a gate
        is considered before moving laterally.
        """
        del obstacles, obstacle_age
        return [float(offset) for offset in configured_offsets
                if math.isfinite(float(offset))]

    def relevant_obstacles(
        self,
        obstacles: Sequence[TrackedObstacle],
        ego_s: float,
        ego_speed: float,
        vehicle_width: float,
        trigger_distance: float,
        rear_distance: float,
        obstacle_age: float,
    ) -> List[TrackedObstacle]:
        """Return obstacles whose inflated footprint intersects the map route.

        A lateral Frenet threshold treats any nearby object as a blocker even
        when the actual oriented vehicle and obstacle boxes do not meet. Sweep
        the ego footprint along the reference route instead. Off-route objects
        still remain in the full candidate collision checks.
        """
        relevant = []
        self.last_blocker_diagnostics = []
        prediction_speed = max(ego_speed, 1.0)
        horizon_distance = min(
            max(0.0, trigger_distance),
            prediction_speed * self.maximum_prediction_time,
        )
        step_count = max(1, int(math.ceil(horizon_distance / 0.20)))
        for obstacle in obstacles:
            route_overlap = None
            for k in range(step_count + 1):
                distance = horizon_distance * k / step_count
                route_s = min(self.reference.length, ego_s + distance)
                time_ahead = distance / prediction_speed
                route_point = self.reference.sample(route_s)
                route_yaw = self.reference.yaw_at(route_s)
                vehicle_center_x = route_point.x + self.rear_axle_to_center * math.cos(route_yaw)
                vehicle_center_y = route_point.y + self.rear_axle_to_center * math.sin(route_yaw)
                obstacle_x, obstacle_y, obstacle_yaw, half_length, half_width = (
                    self.obstacle_box(obstacle, obstacle_age + time_ahead)
                )
                if _rectangles_overlap(
                    vehicle_center_x,
                    vehicle_center_y,
                    route_yaw,
                    0.5 * self.vehicle_length + self.route_trigger_margin,
                    0.5 * vehicle_width + self.route_trigger_margin,
                    obstacle_x,
                    obstacle_y,
                    obstacle_yaw,
                    half_length,
                    half_width,
                ):
                    route_overlap = {"s_m": route_s, "time_s": time_ahead}
                    break
            if route_overlap is not None:
                relevant.append(obstacle)
                self.last_blocker_diagnostics.append({
                    "id": obstacle.track_id, "motion_state": obstacle.motion_state,
                    "vx": obstacle.velocity_x, "vy": obstacle.velocity_y,
                    "yaw_valid": obstacle.yaw_valid, "route_overlap": route_overlap,
                })
        return relevant

    def validate_remaining(self, candidate, vehicle, obstacles, obstacle_age,
                           max_tracking_error, target_speed=0.0):
        """Validate current footprint and the remaining fixed path, not its past.

        A tracking-error limit applies to joining one selected path, not to
        distance from the global route. It does not restrict lane changes.
        """
        x, y, yaw, speed = vehicle
        points = candidate.points
        if len(points) < 2:
            return False, {"reason": "path_too_short"}
        best = None
        for i in range(len(points)-1):
            a, b = points[i], points[i+1]
            dx, dy = b.x-a.x, b.y-a.y
            length2 = dx*dx+dy*dy
            alpha = max(0., min(1., ((x-a.x)*dx+(y-a.y)*dy)/max(length2, 1e-12)))
            px, py = a.x+alpha*dx, a.y+alpha*dy
            distance2 = (x-px)**2+(y-py)**2
            if best is None or distance2 < best[0]:
                best = (distance2, i, alpha, px, py)
        error2, index, alpha, px, py = best
        if error2 > max_tracking_error**2:
            return False, {"reason": "path_tracking_error", "error_m": math.sqrt(error2),
                           "limit_m": max_tracking_error}
        remaining = math.hypot(points[index+1].x-px, points[index+1].y-py)
        remaining += sum(math.hypot(b.x-a.x,b.y-a.y)
                         for a,b in zip(points[index+1:],points[index+2:]))
        if remaining < max(1.0, speed*0.5):
            return False, {"reason": "path_horizon_exhausted", "remaining_m": remaining}
        if not self._kinematically_valid(points[index:],candidate.yaws[index:],max(speed,1.)):
            return False, dict(self._rejection, reason="remaining_kinematic_limit")
        prediction_speed = max(speed,target_speed,1.0)
        samples = [(x, y, yaw, 0.)]
        # Sweep the short connection to the closest point, including actual yaw.
        connect_yaw = candidate.yaws[index]
        join_distance = math.sqrt(error2)
        count = max(1, int(math.ceil(join_distance/0.20)))
        for k in range(1,count+1):
            r = k/count
            samples.append((x+r*(px-x),y+r*(py-y),
                            yaw+r*_wrap_angle(connect_yaw-yaw),join_distance*r/prediction_speed))
        distance = join_distance
        prev_x,prev_y = px,py
        for point, point_yaw in zip(points[index+1:],candidate.yaws[index+1:]):
            distance += math.hypot(point.x-prev_x,point.y-prev_y)
            samples.append((point.x,point.y,point_yaw,distance/prediction_speed))
            prev_x,prev_y = point.x,point.y
        for sx,sy,syaw,t in samples:
            cx=sx+self.rear_axle_to_center*math.cos(syaw)
            cy=sy+self.rear_axle_to_center*math.sin(syaw)
            for obstacle in obstacles:
                ox,oy,oyaw,hl,hw=self.obstacle_box(obstacle,obstacle_age+t)
                if _rectangles_overlap(cx,cy,syaw,self.vehicle_length/2+self.safety_margin,
                                       self.vehicle_width/2+self.safety_margin,ox,oy,oyaw,hl,hw):
                    return False, {"reason":"remaining_collision", "track_id":obstacle.track_id,
                                   "t":t, "at_current_pose":t==0.}
        return True, {"reason":"remaining_path_valid", "remaining_m":remaining,
                      "tracking_error_m":math.sqrt(error2), "index":index}

    def plan(
        self,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_yaw: float,
        vehicle_speed: float,
        target_speed: float,
        obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
        target_offsets: Sequence[float],
        planning_times: Sequence[float],
        initial_lateral_acceleration: float = 0.0,
        completion_budget: Optional[float] = None,
        continuation_direction: int = 0,
    ) -> Optional[FrenetCandidate]:
        self.last_diagnostics = {"attempted": 0, "valid": 0, "reasons": {}}
        self.last_rejections = []
        projection = self.reference.project(vehicle_x, vehicle_y)
        self.last_diagnostics.update({"ego_s": projection.s, "ego_d": projection.d,
            "obstacles": len(obstacles), "vehicle_speed": vehicle_speed,
            "target_speed": target_speed})
        # Log geometry even on successful calls, for guardrail/tunnel comparison.
        self.last_diagnostics["input_boxes"] = [
            {"id": o.track_id, "raw": [o.center_x, o.center_y, o.length, o.width, o.yaw if math.isfinite(o.yaw) else None],
             "yaw_valid": o.yaw_valid, "yaw_source": o.yaw_source,
             "checked": list(self.obstacle_box(o, obstacle_age))}
            for o in obstacles]
        heading_error = _wrap_angle(vehicle_yaw - projection.yaw)
        start_lateral_speed = vehicle_speed * math.sin(heading_error)
        planning_speed = max(1.0, vehicle_speed, target_speed)
        remaining_path = self.reference.length - projection.s
        if remaining_path < max(5.0, planning_speed):
            self.last_diagnostics["early_exit"] = "remaining_path_short"
            return None

        unique_offsets = []
        for value in target_offsets:
            value = float(value)
            if math.isfinite(value) and not any(abs(value - existing) < 1.0e-6 for existing in unique_offsets):
                unique_offsets.append(value)

        if not unique_offsets or not planning_times:
            self.last_diagnostics["early_exit"] = "empty_targets_or_times"
            return None
        candidates: List[FrenetCandidate] = []
        maximum_time = max(float(value) for value in planning_times)
        maximum_offset = max(1.0, max(abs(value) for value in unique_offsets))
        # Evaluate every configured offset and choose the lowest weighted cost.
        # Collision and kinematic failures are already rejected before scoring.
        for target_offset in unique_offsets:
            for duration_value in planning_times:
                duration = float(duration_value)
                if not math.isfinite(duration) or duration <= 0.5:
                    self.last_diagnostics["invalid_duration"] = True
                    continue
                coefficients = _quintic_coefficients(
                    projection.d, start_lateral_speed, initial_lateral_acceleration,
                    target_offset, 0.0, 0.0, duration)
                self.last_diagnostics["attempted"] += 1
                self._rejection = {"reason": "unknown"}
                candidate = self._build_candidate(
                    projection.s, planning_speed, coefficients, target_offset,
                    duration, maximum_time, maximum_offset, obstacles, obstacle_age)
                if candidate is not None:
                    if (completion_budget is not None and math.isfinite(completion_budget)
                            and continuation_direction != 0
                            and (target_offset-projection.d)*continuation_direction >= -0.15):
                        overrun = max(0.0, duration-max(0.0,completion_budget))
                        candidate.total_cost += self.weight_completion * overrun ** 2
                    self.last_diagnostics["valid"] += 1
                    candidates.append(candidate)
                else:
                    reason = self._rejection["reason"]
                    counts = self.last_diagnostics["reasons"]
                    counts[reason] = counts.get(reason, 0) + 1
                    self.last_rejections.append(dict(self._rejection, d=target_offset, T=duration))
        if not candidates:
            return None
        return min(candidates, key=lambda candidate: candidate.total_cost)

    def _build_candidate(
        self,
        start_s: float,
        planning_speed: float,
        coefficients: Sequence[float],
        target_offset: float,
        duration: float,
        maximum_time: float,
        maximum_offset: float,
        obstacles: Sequence[TrackedObstacle],
        obstacle_age: float,
    ) -> Optional[FrenetCandidate]:
        # Continue the candidate at its selected offset after the lateral
        # maneuver and validate that held path through the whole planning
        # horizon. Without this extension, a short 2 s maneuver can be chosen
        # even when it enters a second obstacle at 3-4 s.
        remaining_distance = max(0.0, self.reference.length - start_s - 1.0e-3)
        validation_horizon = min(max(duration, maximum_time), remaining_distance / planning_speed)
        if validation_horizon < duration:
            self._rejection = {"reason": "horizon_short"}
            return None
        step_count = max(2, int(math.ceil(validation_horizon / self.sample_time)),
                         int(math.ceil(validation_horizon * planning_speed / 0.20)))
        times = [validation_horizon * index / step_count for index in range(step_count + 1)]
        points: List[PathPoint] = []
        offsets: List[float] = []
        jerks: List[float] = []
        for time_value in times:
            transition_time = min(time_value, duration)
            offset, _offset_speed, _offset_acceleration, offset_jerk = _evaluate_quintic(
                coefficients, transition_time
            )
            s_value = min(self.reference.length, start_s + planning_speed * time_value)
            point = self.reference.sample(s_value, offset)
            if points and math.hypot(point.x - points[-1].x, point.y - points[-1].y) < 1.0e-5:
                self._rejection = {"reason": "duplicate_point"}
                return None
            points.append(point)
            offsets.append(offset)
            jerks.append(offset_jerk)

        yaws = self._candidate_yaws(points)
        if not self._kinematically_valid(points, yaws, planning_speed):
            return None

        clearance_values = []
        if obstacles:
            physical_boxes=np.array([
                [point.x+self.rear_axle_to_center*math.cos(yaw),
                 point.y+self.rear_axle_to_center*math.sin(yaw),yaw,
                 self.vehicle_length/2.,self.vehicle_width/2.]
                for point,yaw in zip(points,yaws)],dtype=float)
            if self.assume_static_obstacles:
                measured=np.array([self.obstacle_box(o,obstacle_age) for o in obstacles],dtype=float)
                obstacle_boxes=np.broadcast_to(measured,(len(points),len(obstacles),5))
            else:
                obstacle_boxes=np.array([[self.obstacle_box(o,obstacle_age+t)
                    for o in obstacles] for t in times],dtype=float)
            inflated=physical_boxes.copy()
            inflated[:,3:] += self.safety_margin
            overlaps=_batch_overlap(inflated,obstacle_boxes)
            hits=np.argwhere(overlaps)
            if len(hits):
                row,column=map(int,hits[0])
                obstacle=obstacles[column];box=obstacle_boxes[row,column]
                self._rejection={"reason":"collision","track_id":obstacle.track_id,
                    "t":times[row],"at_start":times[row]==0.,
                    "ego_box":[*physical_boxes[row,:3],self.vehicle_length,self.vehicle_width],
                    "raw_obstacle_box":[obstacle.center_x,obstacle.center_y,
                        obstacle.yaw if math.isfinite(obstacle.yaw) else None,
                        obstacle.length,obstacle.width],
                    "yaw_source":obstacle.yaw_source,"yaw_valid":obstacle.yaw_valid,
                    "obstacle_box":[*box[:3],2.*box[3],2.*box[4]],"margin":self.safety_margin}
                return None
            # Exact same polygon separation used by the scalar baseline.
            clearance_values=_batch_distances(physical_boxes,obstacle_boxes,assume_disjoint=True).ravel().tolist()

        speed_normalizer = max(planning_speed, 1.0)
        jerk_cost = sum((value / speed_normalizer) ** 2 for value in jerks) / len(jerks)
        if clearance_values:
            safety_cost = sum(
                math.exp(-value / max(self.safety_sigma, 1.0e-3))
                for value in clearance_values
            ) / len(clearance_values)
            minimum_clearance = min(clearance_values)
        else:
            safety_cost = 0.0
            minimum_clearance = float("inf")
        offset_cost = sum((value / maximum_offset) ** 2 for value in offsets) / len(offsets)
        final_offset_cost = (offsets[-1] / maximum_offset) ** 2
        time_cost = duration / maximum_time
        side_cost = 0.0
        if self.preferred_side == "left" and target_offset < -1.0e-3:
            side_cost = self.side_preference_cost
        elif self.preferred_side == "right" and target_offset > 1.0e-3:
            side_cost = self.side_preference_cost
        total_cost = (
            self.weight_jerk * jerk_cost
            + self.weight_safety * safety_cost
            + self.weight_offset * offset_cost
            + self.weight_final_offset * final_offset_cost
            + self.weight_time * time_cost
            + side_cost
        )
        deficit = max(0.0, (self.preferred_clearance-minimum_clearance)
                      / max(self.preferred_clearance,1.e-6))
        total_cost += self.weight_clearance * deficit ** 2
        return FrenetCandidate(
            points=points,
            times=times,
            lateral_offsets=offsets,
            yaws=yaws,
            target_offset=target_offset,
            planning_time=duration,
            jerk_cost=jerk_cost,
            safety_cost=safety_cost,
            offset_cost=offset_cost,
            final_offset_cost=final_offset_cost,
            time_cost=time_cost,
            side_cost=side_cost,
            total_cost=total_cost,
            minimum_clearance=minimum_clearance,
        )

    @staticmethod
    def _candidate_yaws(points: Sequence[PathPoint]) -> List[float]:
        yaws = []
        for index, point in enumerate(points):
            if index < len(points) - 1:
                other = points[index + 1]
                yaws.append(math.atan2(other.y - point.y, other.x - point.x))
            else:
                other = points[index - 1]
                yaws.append(math.atan2(point.y - other.y, point.x - other.x))
        return yaws

    def _kinematically_valid(
        self,
        points: Sequence[PathPoint],
        yaws: Sequence[float],
        speed: float,
    ) -> bool:
        for index in range(1, len(points)):
            distance = math.hypot(
                points[index].x - points[index - 1].x,
                points[index].y - points[index - 1].y,
            )
            if distance < 1.0e-5:
                self._rejection = {"reason": "duplicate_point"}
                return False
            curvature = abs(_wrap_angle(yaws[index] - yaws[index - 1])) / distance
            if curvature > self.maximum_curvature:
                self._rejection = {"reason": "curvature", "value": curvature, "limit": self.maximum_curvature, "index": index}
                return False
            if speed * speed * curvature > self.maximum_lateral_acceleration:
                self._rejection = {"reason": "lateral_acceleration", "value": speed*speed*curvature, "limit": self.maximum_lateral_acceleration, "index": index}
                return False
        return True
