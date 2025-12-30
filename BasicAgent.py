# Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla
from shapely.geometry import Polygon
from agents.navigation.local_planner import LocalPlanner, RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.tools.misc import (
    get_speed, is_within_distance, get_trafficlight_trigger_location,
    compute_distance
)


class BasicAgent(object):
    """
    BasicAgent that navigates with local+global planners, stops for traffic lights,
    vehicles.
    """

    def __init__(self, vehicle, target_speed=20, opt_dict={}, map_inst=None, grp_inst=None):
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()

        if map_inst and isinstance(map_inst, carla.Map):
            self._map = map_inst
        else:
            self._map = self._world.get_map()

        self._last_traffic_light = None

        # Base flags
        self._ignore_traffic_lights = False
        self._ignore_stop_signs = False
        self._ignore_vehicles = False
        self._use_bbs_detection = False

        self._target_speed = target_speed
        self._sampling_resolution = 2.0

        # Distances + thresholds
        self._base_tlight_threshold = 2.0
        self._base_vehicle_threshold = 5.0
        self._speed_ratio = 0.3
        self._max_brake = 0.5
        self._offset = 0

        # read opt_dict
        opt_dict['target_speed'] = target_speed
        if 'ignore_traffic_lights' in opt_dict:
            self._ignore_traffic_lights = opt_dict['ignore_traffic_lights']
        if 'ignore_stop_signs' in opt_dict:
            self._ignore_stop_signs = opt_dict['ignore_stop_signs']
        if 'ignore_vehicles' in opt_dict:
            self._ignore_vehicles = opt_dict['ignore_vehicles']
        if 'use_bbs_detection' in opt_dict:
            self._use_bbs_detection = opt_dict['use_bbs_detection']
        if 'sampling_resolution' in opt_dict:
            self._sampling_resolution = opt_dict['sampling_resolution']

        if 'base_tlight_threshold' in opt_dict:
            self._base_tlight_threshold = opt_dict['base_tlight_threshold']
        if 'base_vehicle_threshold' in opt_dict:
            self._base_vehicle_threshold = opt_dict['base_vehicle_threshold']
        if 'detection_speed_ratio' in opt_dict:
            self._speed_ratio = opt_dict['detection_speed_ratio']
        if 'max_brake' in opt_dict:
            self._max_brake = opt_dict['max_brake']
        if 'offset' in opt_dict:
            self._offset = opt_dict['offset']

        # local + global planners
        self._local_planner = LocalPlanner(self._vehicle, opt_dict=opt_dict, map_inst=self._map)
        if grp_inst and isinstance(grp_inst, GlobalRoutePlanner):
            self._global_planner = grp_inst
        else:
            self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)

        # traffic lights
        self._lights_list = self._world.get_actors().filter("*traffic_light*")
        self._lights_map = {}

        # stop signs
        self._stop_signs_list = self._world.get_actors().filter("*traffic.stop*")
        self._last_stop_sign = None

        # 2-second logic
        self._stop_sign_stop_time = None

        # track which signs served
        self._ignored_stop_sign_ids = set()

        self.forced_stops_count = 0
        self.traffic_light_stops_count = 0
        self.stop_sign_stops_count = 0

    def add_emergency_stop(self, control):
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        self.forced_stops_count += 1
        return control

    def set_target_speed(self, speed):
        self._target_speed = speed
        self._local_planner.set_speed(speed)

    def follow_speed_limits(self, value=True):
        self._local_planner.follow_speed_limits(value)

    def get_local_planner(self):
        return self._local_planner

    def get_global_planner(self):
        return self._global_planner

    def set_destination(self, end_location, start_location=None):
        if not start_location:
            start_location = self._local_planner.target_waypoint.transform.location
            clean_queue = True
        else:
            start_location = self._vehicle.get_location()
            clean_queue = False

        start_wp = self._map.get_waypoint(start_location)
        end_wp = self._map.get_waypoint(end_location)
        route_trace = self.trace_route(start_wp, end_wp)
        self._local_planner.set_global_plan(route_trace, clean_queue=clean_queue)

    def set_global_plan(self, plan, stop_waypoint_creation=True, clean_queue=True):
        self._local_planner.set_global_plan(
            plan,
            stop_waypoint_creation=stop_waypoint_creation,
            clean_queue=clean_queue
        )

    def trace_route(self, start_wp, end_wp):
        start_loc = start_wp.transform.location
        end_loc = end_wp.transform.location
        return self._global_planner.trace_route(start_loc, end_loc)

    def run_step(self):
        hazard_detected = False
        vehicle_list = self._world.get_actors().filter("*vehicle*")
        vehicle_speed = get_speed(self._vehicle) / 3.6

        max_vehicle_dist = self._base_vehicle_threshold + self._speed_ratio * vehicle_speed
        has_vehicle, _, _ = self._vehicle_obstacle_detected(vehicle_list, max_vehicle_dist)
        if has_vehicle:
            hazard_detected = True

        max_tl_dist = self._base_tlight_threshold + self._speed_ratio * vehicle_speed
        has_tl, _ = self._affected_by_traffic_light(self._lights_list, max_tl_dist)
        if has_tl:
            hazard_detected = True
            self.traffic_light_stops_count += 1

        if not self._ignore_stop_signs:
            max_stop_dist = 20.0 + self._speed_ratio * vehicle_speed
            has_stop, _ = self._affected_by_stop_sign(self._stop_signs_list, max_stop_dist)
            if has_stop:
                hazard_detected = True
                self.stop_sign_stops_count += 1

        if self._last_stop_sign and hazard_detected:
            now_sec = self._world.get_snapshot().timestamp.elapsed_seconds
            if (self._stop_sign_stop_time is not None and now_sec > self._stop_sign_stop_time):
                self._ignored_stop_sign_ids.add(self._last_stop_sign.id)
                self._last_stop_sign = None
                self._stop_sign_stop_time = None
                hazard_detected = False

        control = self._local_planner.run_step()
        if hazard_detected:
            control = self.add_emergency_stop(control)
        return control

    def done(self):
        return self._local_planner.done()

    def ignore_traffic_lights(self, active=True):
        self._ignore_traffic_lights = active

    def ignore_stop_signs(self, active=True):
        self._ignore_stop_signs = active

    def ignore_vehicles(self, active=True):
        self._ignore_vehicles = active

    def set_offset(self, offset):
        self._local_planner.set_offset(offset)

    def lane_change(self, direction, same_lane_time=0, other_lane_time=0, lane_change_time=2):
        speed = self._vehicle.get_velocity().length()
        path = self._generate_lane_change_path(
            self._map.get_waypoint(self._vehicle.get_location()),
            direction,
            same_lane_time * speed,
            other_lane_time * speed,
            lane_change_time * speed,
            False, 1,
            self._sampling_resolution
        )
        if not path:
            print("WARNING: no lane change path found => ignoring.")
        self.set_global_plan(path)

    def _affected_by_traffic_light(self, lights_list=None, max_distance=None):
        if self._ignore_traffic_lights:
            return (False, None)

        if not lights_list:
            lights_list = self._world.get_actors().filter("*traffic_light*")

        if not max_distance:
            max_distance = self._base_tlight_threshold

        if self._last_traffic_light:
            if self._last_traffic_light.state != carla.TrafficLightState.Red:
                self._last_traffic_light = None
            else:
                return (True, self._last_traffic_light)

        ego_loc = self._vehicle.get_location()
        ego_wp = self._map.get_waypoint(ego_loc)

        for tl in lights_list:
            if tl.id in self._lights_map:
                trig_wp = self._lights_map[tl.id]
            else:
                trig_loc = get_trafficlight_trigger_location(tl)
                trig_wp = self._map.get_waypoint(trig_loc)
                self._lights_map[tl.id] = trig_wp

            if trig_wp.transform.location.distance(ego_loc) > max_distance:
                continue
            if trig_wp.road_id != ego_wp.road_id:
                continue

            ve_dir = ego_wp.transform.get_forward_vector()
            wp_dir = trig_wp.transform.get_forward_vector()
            dot = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z
            if dot < 0:
                continue

            if tl.state != carla.TrafficLightState.Red:
                continue

            if is_within_distance(trig_wp.transform, self._vehicle.get_transform(), max_distance, [0, 90]):
                self._last_traffic_light = tl
                return (True, tl)

        return (False, None)

    def _affected_by_stop_sign(self, stop_signs_list=None, max_distance=5.0):
        """
        If new stop sign detected set hazard
        Once stopped, ignore ID
        """
        if self._ignore_stop_signs:
            return (False, None)

        if not stop_signs_list:
            stop_signs_list = self._stop_signs_list

        ego_loc = self._vehicle.get_location()
        ego_wp = self._map.get_waypoint(ego_loc)

        if self._last_stop_sign:
            dist_passed = ego_loc.distance(self._last_stop_sign.get_location())
            if dist_passed > max_distance:
                # Cleared stop sign
                self._last_stop_sign = None
                self._stop_sign_stop_time = None
            else:
                # still in range, keep hazard
                return (True, self._last_stop_sign)

        for sign in stop_signs_list:
            if not sign.is_alive:
                continue
            if sign.id in self._ignored_stop_sign_ids:
                continue

            sign_loc = sign.get_location()
            dist = sign_loc.distance(ego_loc)
            if dist > max_distance:
                continue

            sign_wp = self._map.get_waypoint(sign_loc)
            if sign_wp.road_id != ego_wp.road_id:
                continue

            ve_dir = ego_wp.transform.get_forward_vector()
            wp_dir = sign_wp.transform.get_forward_vector()
            dot = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z
            if dot < 0:
                continue

            self._last_stop_sign = sign
            now_sec = self._world.get_snapshot().timestamp.elapsed_seconds
            self._stop_sign_stop_time = now_sec + 2.0  # 2s stop
            return (True, sign)

        return (False, None)

    def _vehicle_obstacle_detected(self, vehicle_list=None, max_distance=None,
                                  up_angle_th=90, low_angle_th=0, lane_offset=0):
        """
        check if there's a vehicle in front
        """
        def get_route_polygon():
            route_bb = []
            extent_y = self._vehicle.bounding_box.extent.y
            r_ext = extent_y + self._offset
            l_ext = -extent_y + self._offset
            r_vec = ego_transform.get_right_vector()
            p1 = ego_location + carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
            p2 = ego_location + carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
            route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

            for wp, _ in self._local_planner.get_plan():
                if ego_location.distance(wp.transform.location) > max_distance:
                    break
                rv = wp.transform.get_right_vector()
                p1 = wp.transform.location + carla.Location(r_ext * rv.x, r_ext * rv.y)
                p2 = wp.transform.location + carla.Location(l_ext * rv.x, l_ext * rv.y)
                route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

            if len(route_bb) < 3:
                return None
            return Polygon(route_bb)

        if self._ignore_vehicles:
            return (False, None, -1)

        if not vehicle_list:
            vehicle_list = self._world.get_actors().filter("*vehicle*")

        if not max_distance:
            max_distance = self._base_vehicle_threshold

        ego_transform = self._vehicle.get_transform()
        ego_location = ego_transform.location
        ego_wpt = self._map.get_waypoint(ego_location)

        if ego_wpt.lane_id < 0 and lane_offset != 0:
            lane_offset *= -1

        ego_front_tf = ego_transform
        ego_front_tf.location += carla.Location(
            self._vehicle.bounding_box.extent.x * ego_transform.get_forward_vector()
        )

        opposite_invasion = abs(self._offset) + self._vehicle.bounding_box.extent.y > ego_wpt.lane_width / 2
        use_bbs = self._use_bbs_detection or opposite_invasion or ego_wpt.is_junction

        route_polygon = get_route_polygon()

        for targ_v in vehicle_list:
            if targ_v.id == self._vehicle.id:
                continue

            targ_tf = targ_v.get_transform()
            if targ_tf.location.distance(ego_location) > max_distance:
                continue

            targ_wp = self._map.get_waypoint(targ_tf.location, lane_type=carla.LaneType.Any)

            if (use_bbs or targ_wp.is_junction) and route_polygon:
                targ_bb = targ_v.bounding_box
                verts = targ_bb.get_world_vertices(targ_tf)
                vert_list = [[v.x, v.y, v.z] for v in verts]
                targ_polygon = Polygon(vert_list)
                if route_polygon.intersects(targ_polygon):
                    return (True, targ_v, compute_distance(targ_tf.location, ego_location))

            else:
                if targ_wp.road_id != ego_wpt.road_id or targ_wp.lane_id != (ego_wpt.lane_id + lane_offset):
                    next_w, _ = self._local_planner.get_incoming_waypoint_and_direction(steps=3)
                    if not next_w:
                        continue
                    if targ_wp.road_id != next_w.road_id or targ_wp.lane_id != (next_w.lane_id + lane_offset):
                        continue

                targ_fwd = targ_tf.get_forward_vector()
                targ_ext = targ_v.bounding_box.extent.x
                targ_rear_tf = targ_tf
                targ_rear_tf.location -= carla.Location(
                    x=targ_ext * targ_fwd.x,
                    y=targ_ext * targ_fwd.y
                )

                if is_within_distance(targ_rear_tf, ego_front_tf, max_distance, [low_angle_th, up_angle_th]):
                    return (True, targ_v, compute_distance(targ_tf.location, ego_transform.location))

        return (False, None, -1)

    def _generate_lane_change_path(self, waypoint, direction='left',
                                  distance_same_lane=10,
                                  distance_other_lane=25,
                                  lane_change_distance=25,
                                  check=True, lane_changes=1,
                                  step_distance=2):
        plan = []
        plan.append((waypoint, RoadOption.LANEFOLLOW))

        distance_same_lane = max(distance_same_lane, 0.1)
        distance_other_lane = max(distance_other_lane, 0.1)
        lane_change_distance = max(lane_change_distance, 0.1)

        # same lane
        dist_accum = 0
        while dist_accum < distance_same_lane:
            nxts = plan[-1][0].next(step_distance)
            if not nxts:
                return []
            nxt = nxts[0]
            dist_accum += nxt.transform.location.distance(plan[-1][0].transform.location)
            plan.append((nxt, RoadOption.LANEFOLLOW))

        if direction == 'left':
            option = RoadOption.CHANGELANELEFT
        elif direction == 'right':
            option = RoadOption.CHANGELANERIGHT
        else:
            return []

        # do lane_changes times
        single_lc_dist = lane_change_distance / lane_changes
        changes_done = 0
        while changes_done < lane_changes:
            nxts = plan[-1][0].next(single_lc_dist)
            if not nxts:
                return []
            nxt = nxts[0]

            if direction == 'left':
                if check and str(nxt.lane_change) not in ['Left', 'Both']:
                    return []
                side_wp = nxt.get_left_lane()
            else:
                if check and str(nxt.lane_change) not in ['Right', 'Both']:
                    return []
                side_wp = nxt.get_right_lane()

            if not side_wp or side_wp.lane_type != carla.LaneType.Driving:
                return []

            plan.append((side_wp, option))
            changes_done += 1

        dist_accum = 0
        while dist_accum < distance_other_lane:
            nxts = plan[-1][0].next(step_distance)
            if not nxts:
                return []
            nxt = nxts[0]
            dist_accum += nxt.transform.location.distance(plan[-1][0].transform.location)
            plan.append((nxt, RoadOption.LANEFOLLOW))

        return plan
