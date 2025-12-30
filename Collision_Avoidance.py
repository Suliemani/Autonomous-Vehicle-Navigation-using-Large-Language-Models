import math
import carla


def check_collision_ahead(world, ego_vehicle, max_distance=10.0, angle_threshold=50.0):
    # Get ego vehicle's current position and orientation
    ego_tf = ego_vehicle.get_transform()
    ego_loc = ego_tf.location
    ego_fwd = ego_tf.get_forward_vector()
    ego_wp = world.get_map().get_waypoint(ego_loc)

    # Get all nearby vehicles and pedestrians
    vehicles = [v for v in world.get_actors().filter("*vehicle*") if v.id != ego_vehicle.id]
    walkers = list(world.get_actors().filter("*walker.pedestrian*"))
    obstacles = vehicles + walkers

    for actor in obstacles:
        if not actor.is_alive:
            continue

        # Calculate distance to actor
        actor_loc = actor.get_location()
        dist = actor_loc.distance(ego_loc)

        # Account for actor's size using bounding box
        bounding_box = getattr(actor, "bounding_box", None)
        extra_dist = 0.0
        if bounding_box:
            max_extent = max(bounding_box.extent.x, bounding_box.extent.y)
            if max_extent > 1.5:
                extra_dist = max_extent

        # Skip if too far away
        effective_dist = max_distance + extra_dist
        if dist > effective_dist:
            continue

        # Check if actor is in same lane
        actor_wp = world.get_map().get_waypoint(actor_loc)
        if not actor_wp:
            continue
        if actor_wp.road_id != ego_wp.road_id or actor_wp.lane_id != ego_wp.lane_id:
            continue

        # Check if actor is in front of ego vehicle
        to_actor = actor_loc - ego_loc
        forward_dot = (
            ego_fwd.x * to_actor.x + ego_fwd.y * to_actor.y + ego_fwd.z * to_actor.z
        )
        if forward_dot <= 0.0:
            continue

        # Calculate angle between ego's forward vector and actor
        mag_ego = math.sqrt(ego_fwd.x**2 + ego_fwd.y**2 + ego_fwd.z**2)
        cos_ang = forward_dot / (mag_ego * dist + 1e-9)
        cos_ang = max(-1.0, min(1.0, cos_ang))
        angle_deg = math.degrees(math.acos(cos_ang))

        # Return True if within angle threshold (directly ahead)
        if angle_deg < angle_threshold:
            return True

    return False


def check_side_collision(world, ego_vehicle, side_threshold=3.0):
    # Get ego vehicle's position and current lane info
    ego_tf = ego_vehicle.get_transform()
    ego_loc = ego_tf.location
    ego_wp = world.get_map().get_waypoint(ego_loc, lane_type=carla.LaneType.Driving)
    if not ego_wp:
        return False

    # Get all nearby vehicles
    ego_right = ego_tf.get_right_vector()
    vehicles = [v for v in world.get_actors().filter("*vehicle*") if v.id != ego_vehicle.id]

    for v in vehicles:
        if not v.is_alive:
            continue

        # Skip if too far away
        v_loc = v.get_location()
        dist = v_loc.distance(ego_loc)
        if dist > (side_threshold + 5.0):
            continue

        # Check lane alignment with some tolerance for merging
        v_wp = world.get_map().get_waypoint(v_loc, lane_type=carla.LaneType.Driving)
        if not v_wp:
            continue

        # Skip if on different road or more than 1 lane away
        if v_wp.road_id != ego_wp.road_id:
            continue
        lane_diff = abs(v_wp.lane_id - ego_wp.lane_id)
        if lane_diff > 1:
            continue

        # Calculate lateral and forward distance
        offset = v_loc - ego_loc
        lat_dist = offset.x * ego_right.x + offset.y * ego_right.y + offset.z * ego_right.z
        fwd_dist = (
            offset.x * ego_tf.get_forward_vector().x
            + offset.y * ego_tf.get_forward_vector().y
            + offset.z * ego_tf.get_forward_vector().z
        )

        # Check if within collision thresholds
        if abs(lat_dist) < side_threshold and abs(fwd_dist) < 5.0:
            return True

    return False
