import carla
import math

# Dictionary containing special attributes/descriptions for each POI
poi_specialties = {
    "Hospital A": "Offers advanced pediatric care.",
    "Hospital B": "Renowned for quick emergency responses.",
    "Hospital C": "Has an excellent cardiology unit.",
    "Hospital D": "Specializes in outpatient surgeries.",
    "Hospital E": "Features an on-site urgent care clinic.",
    "School A": "Known for its business program.",
    "School B": "Focuses on STEM research and labs.",
    "School C": "Offers top-tier sports facilities.",
    "School D": "Known for arts and creative programs.",
    "School E": "Has a highly-rated computer science department.",
    "Cafe A": "Famous for homemade pastries.",
    "Cafe B": "Popular for fresh salads and sandwiches.",
    "Cafe C": "Organic menu and vegan options.",
    "Cafe D": "Brews the best lattes in town.",
    "Cafe E": "Known for specialty teas and desserts.",
    "Mall A": "Extensive sporting outlets for shoes and gear.",
    "Mall B": "Known for premium clothing boutiques.",
    "Mall C": "Huge electronics and tech shops.",
    "Mall D": "Offers luxury footwear and designer brands.",
    "Mall E": "Features a large food court with global cuisine."
}

# Dictionary containing all POIs categorized by type with their locations
poi_locations = {
    "hospitals": {
        "Hospital A": carla.Location(x=-20.0, y=130.0, z=1.0),
        "Hospital B": carla.Location(x=30.0, y=78.0, z=1.0),
        "Hospital C": carla.Location(x=-150.0, y=220.0, z=1.0),
        "Hospital D": carla.Location(x=100.0, y=-50.0, z=1.0),
        "Hospital E": carla.Location(x=-80.0, y=10.0, z=1.0),
    },
    "schools": {
        "School A": carla.Location(x=-10.0, y=200.0, z=1.0),
        "School B": carla.Location(x=-100.0, y=120.0, z=1.0),
        "School C": carla.Location(x=90.0, y=150.0, z=1.0),
        "School D": carla.Location(x=-50.0, y=-100.0, z=1.0),
        "School E": carla.Location(x=150.0, y=-30.0, z=1.0),
    },
    "cafes": {
        "Cafe A": carla.Location(x=10.0, y=10.0, z=1.0),
        "Cafe B": carla.Location(x=250.0, y=30.0, z=1.0),
        "Cafe C": carla.Location(x=-200.0, y=60.0, z=1.0),
        "Cafe D": carla.Location(x=-40.0, y=250.0, z=1.0),
        "Cafe E": carla.Location(x=100.0, y=200.0, z=1.0),
    },
    "malls": {
        "Mall A": carla.Location(x=200.0, y=200.0, z=1.0),
        "Mall B": carla.Location(x=-160.0, y=160.0, z=1.0),
        "Mall C": carla.Location(x=300.0, y=90.0, z=1.0),
        "Mall D": carla.Location(x=-100.0, y=-20.0, z=1.0),
        "Mall E": carla.Location(x=50.0, y=250.0, z=1.0),
    }
}

# Returns the carla.Location for a given POI name if it exists
def get_location_by_name(name):
    for cat, pois in poi_locations.items():
        if name in pois:
            return pois[name]
    return None

# Calculates the total distance of a route in meters
def compute_route_distance(global_route):
    if not global_route or len(global_route) < 2:
        return 0.0
    dist_sum = 0.0
    for i in range(len(global_route) - 1):
        (wp1, _) = global_route[i]
        (wp2, _) = global_route[i + 1]
        dist_sum += wp1.transform.location.distance(wp2.transform.location)
    return dist_sum

# Calculates distances to all POIs via legal routes from vehicle's current position
def compute_legal_distances(vehicle, grp):
    vehicle_loc = vehicle.get_location()
    data = {}
    for category, pois in poi_locations.items():
        data[category] = {}
        for pn, ploc in pois.items():
            route = grp.trace_route(vehicle_loc, ploc)
            dist = compute_route_distance(route)
            data[category][pn] = round(dist, 2)
    return data

# Extracts POI names mentioned in user's text
def parse_user_route_list(user_text):
    # First collect all POI names from all categories
    all_pois = []
    for cat, pois in poi_locations.items():
        for pn in pois.keys():
            all_pois.append(pn)

    # Find all POIs mentioned in the text
    text_lower = user_text.lower()
    occurrences = []
    for poi in all_pois:
        idx = text_lower.find(poi.lower())
        if idx != -1:
            occurrences.append((idx, poi))

    # Sort POIs by their position in the text
    occurrences.sort(key=lambda x: x[0])

    # Return unique POIs
    used = set()
    result = []
    for pos, pn in occurrences:
        if pn not in used:
            result.append(pn)
            used.add(pn)
    return result

#Finds the nearest POI within category
def get_nearest_poi_in_category(cat_name, skip_poi, vehicle, grp):
    # Get distances to all POIs in the category
    distance_data = compute_legal_distances(vehicle, grp)
    cat_distances = distance_data.get(cat_name, {})
    if not cat_distances:
        return None

    # Filter out the skip_poi if it exists
    filtered = {k: v for k, v in cat_distances.items() if k != skip_poi}
    if not filtered:
        return None

    # Return the POI with the smallest distance
    nearest_poi = min(filtered, key=filtered.get)
    return nearest_poi
