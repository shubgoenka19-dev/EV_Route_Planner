
import osmnx as ox
import networkx as nx
import math
from shapely.geometry import LineString
import logging
import random

ox.settings.use_cache = True
ox.settings.log_console = False
ox.settings.log_level = logging.WARNING

G_cache = None

# ---------- constants ----------
G_const = {
    "g": 9.80665,
    "rho": 1.225
}

# ---------- helpers ----------
def _get_edge_attr_single(u, v, d, attr_name):
    """
    Safe extractor for edge attributes for both DiGraph and MultiDiGraph.
    """
    if attr_name in d:
        return d.get(attr_name, 0.0)
    if isinstance(d, dict):
        for key, val in d.items():
            if isinstance(val, dict) and attr_name in val:
                return val.get(attr_name, 0.0)
    return 0.0

# ---------- energy model ----------
def compute_edge_energy(edge_data, vehicle, default_speed_kph=30.0, regen_eff=0.6):
    length_m = edge_data.get("length", 0.0)
    if length_m <= 0:
        return 0.0

    # speed
    speed_kph = default_speed_kph
    if 'maxspeed' in edge_data:
        try:
            if isinstance(edge_data['maxspeed'], (list, tuple)):
                speed_kph = float(edge_data['maxspeed'][0])
            else:
                speed_kph = float(str(edge_data['maxspeed']).split(";")[0])
        except Exception:
            speed_kph = default_speed_kph

    v = max(speed_kph, 1.0) / 3.6
    m = vehicle.get("mass_kg", 1500.0)
    C_rr = vehicle.get("C_rr", 0.01)
    Cd = vehicle.get("Cd", 0.28)
    A = vehicle.get("A", 2.2)

    F_rr = C_rr * m * G_const["g"]
    work_rr_J = F_rr * length_m
    work_aero_J = 0.5 * G_const["rho"] * Cd * A * (v ** 2) * length_m
    grade = edge_data.get("grade", 0.0)
    work_climb_J = m * G_const["g"] * grade * length_m
    total_J = work_rr_J + work_aero_J + work_climb_J

    if work_climb_J < 0:
        recovered = -work_climb_J * regen_eff
        total_J = work_rr_J + work_aero_J + work_climb_J + (-recovered)

    total_kwh = total_J / 3.6e6
    return max(total_kwh, 0.0)

# ---------- graph loading ----------
def load_graph(place_name=None, point=None, dist_m=3000, network_type="drive", force_reload=False):
    """
    Loads and caches graph. If force_reload True, bypasses cache.
    """
    global G_cache
    if G_cache is not None and not force_reload:
        return G_cache

    if place_name:
        # simple graph_from_place
        G = ox.graph_from_place(place_name, network_type=network_type, simplify=True)
    elif point:
        lat, lon = point
        G = ox.graph_from_point((lat, lon), dist=dist_m, network_type=network_type, simplify=True)
    else:
        raise ValueError("Provide place_name or point")

    # ensure length attribute present
    try:
        # prefer new API names
        G = ox.distance.add_edge_lengths(G)
    except Exception:
        # older osmnx versions may differ; attempt to ensure length exists
        for u, v, k, data in G.edges(keys=True, data=True):
            if "length" not in data:
                data["length"] = 0.0

    # add grade if available or default 0
    for u, v, key, data in G.edges(keys=True, data=True):
        if "grade" not in data:
            inc = data.get("incline", None)
            if inc is not None:
                try:
                    s = str(inc).strip().replace("%", "")
                    data["grade"] = float(s) / 100.0
                except Exception:
                    data["grade"] = 0.0
            else:
                data["grade"] = 0.0

    G_cache = G
    return G_cache

# ---------- add per-edge energy ----------
def add_energy_attributes(G, vehicle, default_speed_kph=30.0, regen_eff=0.6):
    """
    Adds energy_kwh and time_s for each edge in-place.
    """
    for u, v, key, data in G.edges(keys=True, data=True):
        energy_kwh = compute_edge_energy(data, vehicle, default_speed_kph=default_speed_kph, regen_eff=regen_eff)
        # set on each parallel-edge attr dict if MultiDiGraph
        if isinstance(data, dict) and ('energy_kwh' not in data) and (len(data) > 0 and isinstance(list(data.values())[0], dict)):
            # in some graphs, get_edge_data returns mapping key->attr, but here we already have `data` as attr dict
            # safe assign
            data['energy_kwh'] = energy_kwh
        else:
            data['energy_kwh'] = energy_kwh

        # compute time
        sp = default_speed_kph
        try:
            if 'maxspeed' in data:
                sp_raw = data['maxspeed']
                if isinstance(sp_raw, (list, tuple)):
                    sp = float(sp_raw[0])
                else:
                    sp = float(str(sp_raw).split(";")[0])
        except Exception:
            sp = default_speed_kph
        v_m_s = max(sp, 1.0) / 3.6
        data['time_s'] = data.get('length', 0.0) / v_m_s if v_m_s > 0 else 0.0

# ---------- pathfinding ----------
def energy_heuristic(u, v, G, min_kwh_per_km=0.12):
    """
    Heuristic: straight-line distance * min_kwh_per_km (kWh/km)
    """
    y1, x1 = G.nodes[u]['y'], G.nodes[u]['x']
    y2, x2 = G.nodes[v]['y'], G.nodes[v]['x']
    try:
        dist_m = ox.distance.great_circle(y1, x1, y2, x2)
    except Exception:
        # fallback to Euclidean approx on lon/lat
        dist_m = math.hypot((y2 - y1) * 111000.0, (x2 - x1) * 111000.0)
    return (dist_m / 1000.0) * min_kwh_per_km

def energy_astar(G, orig_node, dest_node, vehicle, default_speed_kph=30.0, regen_eff=0.6):
    """
    A* minimizing energy_kwh. Returns node list or None.
    """
    add_energy_attributes(G, vehicle, default_speed_kph=default_speed_kph, regen_eff=regen_eff)
    min_rate = vehicle.get("min_kwh_per_km", 0.05)

    def edge_energy_weight(u, v, d):
        # d can be dict of key->attr or attr dict; use helper on attr dict(s)
        # If d is mapping of parallel edges, take minimal energy among them.
        if isinstance(d, dict) and any(isinstance(vv, dict) for vv in d.values()):
            # MultiDiGraph case
            best = float('inf')
            for val in d.values():
                if isinstance(val, dict):
                    best = min(best, val.get('energy_kwh', float('inf')))
            return best if best != float('inf') else 0.0
        else:
            return _get_edge_attr_single(u, v, d, 'energy_kwh')

    try:
        route = nx.astar_path(G, orig_node, dest_node,
                              heuristic=lambda a, b: energy_heuristic(a, b, G, min_rate),
                              weight=edge_energy_weight)
    except Exception:
        return None
    return route

# ---------- metrics & geojson ----------
def route_metrics_and_geojson(G, route):
    """
    Given node-list route, compute total distance (m), energy (kWh), time (s), GeoJSON LineString.
    """
    if not route or len(route) < 2:
        return {"error": "Route could not be computed or too short."}, None

    total_m = 0.0
    total_kwh = 0.0
    total_s = 0.0
    coords = []
    for n in route:
        node = G.nodes[n]
        coords.append((node.get('y'), node.get('x')))

    for u, v in zip(route[:-1], route[1:]):
        data = G.get_edge_data(u, v)
        if not data:
            continue
        # choose minimal-energy among parallel edges
        if isinstance(data, dict) and any(isinstance(vv, dict) for vv in data.values()):
            candidates = list(data.values())
            ed = min(candidates, key=lambda x: x.get('energy_kwh', float('inf')))
        else:
            ed = data
        total_m += ed.get('length', 0.0)
        total_kwh += ed.get('energy_kwh', 0.0)
        total_s += ed.get('time_s', 0.0)

    # build LineString (lon,lat)
    line = LineString([(lon, lat) for lat, lon in coords])
    geojson = {"type": "Feature", "properties": {}, "geometry": line.__geo_interface__}
    metrics = {"distance_m": total_m, "energy_kwh": total_kwh, "time_s": total_s}
    return metrics, geojson

# ---------- nearest node ----------
def nearest_node(G, lat, lon):
    """
    Return nearest node id to given lat,lon.
    """
    try:
        # osmnx helper expects lon,lat order
        return ox.nearest_nodes(G, lon, lat)
    except Exception:
        # older versions
        return ox.get_nearest_node(G, (lat, lon))

# ---------- charging station utilities ----------
def generate_charging_stations_around(lat1, lon1, lat2=None, lon2=None, num_stations=4):
    """
    Generate charging stations evenly distributed in a circular pattern
    around the midpoint between start and end points.
    If no destination is provided, fallback to random placement around start.
    """
    stations = []

    # If destination is not provided, fallback to around-start random placement
    if lat2 is None or lon2 is None:
        distances_m = [100, 1000, 3000, 5000]
        for d in distances_m:
            ang = random.Random(int((lat1 + lon1) * 1e6) + d).uniform(0, 2 * math.pi)
            delta_lat = (d / 111000.0) * math.cos(ang)
            delta_lon = (d / (111000.0 * math.cos(math.radians(lat1)))) * math.sin(ang)
            s_lat = lat1 + delta_lat
            s_lon = lon1 + delta_lon
            stations.append({"lat": s_lat, "lon": s_lon})
        return stations

    # --- Circular distribution around midpoint ---
    mid_lat = (lat1 + lat2) / 2
    mid_lon = (lon1 + lon2) / 2
    radius_deg = 0.015  # roughly 1.5 km radius circle (adjust as needed)

    for i in range(num_stations):
        angle = 2 * math.pi * i / num_stations
        s_lat = mid_lat + radius_deg * math.cos(angle)
        s_lon = mid_lon + radius_deg * math.sin(angle)
        stations.append({"lat": s_lat, "lon": s_lon})

    return stations


def map_stations_to_nodes(G, stations):
    """
    For each station (lat,lon) find nearest graph node and attach 'node' key.
    """
    mapped = []
    for s in stations:
        try:
            node = nearest_node(G, s['lat'], s['lon'])
            mapped.append({"lat": s['lat'], "lon": s['lon'], "node": int(node)})
        except Exception:
            mapped.append({"lat": s['lat'], "lon": s['lon'], "node": None})
    return mapped

def find_nearest_station_node(G, orig_node, stations_mapped):
    """
    Given mapped stations with node ids, find the nearest station by shortest_path_length (length).
    Returns station dict or None.
    """
    nearest = None
    best_dist = float('inf')
    for s in stations_mapped:
        node = s.get('node')
        if node is None:
            continue
        try:
            d = nx.shortest_path_length(G, orig_node, node, weight='length')
            if d < best_dist:
                best_dist = d
                nearest = s
        except Exception:
            continue
    return nearest

def stations_on_route(G, route, stations_mapped):
    """
    Check which station nodes are exactly on the route path nodes.
    Returns list of station entries that lie on route (node match).
    """
    route_set = set(route)
    onroute = []
    for s in stations_mapped:
        if s.get('node') in route_set:
            onroute.append(s)
    return onroute
