
from flask import Flask, request, jsonify, render_template
import routing
import osmnx as ox
import networkx as nx
import time
import math
app = Flask(__name__)

# Load graph once (adjust place_name or bbox as needed)
PLACE = "Vellore, India"
print("Loading graph for:", PLACE)
# Use force_reload=False here; change to True if you want to re-download
G = routing.load_graph(place_name=PLACE, dist_m=100000, force_reload=False) if hasattr(routing, 'load_graph') else routing.load_graph(place_name=PLACE)
print("Graph loaded: nodes:", len(G.nodes), "edges:", len(G.edges))

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/route", methods=["POST"])
def compute_route():
    """
    Expects JSON:
    {
      "orig": [lat, lon],
      "dest": [lat, lon],
      "vehicle": {...},
      "defaults": {...},
      "battery_level": 100
    }
    """
    t_start = time.time()
    data = request.get_json()
    orig = data.get("orig")
    dest = data.get("dest")
    vehicle = data.get("vehicle", {})
    defaults = data.get("defaults", {})
    battery_level = float(data.get("battery_level", data.get("battery_percent", 100)))

    default_speed_kph = defaults.get("speed_kph", 30.0)
    regen_eff = defaults.get("regen_eff", 0.6)

    if not orig or not dest:
        return jsonify({"error": "orig and dest required"}), 400

    # nearest nodes for origin and destination
    o_node = routing.nearest_node(G, orig[0], orig[1])
    d_node = routing.nearest_node(G, dest[0], dest[1])

    response = {"timing_s": None, "mode": None}

    # map behavior by battery_level
    if battery_level >= 75:
        # FULL battery: return both energy-optimal and shortest-distance to destination
        response["mode"] = "full"
        # energy-optimal route
        route_energy = routing.energy_astar(G, o_node, d_node, vehicle, default_speed_kph, regen_eff)
        if route_energy is not None:
            metrics_e, geo_e = routing.route_metrics_and_geojson(G, route_energy)
            response["energy_route"] = {"metrics": metrics_e, "geojson": geo_e, "nodes": route_energy}
        else:
            response["energy_route"] = None

        # shortest-distance route
        try:
            route_dist = nx.shortest_path(G, o_node, d_node, weight="length")
            metrics_d, geo_d = routing.route_metrics_and_geojson(G, route_dist)
            response["distance_route"] = {"metrics": metrics_d, "geojson": geo_d, "nodes": route_dist}
        except Exception as ex:
            response["distance_route"] = None

        # no stations required, but still provide a small empty list to frontend
        response["charging_stations"] = []

    elif 25 < battery_level < 75:
        # HALF battery: must pass through nearest charging station
        response["mode"] = "half"

        # Generate evenly distributed charging stations between origin & destination
        stations = routing.generate_charging_stations_around(orig[0], orig[1], dest[0], dest[1])

        # --- Find nearest charging station to the origin ---
        def haversine(lat1, lon1, lat2, lon2):
            R = 6371000  # meters
            dlat = math.radians(lat2 - lat1)
            dlon = math.radians(lat2 - lon1)
            a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
            return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        nearest_station = min(stations, key=lambda s: haversine(orig[0], orig[1], s["lat"], s["lon"]))
        print(f"Routing via nearest charging station at {nearest_station}")

        # --- Compute two-part route: origin → station → destination ---
        # Convert coordinates to nearest graph nodes
        o_node = routing.nearest_node(G, orig[0], orig[1])
        station_node = routing.nearest_node(G, nearest_station["lat"], nearest_station["lon"])
        d_node = routing.nearest_node(G, dest[0], dest[1])

        # Compute energy-optimal routes (origin → station → destination)
        route1 = routing.energy_astar(G, o_node, station_node, vehicle)
        route2 = routing.energy_astar(G, station_node, d_node, vehicle)
        print(f"Route1: {route1[:5] if route1 else None}")
        print(f"Route2: {route2[:5] if route2 else None}")
        print(f"Origin node: {o_node}, Station node: {station_node}, Destination node: {d_node}")

        if route1 and route2:
            # Convert node lists to metrics + geojson
            metrics1, geo1 = routing.route_metrics_and_geojson(G, route1)
            metrics2, geo2 = routing.route_metrics_and_geojson(G, route2)

            # Combine both GeoJSON routes into a single LineString
            combined_coords = []
            if geo1["geometry"]["type"] == "LineString":
                combined_coords += geo1["geometry"]["coordinates"]
            if geo2["geometry"]["type"] == "LineString":
                combined_coords += geo2["geometry"]["coordinates"]

            combined_geo = {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": combined_coords
                },
                "properties": {}
            }

            # Combine metrics
            total_metrics = {
                "distance_m": metrics1["distance_m"] + metrics2["distance_m"],
                "energy_kwh": metrics1["energy_kwh"] + metrics2["energy_kwh"],
                "time_s": metrics1["time_s"] + metrics2["time_s"],
            }

            # ✅ Energy route passes via nearest charging station
            response["energy_route"] = {"metrics": total_metrics, "geojson": combined_geo}
            response["charging_stations"] = stations
            response["note"] = "Battery 25–75%: Energy route passes through nearest charging station"

            # ✅ Add shortest-distance route (direct origin → destination)
            try:
                route_dist = nx.shortest_path(G, o_node, d_node, weight="length")
                metrics_d, geo_d = routing.route_metrics_and_geojson(G, route_dist)
                response["distance_route"] = {
                    "metrics": metrics_d,
                    "geojson": geo_d,
                    "nodes": route_dist
                }
            except Exception as ex:
                print("Shortest-distance route failed:", ex)
                response["distance_route"] = None


        else:
            # Fallback if routing fails: use normal energy and distance routes
            stations_mapped = routing.map_stations_to_nodes(G, stations)

            route_energy = routing.energy_astar(G, o_node, d_node, vehicle, default_speed_kph, regen_eff)
            route_dist = None
            try:
                route_dist = nx.shortest_path(G, o_node, d_node, weight="length")
            except Exception:
                route_dist = None

            if route_energy is not None:
                metrics_e, geo_e = routing.route_metrics_and_geojson(G, route_energy)
                response["energy_route"] = {"metrics": metrics_e, "geojson": geo_e, "nodes": route_energy}
            else:
                response["energy_route"] = None

            if route_dist is not None:
                metrics_d, geo_d = routing.route_metrics_and_geojson(G, route_dist)
                response["distance_route"] = {"metrics": metrics_d, "geojson": geo_d, "nodes": route_dist}
            else:
                response["distance_route"] = None

            response["charging_stations"] = stations_mapped
            response["note"] = "Fallback: direct route used"

    
    else:
        # LOW battery (<=25): find nearest charging station and route ONLY to that station (shortest)
        response["mode"] = "low"
        stations = routing.generate_charging_stations_around(orig[0], orig[1], dest[0], dest[1])

        stations_mapped = routing.map_stations_to_nodes(G, stations)

        nearest = routing.find_nearest_station_node(G, o_node, stations_mapped)
        if nearest is None:
            response["charging_stations"] = stations_mapped
            response["low_battery_route"] = None
        else:
            try:
                station_node = nearest["node"]
                route_to_station = nx.shortest_path(G, o_node, station_node, weight="length")
                metrics_s, geo_s = routing.route_metrics_and_geojson(G, route_to_station)
                response["low_battery_route"] = {"metrics": metrics_s, "geojson": geo_s, "nodes": route_to_station}
            except Exception:
                response["low_battery_route"] = None
            # annotate nearest station
            for s in stations_mapped:
                s["is_nearest"] = (s.get("node") == nearest.get("node"))
            response["charging_stations"] = stations_mapped

    response["timing_s"] = time.time() - t_start
    return jsonify(response)

if __name__ == "__main__":
    app.run(debug=True, port=5000)


