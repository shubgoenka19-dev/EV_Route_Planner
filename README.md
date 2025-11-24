EV Energy-Optimal Route Planner
An intelligent routing system that computes energy-optimal routes for Electric Vehicles (EVs) using real road networks from OpenStreetMap.
The system supports battery-aware routing, charging station generation, and a Leaflet-based interactive map.
Features
1. Energy-Optimal Route
   .Computes routes based on:
     .Rolling resistance
     .Aerodynamic drag
     .Road grade
     .Regenerative braking.
   .Uses A* with custom energy heuristic.
2. Battery-Aware Routing Logic
   Battery percentage fully changes behavior:
   Battery - Level	Behavior
   75–100%	Show energy-optimal + shortest-distance routes
   25–75%	Must pass through nearest charging station (energy route only)
   0–25%	Route only to nearest charging station (shortest path)
3. Auto-Generated Charging Stations
   For 25–75% battery: charging stations are placed evenly around a circular region between origin & destination.
   For 0–25% battery: stations generated around origin.
4. Interactive Web Interface
   .Built with Flask + Leaflet.js
   .Click on the map to choose origin & destination
   .Displays:
       .Energy-optimal route (green)
       .Shortest distance route (blue)
       .Charging stations (red markers)
Project Structure
    ev-route-planner/
    │── app.py               # Flask backend
    │── routing.py           # Energy model + routing algorithms
    │── index.html           # Frontend UI with Leaflet map
    │── requirements.txt     # Dependencies
    │── README.md            # Project documentation

Installation & Setup
1️. Clone the repository
git clone https://github.com/your-username/ev-route-planner.git
cd ev-route-planner
2. Create virtual environment
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
3. Install dependencies
pip install -r requirements.txt

▶ Running the Application
    Start the Flask server:
    python app.py
    Then open in your browser:
    http://127.0.0.1:5000

How It Works
Battery Logic
The routing adapts based on battery percentage:

🔋 Battery ≥ 75%
  Computes two routes:
  Energy optimal
  Shortest distance
Both displayed on map
🔋 25% < Battery < 75%
  System finds nearest virtual charging station
  Ensures route passes through the station
  After reaching station → continues to destination
🔋 Battery ≤ 25%
  Computes route only to nearest station
  No destination routing

Charging Station Placement
  Stations are generated around the midpoint of origin and destination in a circular pattern, not in a straight line.
  Even distribution example:

        ●
   ●         ●      (charging stations around midpoint)
        ●

Screenshots

![imag1](https://github.com/user-attachments/assets/3e12432b-9790-464a-84db-64edfbb800c7)
![imag2](https://github.com/user-attachments/assets/133f95ee-fc42-487e-96f4-b4323ef888a0)
![imag3](https://github.com/user-attachments/assets/a85344f2-392d-4280-ab71-0bab146c27db)

PRs are welcome!
    Feel free to open issues for:
    Feature suggestions
    Bugs
    Optimization ideas











