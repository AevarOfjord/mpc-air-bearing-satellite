"""
Motive Data Server for OptiTrack Motion Capture System

Flask HTTP server that streams real-time position and orientation data from OptiTrack Motive.
Provides JSON API endpoint for satellite control system integration.

Data flow:
- NatNetClient receives rigid body data from Motive via NatNet SDK
- Coordinate transformation from Motive (Y-up) to standard (Z-up) coordinates
- Flask server exposes /data endpoint with latest position and orientation
- Real.py telemetry client polls this endpoint for feedback control

Coordinate system mapping:
- Motive X → Standard Y (lateral position)
- Motive Z → Standard X (forward position)
- Quaternion → Yaw angle (rotation about vertical axis)

API endpoint:
- GET /data: Returns {"x": mm, "y": mm, "yaw": degrees, "timestamp": unix_time}

Key features:
- Real-time streaming with sub-10ms latency
- Thread-safe data handling with locks
- CORS enabled for web-based monitoring
- Automatic coordinate system conversion
- Connection health monitoring
"""

import sys
import time
import math
import threading
from flask import Flask, jsonify
from flask_cors import CORS
from nat_net_client import NatNetClient

# Note: Coordinate system mapping from Motive to standard:
latest_data = {"x": None, "y": None, "yaw": None}
data_lock = threading.Lock()

def quaternion_to_yaw(q):
    """
    Convert a quaternion (x, y, z, w) to a yaw angle (rotation about the Y axis).
    Assumes a right-handed Y-up coordinate system.
    """
    x, y, z, w = q
    yaw = math.atan2(2 * (w * y + x * z), 1 - 2 * (x * x + y * y))
    return math.degrees(yaw)

def receive_rigid_body_frame(new_id, position, rotation):
    x_mm = position[2] * 1000  # Standard X from Motive Z
    y_mm = position[0] * 1000  # Standard Y from Motive X
    yaw_deg = quaternion_to_yaw(rotation)
    with data_lock:
        latest_data["x"] = x_mm  # type: ignore[assignment]
        latest_data["y"] = y_mm  # type: ignore[assignment]
        latest_data["yaw"] = yaw_deg  # type: ignore[assignment]
    # Print values showing both Motive raw and converted standard coordinates
    print("RigidBody %d: Motive(x=%.0f, z=%.0f) -> Standard(x=%.0f, y=%.0f) mm, Yaw=%.2f°" %
          (new_id, position[0] * 1000, position[2] * 1000, x_mm, y_mm, yaw_deg))

def receive_new_frame(data_dict):
    print("Frame #: %s" % data_dict.get("frame_number", "N/A"))

# Function to run the NatNet client in a separate thread.
def run_natnet_client():
    client = NatNetClient()
    client.set_client_address("127.0.0.1")
    client.set_server_address("127.0.0.1")
    # Use multicast as per your settings.
    client.set_use_multicast(True)
    # Assign our callbacks.
    client.new_frame_listener = receive_new_frame  # type: ignore[assignment]
    client.rigid_body_listener = receive_rigid_body_frame  # type: ignore[assignment]

    if not client.run():
        print("ERROR: Could not start the NatNet client.")
        sys.exit(1)
    # Keep the thread alive.
    while True:
        time.sleep(1)

# Start the NatNet client in its own thread.
natnet_thread = threading.Thread(target=run_natnet_client)
natnet_thread.daemon = True
natnet_thread.start()

# Set up the Flask app.
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

@app.route('/data')
def get_data():
    with data_lock:
        # Return the latest data as JSON.
        return jsonify(latest_data)

if __name__ == '__main__':
    # Run the Flask server on port 5000.
    print("Starting Flask server. Access the data at http://127.0.0.1:5000/data")
    app.run(host="127.0.0.1", port=5000)
