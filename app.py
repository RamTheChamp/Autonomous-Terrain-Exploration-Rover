"""
Mars Rover Web Dashboard (Python side)
---------------------------------------
This is a tiny Flask app that serves a local dashboard page. The page
itself connects DIRECTLY to the ESP32's WebSocket endpoint (ws://<esp32-ip>/ws)
from the browser - that's the same WiFi network as the rover, so no data
needs to be relayed through this Python process. Python's only job here is
to host the HTML/JS/CSS file so you can open it in a browser.

Run:
    pip install flask
    python app.py
Then open:
    http://127.0.0.1:5000
Enter the ESP32's IP address (printed on Serial Monitor at boot) in the
dashboard's IP field and click Connect.
"""

from flask import Flask, render_template

app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html")


if __name__ == "__main__":
    # host="0.0.0.0" lets you open the dashboard from a phone on the same
    # WiFi too, e.g. http://<your-pc-ip>:5000
    app.run(host="0.0.0.0", port=5000, debug=True)
