from flask import Flask, send_from_directory
import os
from backend.routes import register_routes  # Import your API route registration

app = Flask(__name__, static_folder="../frontend", static_url_path="")

# Serve the index.html when accessing the root
@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")

# (Other routes are already registered via routes.py)
# Register other API routes/blueprints
register_routes(app)

if __name__ == "__main__":
    app.run(debug=True)