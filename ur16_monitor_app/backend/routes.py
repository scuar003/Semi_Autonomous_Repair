from backend.controllers import node_controller, log_controller, network_controller

def register_routes(app):

    # Node control endpoints
    app.register_blueprint(node_controller.node_bp, url_prefix='/api/nodes')
    # Log endpoints
    app.register_blueprint(log_controller.log_bp, url_prefix='/api/logs')
    # Network endpoints
    app.register_blueprint(network_controller.network_bp, url_prefix='/api/network')
