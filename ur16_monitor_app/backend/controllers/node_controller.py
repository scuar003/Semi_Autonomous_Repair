from flask import Blueprint, jsonify
from backend.services.ros_service import start_node, stop_node, get_node_status

node_bp = Blueprint('node_controller', __name__)

@node_bp.route('/<node_name>/start', methods=['POST'])
def start_node_endpoint(node_name):
    result = start_node(node_name)
    return jsonify(result)

@node_bp.route('/<node_name>/stop', methods=['POST'])
def stop_node_endpoint(node_name):
    result = stop_node(node_name)
    return jsonify(result)

@node_bp.route('/<node_name>/status', methods=['GET'])
def node_status_endpoint(node_name):
    result = get_node_status(node_name)
    return jsonify(result)
