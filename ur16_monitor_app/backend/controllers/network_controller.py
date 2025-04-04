from flask import Blueprint, jsonify
from backend.services.network_service import get_network_info

network_bp = Blueprint('network_controller', __name__)

@network_bp.route('/', methods=['GET'])
def network_info():
    info = get_network_info()
    return jsonify(info)
