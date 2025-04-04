import socket

def get_network_info():
    try:
        hostname = socket.gethostname()
        ip_address = socket.gethostbyname(hostname)
        return {"hostname": hostname, "ip_address": ip_address}
    except Exception as e:
        return {"error": str(e)}
