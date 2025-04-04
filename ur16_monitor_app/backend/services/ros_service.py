import subprocess
import os
import signal
import time
from backend.config import NODE_CONFIG

# Dictionary to store process handles for each node.
node_processes = {}

def start_node(node_name):
    if node_name not in NODE_CONFIG:
        return {"status": "error", "message": f"Unknown node: {node_name}"}
    
    # If the node is already running, report that.
    if node_name in node_processes and node_processes[node_name] and node_processes[node_name].poll() is None:
        return {"status": "running", "message": f"{node_name} is already running."}
    
    try:
        command = NODE_CONFIG[node_name]
        process = subprocess.Popen(
            command.split(),
            preexec_fn=os.setsid,  # Launch in its own process group.
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        node_processes[node_name] = process
        
        # Wait a short period for the process to initialize.
        time.sleep(1)
        retcode = process.poll()
        if retcode is not None:
            # Process terminated—capture its stderr output.
            stdout, stderr = process.communicate(timeout=1)
            error_message = stderr.strip() or f"Process terminated with exit code {retcode}"
            return {"status": "error", "message": error_message, "node": node_name}
        
        return {"status": "started", "node": node_name}
    except Exception as e:
        return {"status": "error", "message": str(e), "node": node_name}

def stop_node(node_name):
    if (node_name not in node_processes or 
        node_processes[node_name] is None or 
        node_processes[node_name].poll() is not None):
        return {"status": "not running", "message": f"{node_name} is not running.", "node": node_name}
    try:
        process = node_processes[node_name]
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5)
        return {"status": "stopped", "node": node_name}
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait()
        return {"status": "stopped", "node": node_name, "message": "Forced kill after timeout."}
    except Exception as e:
        return {"status": "error", "message": str(e), "node": node_name}

def get_node_status(node_name):
    if node_name in node_processes and node_processes[node_name] and node_processes[node_name].poll() is None:
        return {"status": "running", "node": node_name}
    else:
        return {"status": "stopped", "node": node_name}
