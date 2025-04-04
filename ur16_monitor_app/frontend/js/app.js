// Control a node action (start/stop) for the given node.
async function controlNode(node, action) {
    const url = `/api/nodes/${node}/${action}`;
    const response = await fetch(url, { method: 'POST' });
    const result = await response.json();
    document.getElementById('result').innerText = JSON.stringify(result, null, 2);
    updateStatusIndicator(node);
  }
  
  // Check the status of a specific node.
  async function checkStatus(node) {
    const response = await fetch(`/api/nodes/${node}/status`);
    const result = await response.json();
    document.getElementById('result').innerText = JSON.stringify(result, null, 2);
    updateStatusIndicator(node, result);
  }
  
  // Update the status indicator for a given node.
  function updateStatusIndicator(node, statusData) {
    if (statusData) {
      setIndicator(node, statusData.status);
    } else {
      fetch(`/api/nodes/${node}/status`)
        .then(response => response.json())
        .then(data => {
          setIndicator(node, data.status);
        });
    }
  }
  
  // Set the visual indicator (color and shadow) based on status.
  function setIndicator(node, status) {
    const indicator = document.getElementById(`statusIndicator-${node}`);
    if (status === 'running') {
      indicator.style.backgroundColor = '#0f0'; // green for running
      indicator.style.boxShadow = '0 0 20px #0f0';
    } else {
      indicator.style.backgroundColor = '#800'; // red for offline/error
      indicator.style.boxShadow = '0 0 15px rgba(0, 0, 0, 0.5)';
    }
  }
  
  // Initialize status for all nodes on page load.
  window.onload = function() {
    ['turtlesim', 'realsense', 'ur_robot'].forEach(node => {
      updateStatusIndicator(node);
    });
  };
  