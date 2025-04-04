import unittest
from backend.app import app

class NodeControllerTestCase(unittest.TestCase):
    def setUp(self):
        self.app = app.test_client()
        self.app.testing = True

    def test_start_stop_status(self):
        # Test starting the node.
        start_response = self.app.post('/api/nodes/start')
        self.assertEqual(start_response.status_code, 200)
        # Test retrieving the node status.
        status_response = self.app.get('/api/nodes/status')
        self.assertEqual(status_response.status_code, 200)
        # Test stopping the node.
        stop_response = self.app.post('/api/nodes/stop')
        self.assertEqual(stop_response.status_code, 200)

if __name__ == '__main__':
    unittest.main()
