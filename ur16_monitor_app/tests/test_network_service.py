import unittest
from backend.app import app

class NetworkServiceTestCase(unittest.TestCase):
    def setUp(self):
        self.app = app.test_client()
        self.app.testing = True

    def test_network_info(self):
        response = self.app.get('/api/network/')
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertIn("hostname", data)
        self.assertIn("ip_address", data)

if __name__ == '__main__':
    unittest.main()
