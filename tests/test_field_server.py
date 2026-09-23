import unittest

from sentinel import field_server


class FieldServerStatusTests(unittest.TestCase):
    def test_status_payload_is_safe_and_readable(self):
        payload = field_server.status_payload()
        self.assertEqual(set(payload), {"phase", "detail"})
        self.assertIsInstance(payload["phase"], str)
        self.assertIsInstance(payload["detail"], str)
