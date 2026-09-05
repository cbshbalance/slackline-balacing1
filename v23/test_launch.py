import io
import json
import socket
import unittest
from unittest.mock import patch, Mock

import launch


class LaunchTests(unittest.TestCase):
    def free_port(self):
        with socket.socket() as s:
            s.bind(('127.0.0.1', 0))
            return s.getsockname()[1]

    def test_reserves_free_port(self):
        with patch.object(launch, 'is_our_server', return_value=False):
            port, listener = launch.find_endpoint(self.free_port(), 1)
        try:
            with socket.socket() as other:
                with self.assertRaises(OSError):
                    other.bind(('127.0.0.1', port))
        finally:
            listener.close()

    def test_skips_unrelated_occupied_port(self):
        with socket.socket() as busy:
            busy.bind(('127.0.0.1', 0))
            first = busy.getsockname()[1]
            with patch.object(launch, 'is_our_server', return_value=False):
                port, listener = launch.find_endpoint(first, 20)
            self.assertGreater(port, first)
            listener.close()

    def test_reuses_server_on_later_port_and_releases_reservation(self):
        first = self.free_port()
        with patch.object(launch, 'is_our_server', side_effect=[False, True]):
            port, listener = launch.find_endpoint(first, 2)
        self.assertEqual(port, first + 1)
        self.assertIsNone(listener)
        with socket.socket() as s:
            s.bind(('127.0.0.1', first))

    def test_all_busy(self):
        with socket.socket() as busy:
            busy.bind(('127.0.0.1', 0))
            with patch.object(launch, 'is_our_server', return_value=False):
                with self.assertRaises(OSError):
                    launch.find_endpoint(busy.getsockname()[1], 1)

    def test_identity_requires_same_application_and_workspace(self):
        for data, expected in [
            ({'app': launch.APP_ID, 'workspace': str(launch.HERE)}, True),
            ({'app': 'other', 'workspace': str(launch.HERE)}, False),
            ({'app': launch.APP_ID, 'workspace': '/different/workspace'}, False),
            ([], False),
        ]:
            opener = Mock()
            opener.open.return_value = io.BytesIO(json.dumps(data).encode())
            with patch.object(launch.urllib.request, 'build_opener', return_value=opener):
                self.assertEqual(launch.is_our_server(8230), expected)


if __name__ == '__main__':
    unittest.main()
