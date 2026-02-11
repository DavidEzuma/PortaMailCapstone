import os
import unittest
from unittest.mock import patch

from app import app


class UiDebugToggleTests(unittest.TestCase):
    def setUp(self):
        self.client = app.test_client()

    def test_debug_panel_is_enabled_by_default(self):
        with patch.dict(os.environ, {"LCD_SHOW_DEBUG_PANEL": "1"}, clear=False):
            resp = self.client.get("/")
        html = resp.data.decode("utf-8")
        self.assertEqual(resp.status_code, 200)
        self.assertIn('class="layout has-debug"', html)
        self.assertIn('class="debug"', html)
        self.assertIn('id="dbgMode"', html)
        self.assertIn('id="introOverlay"', html)
        self.assertIn("touch to start", html)
        self.assertIn("Select destination", html)
        self.assertNotIn("Welcome to PortaMail</h2>", html)
        self.assertIn('class="car-group"', html)
        self.assertIn('class="btn big room-btn destination-btn"', html)
        self.assertIn('id="selectedRoomsRow"', html)
        self.assertIn('data-selected-chip="ROOM1"', html)
        self.assertIn('data-selected-chip="ROOM2"', html)
        self.assertIn('id="deliverStartBtn"', html)

    def test_debug_panel_can_be_disabled(self):
        with patch.dict(os.environ, {"LCD_SHOW_DEBUG_PANEL": "0"}, clear=False):
            resp = self.client.get("/")
        html = resp.data.decode("utf-8")
        self.assertEqual(resp.status_code, 200)
        self.assertIn('class="layout"', html)
        self.assertNotIn('class="layout has-debug"', html)
        self.assertNotIn('class="debug"', html)
        self.assertNotIn('id="dbgMode"', html)
        self.assertIn('id="introOverlay"', html)


if __name__ == "__main__":
    unittest.main()
