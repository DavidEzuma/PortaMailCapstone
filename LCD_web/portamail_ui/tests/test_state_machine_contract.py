import re
import unittest

from state.model import snapshot_state
from state.state_machine import handle_edge, set_mode
from tests.test_support import TemporaryLogPath, reset_runtime_state


class StateMachineContractTests(unittest.TestCase):
    def setUp(self):
        reset_runtime_state()
        self._log_ctx = TemporaryLogPath()
        self.log_path = self._log_ctx.__enter__()

    def tearDown(self):
        self._log_ctx.__exit__(None, None, None)

    def _read_log_lines(self):
        with open(self.log_path, "r", encoding="utf-8") as f:
            return [line.strip() for line in f.readlines() if line.strip()]

    def test_queue_ordering_is_room1_priority(self):
        self.assertTrue(handle_edge("start_room2"))
        self.assertTrue(handle_edge("start_room1"))
        st = snapshot_state()
        self.assertEqual(st["pending_rooms"], ["ROOM1", "ROOM2"])
        self.assertEqual(st["active_room"], "ROOM2")
        self.assertEqual(st["screen"], "DELIVERING_ROOM2")

    def test_open_confirm_flow_requires_arrived(self):
        self.assertFalse(handle_edge("open_confirm_flow"))
        self.assertEqual(snapshot_state()["screen"], "HOME")
        self.assertTrue(set_mode("ARRIVED"))
        self.assertTrue(handle_edge("open_confirm_flow"))
        self.assertEqual(snapshot_state()["screen"], "CONFIRM_SELECT")

    def test_delivery_confirmed_logs_and_advances_queue(self):
        self.assertTrue(handle_edge("start_room1"))
        self.assertTrue(handle_edge("start_room2"))
        self.assertTrue(set_mode("ARRIVED"))
        self.assertTrue(handle_edge("open_confirm_flow"))
        self.assertTrue(handle_edge("select_room1"))
        self.assertTrue(handle_edge("delivery_confirmed"))

        lines = self._read_log_lines()
        self.assertEqual(len(lines), 1)
        self.assertRegex(lines[0], r"^.+,ROOM1$")
        st = snapshot_state()
        self.assertEqual(st["active_room"], "ROOM2")
        self.assertEqual(st["pending_rooms"], ["ROOM2"])
        self.assertEqual(st["screen"], "DELIVERING_ROOM2")

    def test_delivery_confirmed_without_valid_flow_does_not_log(self):
        self.assertFalse(handle_edge("delivery_confirmed"))
        self.assertEqual(self._read_log_lines(), [])

    def test_log_line_uses_iso_timestamp_with_offset(self):
        self.assertTrue(set_mode("ARRIVED"))
        self.assertTrue(handle_edge("open_confirm_flow"))
        self.assertTrue(handle_edge("select_room2"))
        self.assertTrue(handle_edge("delivery_confirmed"))
        line = self._read_log_lines()[0]
        timestamp, room = line.split(",")
        self.assertEqual(room, "ROOM2")
        self.assertRegex(timestamp, r"^\d{4}-\d{2}-\d{2}T")
        self.assertTrue(timestamp.endswith("Z") or re.search(r"[+-]\d{2}:\d{2}$", timestamp))


if __name__ == "__main__":
    unittest.main()
