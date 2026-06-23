import unittest

import hxj_duoji_node as node


class FakeServo:
    def __init__(self, angle, current=0.0, voltage=0.0, power=0.0, temp=0.0, status=0):
        self.angle_monitor = angle
        self.current = current
        self.voltage = voltage
        self.power = power
        self.temp = temp
        self.status = status


class FakeServoTable:
    def __getitem__(self, servo_id):
        if servo_id == 1:
            return FakeServo(angle=21.0, current=0.5)
        raise KeyError(servo_id)


class FakeManager:
    def __init__(self):
        self.servos = FakeServoTable()

    def send_sync_servo_monitor(self, servo_ids):
        return None


class HxjDuojiNodeTest(unittest.TestCase):
    def test_parse_int_list_accepts_ros_string(self):
        self.assertEqual(node.parse_int_list("[0, 1, 2, 6]"), [0, 1, 2, 6])

    def test_parse_int_list_accepts_single_number(self):
        self.assertEqual(node.parse_int_list(3), [3])

    def test_build_servo_state_reads_sync_monitor_object(self):
        servo = FakeServo(angle=12.5, current=0.3, voltage=7.4, power=1.2, temp=30.0, status=0)
        state = node.build_servo_state(2, servo)

        self.assertEqual(state["id"], 2)
        self.assertEqual(state["angle"], 12.5)
        self.assertEqual(state["current"], 0.3)
        self.assertTrue(state["online"])

    def test_make_angle_array_uses_servo_id_as_index(self):
        shared_state = node.create_shared_state([0, 2], 4)
        shared_state["servos"][0] = {"id": 0, "angle": 10.0, "online": True}
        shared_state["servos"][2] = {"id": 2, "angle": -5.0, "online": True}

        self.assertEqual(node.make_angle_array(shared_state), [10.0, 0.0, -5.0, 0.0])

    def test_read_sync_monitor_uses_manager_servos_when_api_returns_none(self):
        states = node.read_sync_monitor(FakeManager(), [1])

        self.assertEqual(states[1]["angle"], 21.0)
        self.assertEqual(states[1]["current"], 0.5)
        self.assertTrue(states[1]["online"])

    def test_unknown_control_state_switches_to_error(self):
        lock = node.threading.Lock()
        shared_state = node.create_shared_state([0], 1)
        shared_state["control_state"] = "BAD_STATE"

        node.run_state_machine_once(None, {}, shared_state, lock)

        self.assertEqual(shared_state["control_state"], node.STATE_ERROR)


if __name__ == "__main__":
    unittest.main()
