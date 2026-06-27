import ast
import os
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


class FakeRospy:
    def __init__(self, params=None):
        if params is None:
            params = {}
        self.params = params

    def get_param(self, name, default_value=None):
        return self.params.get(name, default_value)


class HxjDuojiNodeTest(unittest.TestCase):
    def test_parse_int_list_accepts_ros_string(self):
        self.assertEqual(node.DuojiConfig.parse_int_list("[0, 1, 2, 6]"), [0, 1, 2, 6])

    def test_parse_int_list_accepts_single_number(self):
        self.assertEqual(node.DuojiConfig.parse_int_list(3), [3])

    def test_build_servo_state_reads_sync_monitor_object(self):
        servo = FakeServo(angle=12.5, current=0.3, voltage=7.4, power=1.2, temp=30.0, status=0)
        state = node.ControlWorker.build_servo_state(2, servo)

        self.assertEqual(state["id"], 2)
        self.assertEqual(state["angle"], 12.5)
        self.assertEqual(state["current"], 0.3)
        self.assertTrue(state["online"])

    def test_make_angle_array_uses_servo_id_as_index(self):
        shared_state = node.DuojiConfig.create_shared_state_from_values([0, 2], 4)
        shared_state["servos"][0] = {"id": 0, "angle": 10.0, "online": True}
        shared_state["servos"][2] = {"id": 2, "angle": -5.0, "online": True}
        worker = node.RosWorker({}, shared_state, None, None, None)

        self.assertEqual(worker.make_angle_array(shared_state), [10.0, 0.0, -5.0, 0.0])

    def test_read_sync_monitor_uses_manager_servos_when_api_returns_none(self):
        states = node.ControlWorker.read_sync_monitor(FakeManager(), [1])

        self.assertEqual(states[1]["angle"], 21.0)
        self.assertEqual(states[1]["current"], 0.5)
        self.assertTrue(states[1]["online"])

    def test_unknown_control_state_switches_to_error(self):
        lock = node.threading.Lock()
        shared_state = node.DuojiConfig.create_shared_state_from_values([0], 1)
        shared_state["control_state"] = "BAD_STATE"
        worker = node.ControlWorker({}, shared_state, lock, None, None)

        worker.run_state_machine_once(None)

        self.assertEqual(shared_state["control_state"], node.STATE_ERROR)

    def test_distribute_end_damping_uses_tangential_joint_lengths(self):
        arm_params = {
            "dof": 3,
            "joints": {
                "joint1": {"type": "axial", "link": {"length": 0.18}},
                "joint2": {"type": "tangential", "link": {"length": 0.15}},
                "joint3": {"type": "tangential", "link": {"length": 0.10}},
            },
        }

        result = node.DuojiConfig.distribute_end_damping(1000.0, arm_params)

        self.assertAlmostEqual(result[1], 714.285714, places=5)
        self.assertAlmostEqual(result[2], 285.714285, places=5)
        self.assertNotIn(0, result)

    def test_distribute_end_damping_returns_empty_when_no_tangential_joint(self):
        arm_params = {
            "dof": 2,
            "joints": {
                "joint1": {"type": "axial", "link": {"length": 0.18}},
                "joint2": {"type": "axial", "link": {"length": 0.15}},
            },
        }

        self.assertEqual(node.DuojiConfig.distribute_end_damping(1000.0, arm_params), {})

    def test_handle_plan_stores_damping_targets_in_shared_state(self):
        lock = node.threading.Lock()
        shared_state = node.DuojiConfig.create_shared_state_from_values([0, 1, 2], 3)
        config = {
            "end_damping": 1000.0,
            "arm_params": {
                "dof": 3,
                "joints": {
                    "joint1": {"type": "axial", "link": {"length": 0.18}},
                    "joint2": {"type": "tangential", "link": {"length": 0.15}},
                    "joint3": {"type": "tangential", "link": {"length": 0.10}},
                },
            },
        }

        worker = node.ControlWorker(config, shared_state, lock, None, None)
        worker.handle_plan(None)

        self.assertAlmostEqual(shared_state["damping_targets"][1], 714.285714, places=5)
        self.assertAlmostEqual(shared_state["damping_targets"][2], 285.714285, places=5)

    def test_read_ros_config_uses_local_config_example_by_default(self):
        config = node.DuojiConfig.read_ros_config(FakeRospy())
        expected_path = os.path.join(os.path.dirname(node.__file__), "config", "example.json")

        self.assertEqual(config["arm_config"], expected_path)
        self.assertTrue(os.path.exists(config["arm_config"]))

    def test_worker_classes_exist(self):
        self.assertTrue(hasattr(node, "DuojiConfig"))
        self.assertTrue(hasattr(node, "RosWorker"))
        self.assertTrue(hasattr(node, "ControlWorker"))

    def test_duoji_config_loads_local_json(self):
        config_reader = node.DuojiConfig(FakeRospy())

        self.assertTrue(os.path.exists(config_reader.values["arm_config"]))
        self.assertIsNotNone(config_reader.values["arm_params"])
        self.assertEqual(config_reader.values["arm_params"]["dof"], 3)

    def test_control_worker_plan_updates_damping_targets(self):
        lock = node.threading.Lock()
        shared_state = node.DuojiConfig.create_shared_state_from_values([0, 1, 2], 3)
        config_reader = node.DuojiConfig(FakeRospy({"~num_servos": 3, "~servo_ids": "[0,1,2]", "~end_damping": 1000.0}))
        worker = node.ControlWorker(config_reader.values, shared_state, lock, None, None)

        worker.handle_plan(None)

        self.assertAlmostEqual(shared_state["damping_targets"][1], 714.285714, places=5)
        self.assertAlmostEqual(shared_state["damping_targets"][2], 285.714285, places=5)

    def test_load_example_json_and_distribute_damping(self):
        json_path = os.path.join(os.path.dirname(node.__file__), "config", "example.json")

        arm_params = node.DuojiConfig.load_arm_params_file(json_path)
        result = node.DuojiConfig.distribute_end_damping(1000.0, arm_params)

        self.assertAlmostEqual(result[1], 714.285714, places=5)
        self.assertAlmostEqual(result[2], 285.714285, places=5)

    def test_function_summaries_are_above_def_lines(self):
        source_path = "hxj_duoji_node.py"
        with open(source_path, "r", encoding="utf-8") as file_obj:
            source_text = file_obj.read()
        source_lines = source_text.splitlines()
        tree = ast.parse(source_text)

        for item in tree.body:
            if not isinstance(item, ast.FunctionDef):
                continue
            self.assertIsNone(ast.get_docstring(item))
            previous_line = source_lines[item.lineno - 2].strip()
            self.assertTrue(
                previous_line.startswith("#"),
                msg="函数 %s 上方缺少 # 摘要注释" % item.name,
            )

    def test_old_top_level_helpers_are_removed(self):
        removed_names = [
            "load_arm_params",
            "joint_number",
            "ordered_joint_items",
            "joint_link_length",
            "calculate_tangential_weights",
            "distribute_end_damping",
            "parse_int_list",
            "get_attr",
            "build_servo_state",
            "get_servo_from_manager",
            "create_shared_state",
            "make_angle_array",
            "make_current_array",
            "read_ros_config",
            "open_servo_manager",
            "read_sync_monitor",
            "update_shared_servo_state",
            "set_control_state",
            "copy_shared_state",
        ]

        for name in removed_names:
            self.assertFalse(hasattr(node, name), msg="%s 还留在类外面" % name)


if __name__ == "__main__":
    unittest.main()
