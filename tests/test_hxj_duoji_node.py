import ast
import math
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


class HxjDuojiNodeTest(unittest.TestCase):
    def test_parse_int_list_accepts_ros_string(self):
        self.assertEqual(node.DuojiConfig.parse_int_list("[0, 1, 2, 6]"), [0, 1, 2, 6])

    def test_parse_int_list_accepts_single_number(self):
        self.assertEqual(node.DuojiConfig.parse_int_list(3), [3])

    def test_build_servo_state_reads_sync_monitor_object(self):
        servo = FakeServo(angle=12.5, current=0.3, voltage=7.4, power=1.2, temp=30.0, status=0)
        state = node.Controller.build_servo_state(2, servo)

        self.assertEqual(state["id"], 2)
        self.assertEqual(state["angle"], 12.5)
        self.assertEqual(state["current"], 0.3)
        self.assertTrue(state["online"])

    def test_make_state_array_uses_servo_id_as_index(self):
        shared_state = node.DuojiConfig.create_shared_state_from_values([0, 2], 4)
        shared_state["servos"][0] = {"id": 0, "angle": 10.0, "online": True}
        shared_state["servos"][2] = {"id": 2, "angle": -5.0, "online": True}

        result = node.RosPublisher.make_state_array(shared_state, "angle")
        self.assertEqual(result, [10.0, 0.0, -5.0, 0.0])

    def test_read_sync_monitor_uses_manager_servos_when_api_returns_none(self):
        states = node.Controller.read_sync_monitor(FakeManager(), [1])

        self.assertEqual(states[1]["angle"], 21.0)
        self.assertEqual(states[1]["current"], 0.5)
        self.assertTrue(states[1]["online"])

    def test_unknown_control_state_switches_to_error(self):
        lock = node.threading.Lock()
        shared_state = node.DuojiConfig.create_shared_state_from_values([0], 1)
        shared_state["control_state"] = "BAD_STATE"
        worker = node.Controller({}, shared_state, lock, None, None)

        worker.run_state_machine_once(None)

        self.assertEqual(shared_state["control_state"], node.STATE_ERROR)

    def test_read_config_uses_local_config_example_by_default(self):
        config = node.DuojiConfig.read_config()
        expected_path = os.path.join(os.path.dirname(node.__file__), "config", "example.json")

        self.assertEqual(config["arm_config"], expected_path)
        self.assertTrue(os.path.exists(config["arm_config"]))

    def test_worker_classes_exist(self):
        self.assertTrue(hasattr(node, "DuojiConfig"))
        self.assertTrue(hasattr(node, "RosPublisher"))
        self.assertTrue(hasattr(node, "Controller"))

    def test_duoji_config_loads_local_json(self):
        config_reader = node.DuojiConfig()

        self.assertTrue(os.path.exists(config_reader.values["arm_config"]))
        self.assertIsNotNone(config_reader.values["arm_params"])
        self.assertEqual(config_reader.values["arm_params"].dof, 3)

    def test_load_example_json_returns_arm_object(self):
        json_path = os.path.join(os.path.dirname(node.__file__), "config", "example.json")

        arm = node.DuojiConfig.load_arm_params_file(json_path)

        self.assertEqual(arm.dof, 3)
        self.assertEqual(len(arm.joints_length), 3)
        self.assertAlmostEqual(arm.joints_length[0], 0.18)
        self.assertAlmostEqual(arm.joints_length[1], 0.15)
        self.assertAlmostEqual(arm.joints_length[2], 0.10)

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

    # ===== 末端阻尼分发测试 =====

    def test_arm_parses_joint_types(self):
        """验证 Arm 对象包含 joint_types 字段且从 JSON 正确解析。"""
        config_reader = node.DuojiConfig()
        arm = config_reader.values["arm_params"]
        self.assertEqual(arm.joint_types, ["axial", "tangential", "tangential"])

    def test_build_arm_from_dict_with_joint_types(self):
        """验证 build_arm_from_dict 直接构建带 joint_types 的 Arm。"""
        raw = {
            "arm_name": "test",
            "dof": 4,
            "joints": {
                "joint1": {"type": "axial", "link": {"length": 0.2, "mass": 0.1}, "actuator_mass": 0.05},
                "joint2": {"type": "tangential", "link": {"length": 0.15, "mass": 0.1}, "actuator_mass": 0.05},
                "joint3": {"type": "tangential", "link": {"length": 0.10, "mass": 0.1}, "actuator_mass": 0.05},
                "joint4": {"type": "axial", "link": {"length": 0.08, "mass": 0.1}, "actuator_mass": 0.05},
            }
        }
        arm = node.DuojiConfig.build_arm_from_dict(raw)
        self.assertEqual(arm.joint_types, ["axial", "tangential", "tangential", "axial"])

    def test_build_arm_from_dict_default_tangential(self):
        """缺省 type 字段时默认为 tangential。"""
        raw = {
            "arm_name": "test",
            "dof": 2,
            "joints": {
                "joint1": {"link": {"length": 0.2, "mass": 0.1}, "actuator_mass": 0.05},
                "joint2": {"link": {"length": 0.1, "mass": 0.1}, "actuator_mass": 0.05},
            }
        }
        arm = node.DuojiConfig.build_arm_from_dict(raw)
        self.assertEqual(arm.joint_types, ["tangential", "tangential"])

    def test_classify_joints_3dof(self):
        """3-DOF: 1 axial + 2 tangential 分类。
        关节序列: axial(0.18) → tangential(0.15) → tangential(0.10)
        切向关节 0 (joint2): a = 0.15 (到下一个切向关节 joint3，中间无轴向)
        切向关节 1 (joint3): a = 0.10 (末尾)
        """
        arm = node.DuojiConfig().values["arm_params"]
        axial, tang, lengths = node.Controller.classify_joints(arm, [0, 1, 2])
        self.assertEqual(axial, [0])
        self.assertEqual(tang, [1, 2])
        self.assertEqual(lengths, [0.15, 0.10])

    def test_classify_joints_5dof_mixed(self):
        """5-DOF 混合构型分类，验证中间轴向杆长累加。
        关节序列: axial(0.18) → tangential(0.15) → tangential(0.10) → axial(0.08) → tangential(0.06)
        切向关节 0 (joint2, pos=1): a = 0.15 (到 joint3, pos=2, 中间无轴向)
        切向关节 1 (joint3, pos=2): a = 0.10 + 0.08 = 0.18 (到 joint5, pos=4, 中间有 axial joint4)
        切向关节 2 (joint5, pos=4): a = 0.06 (末尾)
        """
        arm = node.Arm(dof=5, joint_types=["axial", "tangential", "tangential", "axial", "tangential"],
                       joints_length=[0.18, 0.15, 0.10, 0.08, 0.06])
        axial, tang, lengths = node.Controller.classify_joints(arm, [0, 1, 2, 3, 4])
        self.assertEqual(axial, [0, 3])
        self.assertEqual(tang, [1, 2, 4])
        self.assertEqual(lengths, [0.15, 0.18, 0.06])

    def test_classify_joints_7dof(self):
        """7-DOF 分类，验证多段轴向杆长累加。
        关节序列: axial(0.20) → tang(0.18) → tang(0.15) → axial(0.12) → tang(0.10) → tang(0.08) → axial(0.06)
        切向关节 0 (pos=1): a = 0.18 (到 pos=2, 中间无轴向)
        切向关节 1 (pos=2): a = 0.15 + 0.12 = 0.27 (到 pos=4, 中间有 axial pos=3)
        切向关节 2 (pos=4): a = 0.10 (到 pos=5, 中间无轴向)
        切向关节 3 (pos=5): a = 0.08 + 0.06 = 0.14 (末尾, 后面有 axial pos=6)
        """
        arm = node.Arm(dof=7,
                       joint_types=["axial", "tangential", "tangential", "axial", "tangential", "tangential", "axial"],
                       joints_length=[0.2, 0.18, 0.15, 0.12, 0.10, 0.08, 0.06])
        axial, tang, lengths = node.Controller.classify_joints(arm, [0, 1, 2, 3, 4, 5, 6])
        self.assertEqual(axial, [0, 3, 6])
        self.assertEqual(tang, [1, 2, 4, 5])
        self.assertEqual(lengths, [0.18, 0.27, 0.10, 0.14])

    def test_build_dh_table_varies_with_n(self):
        """DH 表行数随切向关节数变化。"""
        self.assertEqual(len(node.Controller.build_dh_table([0.15])), 1)
        self.assertEqual(len(node.Controller.build_dh_table([0.15, 0.10])), 2)
        self.assertEqual(len(node.Controller.build_dh_table([0.15, 0.10, 0.08, 0.06])), 4)

    def test_forward_kinematics_zero_angles(self):
        """零角度时末端在 +x 方向，距离为连杆长度之和。"""
        dh = node.Controller.build_dh_table([0.15, 0.10])
        px, py, _ = node.Controller.forward_kinematics(dh)
        self.assertAlmostEqual(px, 0.25)
        self.assertAlmostEqual(py, 0.0)

    def test_forward_kinematics_90_degrees(self):
        """第一关节 90° 时末端在 +y 方向。"""
        dh = node.Controller.build_dh_table([0.15, 0.10])
        dh[0]["theta"] = math.pi / 2  # 90°
        dh[1]["theta"] = 0.0
        px, py, _ = node.Controller.forward_kinematics(dh)
        self.assertAlmostEqual(px, 0.0, places=10)
        self.assertAlmostEqual(py, 0.25, places=10)

    def test_jacobian_shape_varies(self):
        """雅可比维度自适应：2 行 N 列。"""
        for n in [2, 3, 4]:
            dh = node.Controller.build_dh_table([0.1] * n)
            _, _, cum = node.Controller.forward_kinematics(dh)
            J = node.Controller.compute_jacobian(dh, cum)
            self.assertEqual(len(J), 2)
            self.assertEqual(len(J[0]), n)

    def test_distribute_damping_budget_conservation(self):
        """分发后总功率 = end_damping（预算守恒）。"""
        dh = node.Controller.build_dh_table([0.15, 0.10])
        dh[0]["theta"] = 0.5
        dh[1]["theta"] = 0.3
        _, _, cum = node.Controller.forward_kinematics(dh)
        J = node.Controller.compute_jacobian(dh, cum)
        powers = node.Controller.distribute_damping(J, 1000)
        self.assertAlmostEqual(sum(powers), 1000, places=2)
        self.assertEqual(len(powers), 2)

    def test_distribute_damping_base_joint_gets_more(self):
        """基关节（杠杆高）分到更多功率。"""
        dh = node.Controller.build_dh_table([0.15, 0.10])
        dh[0]["theta"] = 0.0
        dh[1]["theta"] = 0.0
        _, _, cum = node.Controller.forward_kinematics(dh)
        J = node.Controller.compute_jacobian(dh, cum)
        powers = node.Controller.distribute_damping(J, 1000)
        self.assertGreater(powers[0], powers[1])

    def test_distribute_damping_n4(self):
        """4 切向关节的分发：预算守恒，长度正确。"""
        dh = node.Controller.build_dh_table([0.15, 0.10, 0.08, 0.06])
        for i in range(4):
            dh[i]["theta"] = 0.3 * i
        _, _, cum = node.Controller.forward_kinematics(dh)
        J = node.Controller.compute_jacobian(dh, cum)
        powers = node.Controller.distribute_damping(J, 1000)
        self.assertEqual(len(powers), 4)
        self.assertAlmostEqual(sum(powers), 1000, places=1)

    def test_distribute_damping_singular_uniform(self):
        """奇异位姿退化到均匀分配。"""
        dh = node.Controller.build_dh_table([1e-10, 1e-10])
        _, _, cum = node.Controller.forward_kinematics(dh)
        J = node.Controller.compute_jacobian(dh, cum)
        powers = node.Controller.distribute_damping(J, 1000)
        self.assertEqual(len(powers), 2)
        self.assertAlmostEqual(powers[0], powers[1], places=5)

    def test_damping_config_fields_in_to_dict(self):
        """验证 to_dict 导出了新增的阻尼配置字段。"""
        config = node.DuojiConfig.read_config(end_damping=1000, base_damping_power=500, max_damping_power=1000)
        self.assertEqual(config["end_damping"], 1000)
        self.assertEqual(config["base_damping_power"], 500)
        self.assertEqual(config["max_damping_power"], 1000)


if __name__ == "__main__":
    unittest.main()
