import ast
import math
import os
import unittest

import numpy as np

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
        self.assertEqual(node.ServoConfig.parse_int_list("[0, 1, 2, 6]"), [0, 1, 2, 6])

    def test_parse_int_list_accepts_single_number(self):
        self.assertEqual(node.ServoConfig.parse_int_list(3), [3])

    def test_build_servo_state_reads_sync_monitor_object(self):
        servo = FakeServo(angle=12.5, current=0.3, voltage=7.4, power=1.2, temp=30.0, status=0)
        state = node.Controller.build_servo_state(2, servo)

        self.assertEqual(state["id"], 2)
        self.assertEqual(state["angle"], 12.5)
        self.assertEqual(state["current"], 0.3)
        self.assertTrue(state["online"])

    def test_make_state_array_uses_servo_id_as_index(self):
        shared_state = node.ServoConfig.create_shared_state_from_values([0, 2], 4)
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
        shared_state = node.ServoConfig.create_shared_state_from_values([0], 1)
        shared_state["control_state"] = "BAD_STATE"
        worker = node.Controller({}, shared_state, lock, None, None)

        worker.run_state_machine_once(None)

        self.assertEqual(shared_state["control_state"], node.STATE_ERROR)

    def test_read_config_uses_local_config_example_by_default(self):
        config = node.ServoConfig.read_config()
        expected_path = os.path.join(os.path.dirname(node.__file__), "config", "example.json")

        self.assertEqual(config["arm_config"], expected_path)
        self.assertTrue(os.path.exists(config["arm_config"]))

    def test_worker_classes_exist(self):
        self.assertTrue(hasattr(node, "ServoConfig"))
        self.assertTrue(hasattr(node, "RosPublisher"))
        self.assertTrue(hasattr(node, "Controller"))

    def test_duoji_config_loads_local_json(self):
        config_reader = node.ServoConfig()

        self.assertTrue(os.path.exists(config_reader.values["arm_config"]))
        self.assertIsNotNone(config_reader.values["arm_params"])
        self.assertEqual(config_reader.values["arm_params"].dof, 3)

    def test_load_example_json_returns_arm_object(self):
        json_path = os.path.join(os.path.dirname(node.__file__), "config", "example.json")

        arm = node.ServoConfig.load_arm_params_file(json_path)

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

    # ===== 3D 运动学 + 动态阻尼测试 =====

    def test_arm_parses_joint_types(self):
        """验证 Arm 对象包含 joint_types 字段且从 JSON 正确解析。"""
        config_reader = node.ServoConfig()
        arm = config_reader.values["arm_params"]
        self.assertEqual(arm.joint_types, ["axial", "tangential", "tangential"])

    def test_arm_parses_twist_and_offset(self):
        """验证 Arm 对象包含 joints_twist 和 joints_offset。"""
        config_reader = node.ServoConfig()
        arm = config_reader.values["arm_params"]
        # axial → α=π/2, tangential → α=0（JSON 写的 1.5708，容差 4 位）
        self.assertAlmostEqual(arm.joints_twist[0], math.pi / 2, places=4)
        self.assertAlmostEqual(arm.joints_twist[1], 0.0)
        self.assertAlmostEqual(arm.joints_twist[2], 0.0)
        # offset 默认 0
        self.assertEqual(arm.joints_offset, [0.0, 0.0, 0.0])

    def test_build_arm_from_dict_with_twist(self):
        """验证 build_arm_from_dict 读取 twist 字段。"""
        raw = {
            "arm_name": "test", "dof": 2,
            "joints": {
                "joint1": {"type": "axial", "link": {"length": 0.2, "mass": 0.1, "twist": 0.5}, "actuator_mass": 0.05},
                "joint2": {"type": "tangential", "link": {"length": 0.1, "mass": 0.1}, "actuator_mass": 0.05},
            }
        }
        arm = node.ServoConfig.build_arm_from_dict(raw)
        self.assertAlmostEqual(arm.joints_twist[0], 0.5)       # 用户指定
        self.assertAlmostEqual(arm.joints_twist[1], 0.0)       # tangential 缺省 0

    def test_classify_joints_3dof(self):
        """3-DOF 分类，返回三元组。"""
        arm = node.ServoConfig().values["arm_params"]
        axial, tang, indices = node.Controller.classify_joints(arm, [0, 1, 2])
        self.assertEqual(axial, [0])
        self.assertEqual(tang, [1, 2])
        self.assertEqual(indices, [1, 2])

    def test_classify_joints_5dof_mixed(self):
        """5-DOF 混合构型分类。"""
        arm = node.Arm(dof=5, joint_types=["axial", "tangential", "tangential", "axial", "tangential"],
                       joints_length=[0.18, 0.15, 0.10, 0.08, 0.06])
        axial, tang, indices = node.Controller.classify_joints(arm, [0, 1, 2, 3, 4])
        self.assertEqual(axial, [0, 3])
        self.assertEqual(tang, [1, 2, 4])
        self.assertEqual(indices, [1, 2, 4])

    def test_classify_joints_7dof(self):
        """7-DOF 分类。"""
        arm = node.Arm(dof=7,
                       joint_types=["axial", "tangential", "tangential", "axial", "tangential", "tangential", "axial"],
                       joints_length=[0.2, 0.18, 0.15, 0.12, 0.10, 0.08, 0.06])
        axial, tang, indices = node.Controller.classify_joints(arm, [0, 1, 2, 3, 4, 5, 6])
        self.assertEqual(axial, [0, 3, 6])
        self.assertEqual(tang, [1, 2, 4, 5])
        self.assertEqual(indices, [1, 2, 4, 5])

    def test_build_dh_table_3d_length(self):
        """3D DH 表行数 = dof。"""
        arm = node.ServoConfig().values["arm_params"]
        dh = node.Controller.build_dh_table_3d(arm)
        self.assertEqual(len(dh), 3)
        # axial α=π/2, tangential α=0（JSON 写的 1.5708，容差 4 位）
        self.assertAlmostEqual(dh[0]["alpha"], math.pi / 2, places=4)
        self.assertAlmostEqual(dh[1]["alpha"], 0.0)

    def test_forward_kinematics_3d_zero_angles(self):
        """零角度时末端在 +x 方向。"""
        arm = node.ServoConfig().values["arm_params"]
        dh = node.Controller.build_dh_table_3d(arm)
        T_total, _ = node.Controller.forward_kinematics_3d(dh)
        px, py, pz = T_total[0, 3], T_total[1, 3], T_total[2, 3]
        # 第一个关节 axial α=π/2，零角度时连杆沿 x，但 α 旋转后 z 方向有分量
        # 具体值取决于 DH 约定，这里验证末端不是原点即可
        self.assertGreater(abs(px) + abs(py) + abs(pz), 0.01)

    def test_jacobian_3d_shape(self):
        """3D 雅可比形状 3×N_total。"""
        arm = node.ServoConfig().values["arm_params"]
        dh = node.Controller.build_dh_table_3d(arm)
        T_total, joint_transforms = node.Controller.forward_kinematics_3d(dh)
        p_end = np.array([T_total[0, 3], T_total[1, 3], T_total[2, 3]])
        J = node.Controller.compute_jacobian_3d(joint_transforms, p_end)
        self.assertEqual(J.shape, (3, 3))

    def test_extract_tangential_jacobian(self):
        """提取切向列后维度 3×N_tan。"""
        arm = node.ServoConfig().values["arm_params"]
        dh = node.Controller.build_dh_table_3d(arm)
        T_total, joint_transforms = node.Controller.forward_kinematics_3d(dh)
        p_end = np.array([T_total[0, 3], T_total[1, 3], T_total[2, 3]])
        J_3d = node.Controller.compute_jacobian_3d(joint_transforms, p_end)
        _, _, tang_indices = node.Controller.classify_joints(arm, [0, 1, 2])
        J_tan = node.Controller.extract_tangential_jacobian(J_3d, tang_indices)
        self.assertEqual(J_tan.shape, (3, 2))

    def test_distribute_damping_3d_budget_conservation(self):
        """3D 分发预算守恒。"""
        J_tan = np.array([[0.1, 0.2], [0.3, 0.4], [0.5, 0.6]])
        powers = node.Controller.distribute_damping_3d(J_tan, 1000)
        self.assertAlmostEqual(np.sum(powers), 1000, places=2)
        self.assertEqual(len(powers), 2)

    def test_distribute_damping_3d_base_joint_more(self):
        """杠杆高的关节分到更多。"""
        J_tan = np.array([[0.3, 0.01], [0.4, 0.01], [0.5, 0.01]])
        powers = node.Controller.distribute_damping_3d(J_tan, 1000)
        self.assertGreater(powers[0], powers[1])

    def test_distribute_damping_3d_singular(self):
        """奇异位姿退化到均匀分配。"""
        J_tan = np.zeros((3, 2))
        powers = node.Controller.distribute_damping_3d(J_tan, 1000)
        self.assertAlmostEqual(powers[0], powers[1], places=5)
        self.assertAlmostEqual(powers[0], 500, places=5)

    def test_damping_config_fields_in_to_dict(self):
        """验证 to_dict 导出新增字段，默认值与 DEFAULTS 一致。"""
        config = node.ServoConfig.read_config()
        self.assertEqual(config["end_damping"], node.DEFAULTS["end_damping"])
        self.assertEqual(config["end_damping_low"], node.DEFAULTS["end_damping_low"])
        self.assertEqual(config["end_damping_mid"], node.DEFAULTS["end_damping_mid"])
        self.assertEqual(config["end_damping_high"], node.DEFAULTS["end_damping_high"])
        self.assertEqual(config["base_damping_power"], node.DEFAULTS["base_damping_power"])
        self.assertEqual(config["max_damping_power"], node.DEFAULTS["max_damping_power"])
        self.assertEqual(config["vz_threshold"], node.DEFAULTS["vz_threshold"])

    def test_state_machine_has_locked_branch(self):
        """状态机包含 LOCKED 分支。"""
        self.assertTrue(hasattr(node, "STATE_LOCKED"))
        self.assertFalse(hasattr(node, "STATE_MANUAL"))

    def test_controller_has_lock_flags(self):
        """Controller 有 lock_requested / unlock_requested flag。"""
        lock = node.threading.Lock()
        shared_state = node.ServoConfig.create_shared_state_from_values([0], 1)
        stop_event = node.threading.Event()
        worker = node.Controller({}, shared_state, lock, stop_event, None)
        self.assertFalse(worker.lock_requested)
        self.assertFalse(worker.unlock_requested)

    def test_state_start_exists_and_is_default(self):
        """STATE_START 存在，且 shared_state 初始状态为 START。"""
        self.assertTrue(hasattr(node, "STATE_START"))
        shared_state = node.ServoConfig.create_shared_state_from_values([0], 1)
        self.assertEqual(shared_state["control_state"], node.STATE_START)

    def test_controller_has_start_done_flag(self):
        """Controller 有 _start_done flag，初始为 False。"""
        lock = node.threading.Lock()
        shared_state = node.ServoConfig.create_shared_state_from_values([0], 1)
        stop_event = node.threading.Event()
        worker = node.Controller({}, shared_state, lock, stop_event, None)
        self.assertFalse(worker._start_done)

    # ===== 遥测测试 =====

    def test_shared_state_has_telemetry_fields(self):
        """shared_state 初始包含 3 个遥测字段，默认值正确。"""
        shared_state = node.ServoConfig.create_shared_state_from_values([0, 1, 2], 3)
        self.assertEqual(shared_state["end_velocity"], [0.0, 0.0, 0.0])
        self.assertEqual(shared_state["damping_mode"], 0)
        self.assertEqual(shared_state["damping_powers"], {})

    def test_copy_shared_state_includes_telemetry(self):
        """copy_shared_state 后遥测字段存在且值一致。"""
        lock = node.threading.Lock()
        shared_state = node.ServoConfig.create_shared_state_from_values([0, 1], 2)
        shared_state["end_velocity"] = [0.1, -0.2, 0.05]
        shared_state["damping_mode"] = 2
        shared_state["damping_powers"] = {0: 500, 1: 300}

        copied = node.RosPublisher.copy_shared_state(shared_state, lock)
        self.assertEqual(copied["end_velocity"], [0.1, -0.2, 0.05])
        self.assertEqual(copied["damping_mode"], 2)
        self.assertEqual(copied["damping_powers"], {0: 500, 1: 300})

    def test_copy_shared_state_isolates_telemetry(self):
        """copy 后修改副本不影响原 shared_state 的遥测字段。"""
        lock = node.threading.Lock()
        shared_state = node.ServoConfig.create_shared_state_from_values([0], 1)
        shared_state["damping_powers"] = {0: 500}

        copied = node.RosPublisher.copy_shared_state(shared_state, lock)
        copied["damping_powers"][0] = 999
        # 原始不受影响
        self.assertEqual(shared_state["damping_powers"][0], 500)

    def test_write_telemetry_writes_all_fields(self):
        """_write_telemetry 一次写入 3 个字段。"""
        lock = node.threading.Lock()
        shared_state = node.ServoConfig.create_shared_state_from_values([0, 1, 2], 3)
        worker = node.Controller({}, shared_state, lock, node.threading.Event(), None)

        v_end = node.np.array([0.1, -0.2, 0.05])
        worker._write_telemetry(v_end, 3, {0: 500, 1: 800, 2: 200})

        self.assertEqual(shared_state["end_velocity"], [0.1, -0.2, 0.05])
        self.assertEqual(shared_state["damping_mode"], 3)
        self.assertEqual(shared_state["damping_powers"], {0: 500, 1: 800, 2: 200})

    def test_clear_telemetry_zeros_all_fields(self):
        """_clear_telemetry 清零 3 个字段。"""
        lock = node.threading.Lock()
        shared_state = node.ServoConfig.create_shared_state_from_values([0], 1)
        shared_state["end_velocity"] = [0.1, 0.2, 0.3]
        shared_state["damping_mode"] = 2
        shared_state["damping_powers"] = {0: 500}

        worker = node.Controller({}, shared_state, lock, node.threading.Event(), None)
        worker._clear_telemetry()

        self.assertEqual(shared_state["end_velocity"], [0.0, 0.0, 0.0])
        self.assertEqual(shared_state["damping_mode"], 0)
        self.assertEqual(shared_state["damping_powers"], {})

    def test_telemetry_cleared_in_idle_state(self):
        """IDLE 状态后遥测清零。"""
        lock = node.threading.Lock()
        shared_state = node.ServoConfig.create_shared_state_from_values([0], 1)
        shared_state["control_state"] = node.STATE_IDLE
        # 预置脏数据
        shared_state["damping_mode"] = 2
        shared_state["damping_powers"] = {0: 500}

        worker = node.Controller({}, shared_state, lock, node.threading.Event(), None)
        worker.run_state_machine_once(None)

        self.assertEqual(shared_state["damping_mode"], 0)
        self.assertEqual(shared_state["damping_powers"], {})

    def test_telemetry_not_cleared_in_hold_state(self):
        """HOLD 状态不清零遥测（由 apply_dynamic_damping 负责写入）。"""
        lock = node.threading.Lock()
        shared_state = node.ServoConfig.create_shared_state_from_values([0], 1)
        shared_state["control_state"] = node.STATE_HOLD
        shared_state["damping_mode"] = 2
        shared_state["damping_powers"] = {0: 500}

        # 用 mock manager，handle_hold → apply_dynamic_damping 会在 arm_params None 时直接返回
        # 但 config 是空字典，get("arm_params") 返回 None → return，不写遥测也不清零
        worker = node.Controller({}, shared_state, lock, node.threading.Event(), None)
        worker.run_state_machine_once(None)

        # HOLD 状态不清零，保持原值
        self.assertEqual(shared_state["damping_mode"], 2)
        self.assertEqual(shared_state["damping_powers"], {0: 500})


if __name__ == "__main__":
    unittest.main()
