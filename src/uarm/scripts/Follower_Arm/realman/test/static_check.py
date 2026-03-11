#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealMan 遥操代码静态检查
检查语法、导入、ROS消息类型等
"""

import os
import sys
import ast
import importlib.util


class StaticChecker:
    """静态代码检查器"""
    
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.info = []
        
    def check_file(self, filepath):
        """检查单个文件"""
        print(f"\n{'='*60}")
        print(f"检查文件: {filepath}")
        print('='*60)
        
        if not os.path.exists(filepath):
            self.errors.append(f"文件不存在: {filepath}")
            return False
        
        # 检查1: Python语法
        if not self._check_syntax(filepath):
            return False
        
        # 检查2: 导入语句
        self._check_imports(filepath)
        
        # 检查3: ROS消息类型
        self._check_ros_messages(filepath)
        
        # 检查4: 代码规范
        self._check_code_style(filepath)
        
        return len(self.errors) == 0
    
    def _check_syntax(self, filepath):
        """检查Python语法"""
        try:
            with open(filepath, 'r') as f:
                source = f.read()
            ast.parse(source)
            self.info.append("✅ Python语法检查通过")
            return True
        except SyntaxError as e:
            self.errors.append(f"❌ 语法错误: {e}")
            return False
    
    def _check_imports(self, filepath):
        """检查导入"""
        required_imports = {
            'realman_teleop_safe.py': [
                'rospy', 'numpy', 'std_msgs.msg', 'rm_msgs.msg'
            ],
            'realman_safety_monitor.py': [
                'rospy', 'numpy', 'std_msgs.msg', 'rm_msgs.msg'
            ]
        }
        
        filename = os.path.basename(filepath)
        if filename not in required_imports:
            return
        
        with open(filepath, 'r') as f:
            source = f.read()
        
        tree = ast.parse(source)
        imports = []
        
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    imports.append(alias.name)
            elif isinstance(node, ast.ImportFrom):
                module = node.module or ''
                for alias in node.names:
                    imports.append(f"{module}.{alias.name}")
        
        required = required_imports[filename]
        for imp in required:
            if any(imp in i for i in imports):
                self.info.append(f"✅ 导入检查通过: {imp}")
            else:
                self.warnings.append(f"⚠️ 可能缺少导入: {imp}")
    
    def _check_ros_messages(self, filepath):
        """检查ROS消息类型"""
        msg_types = {
            'Arm_Current_State': 'rm_msgs.msg',
            'JointPos': 'rm_msgs.msg',
            'Gripper_Set': 'rm_msgs.msg',
            'Stop': 'rm_msgs.msg',
            'Float64MultiArray': 'std_msgs.msg',
            'Bool': 'std_msgs.msg',
        }
        
        with open(filepath, 'r') as f:
            content = f.read()
        
        for msg_type, pkg in msg_types.items():
            if msg_type in content:
                self.info.append(f"✅ 使用ROS消息: {pkg}.{msg_type}")
    
    def _check_code_style(self, filepath):
        """检查代码规范"""
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        # 检查文件头
        if not lines[0].startswith('#!'):
            self.warnings.append("⚠️ 缺少shebang行 (#!/usr/bin/env python3)")
        
        # 检查编码声明
        if 'utf-8' not in lines[1] if len(lines) > 1 else True:
            self.warnings.append("⚠️ 建议添加编码声明 (# -*- coding: utf-8 -*-)")
        
        # 检查行长度
        long_lines = []
        for i, line in enumerate(lines, 1):
            if len(line) > 120:
                long_lines.append(i)
        
        if long_lines:
            self.warnings.append(f"⚠️ 以下行超过120字符: {long_lines[:5]}...")
        
        # 检查TODO/FIXME
        todos = []
        for i, line in enumerate(lines, 1):
            if 'TODO' in line or 'FIXME' in line:
                todos.append((i, line.strip()))
        
        if todos:
            self.info.append(f"ℹ️ 发现 {len(todos)} 个TODO/FIXME标记")
    
    def print_report(self):
        """打印检查报告"""
        print(f"\n{'='*60}")
        print("检查报告")
        print('='*60)
        
        if self.errors:
            print(f"\n❌ 错误 ({len(self.errors)}):")
            for err in self.errors:
                print(f"  {err}")
        
        if self.warnings:
            print(f"\n⚠️  警告 ({len(self.warnings)}):")
            for warn in self.warnings:
                print(f"  {warn}")
        
        if self.info:
            print(f"\nℹ️  信息 ({len(self.info)}):")
            for info in self.info[:20]:  # 只显示前20条
                print(f"  {info}")
            if len(self.info) > 20:
                print(f"  ... 还有 {len(self.info)-20} 条")
        
        print(f"\n{'='*60}")
        if not self.errors:
            print("✅ 静态检查通过！")
        else:
            print(f"❌ 发现 {len(self.errors)} 个错误，需要修复")
        print('='*60)


def check_ros_environment():
    """检查ROS环境"""
    print("\n" + "="*60)
    print("ROS环境检查")
    print("="*60)
    
    issues = []
    
    # 检查ROS_PACKAGE_PATH
    ros_package_path = os.environ.get('ROS_PACKAGE_PATH', '')
    if not ros_package_path:
        issues.append("❌ ROS_PACKAGE_PATH 未设置")
    else:
        print(f"✅ ROS_PACKAGE_PATH: {ros_package_path}")
    
    # 检查Python路径
    python_path = os.environ.get('PYTHONPATH', '')
    if 'ros' not in python_path.lower():
        issues.append("⚠️ PYTHONPATH 可能未包含ROS路径")
    
    # 检查rm_msgs
    try:
        spec = importlib.util.find_spec('rm_msgs')
        if spec:
            print(f"✅ rm_msgs 包已找到: {spec.origin}")
        else:
            issues.append("❌ rm_msgs 包未找到，需要source工作空间")
    except Exception as e:
        issues.append(f"⚠️ 无法检查rm_msgs: {e}")
    
    if issues:
        print("\n发现问题:")
        for issue in issues:
            print(f"  {issue}")
    else:
        print("✅ ROS环境检查通过")
    
    return len(issues) == 0


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='RealMan遥操代码静态检查')
    parser.add_argument('--files', nargs='+', help='要检查的文件')
    parser.add_argument('--ros', action='store_true', help='检查ROS环境')
    args = parser.parse_args()
    
    print("="*60)
    print("RealMan 遥操代码静态检查工具")
    print("="*60)
    
    # 检查ROS环境
    if args.ros or not args.files:
        ros_ok = check_ros_environment()
    
    # 检查文件
    if args.files:
        checker = StaticChecker()
        all_passed = True
        
        for filepath in args.files:
            if not checker.check_file(filepath):
                all_passed = False
        
        checker.print_report()
        
        return 0 if all_passed else 1
    
    # 默认检查当前目录下的主要文件
    elif not args.ros:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(script_dir)
        
        files_to_check = [
            os.path.join(parent_dir, 'realman_teleop_safe.py'),
            os.path.join(parent_dir, 'realman_safety_monitor.py'),
        ]
        
        checker = StaticChecker()
        all_passed = True
        
        for filepath in files_to_check:
            if os.path.exists(filepath):
                if not checker.check_file(filepath):
                    all_passed = False
            else:
                print(f"⚠️  文件不存在: {filepath}")
        
        checker.print_report()
        return 0 if all_passed else 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
