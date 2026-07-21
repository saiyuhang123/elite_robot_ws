#!/usr/bin/env python3
"""手动输入坐标点测试机械臂（调用 grasp_move_server 的 move_to_pose 服务）。

用法：
  ros2 run hello_moveit manual_move_client.py            # 默认服务名
  ros2 run hello_moveit manual_move_client.py --service /grasp_move_server/move_to_pose

命令（角度单位为度，长度单位为米，基座系 cs66_base_link）：
  c                              打印当前末端位姿
  j j1 j2 j3 j4 j5 j6 [v] [a]    关节目标 MoveJ（6 个关节角，度）
  p x y z [r p y] [v] [a]        位姿目标 MoveJ；姿态省略时保持当前姿态
  l x y z [r p y] [v] [a]        位姿目标 MoveL（末端直线，仅适合短距离）
  q                              退出
  v / a 为速度/加速度缩放(0~1)，省略用服务端默认值
"""

import math
import sys
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from grasp_moveit_msgs.srv import MoveToPose

try:
    import tf2_ros
except ImportError:  # 没有 tf2 也能用，只是 c 命令和默认姿态不可用
    tf2_ros = None

BASE_FRAME = "cs66_base_link"
TIP_LINK = "cs66_tool0"


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def rpy_from_quat(x, y, z, w):
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class ManualMoveClient(Node):
    def __init__(self, service_name):
        super().__init__("manual_move_client")
        self.cli = self.create_client(MoveToPose, service_name)
        self.tf_buffer = tf2_ros.Buffer() if tf2_ros else None
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) if tf2_ros else None

    def current_pose(self):
        """返回 (x, y, z, roll, pitch, yaw)，查询失败返回 None。"""
        if not self.tf_buffer:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, TIP_LINK, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f"查询 TF 失败: {e}")
            return None
        p = t.transform.translation
        q = t.transform.rotation
        r, pi, y = rpy_from_quat(q.x, q.y, q.z, q.w)
        return p.x, p.y, p.z, r, pi, y

    def send(self, req):
        if not self.cli.wait_for_service(timeout_sec=2.0):
            print("服务不可用，请先启动 grasp_move_server 和 move_group")
            return
        future = self.cli.call_async(req)
        # 节点已在后台线程 spin，这里只需等 future 完成
        while rclpy.ok() and not future.done():
            threading.Event().wait(0.05)
        resp = future.result()
        if resp is None:
            print("调用失败（无响应）")
        elif resp.success:
            print(f"成功: {resp.message}")
        else:
            print(f"失败: {resp.message}")


def make_pose_req(motion_type, xyz, rpy, scaling):
    req = MoveToPose.Request()
    req.motion_type = motion_type
    req.target_pose = PoseStamped()
    req.target_pose.header.frame_id = BASE_FRAME
    req.target_pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    req.target_pose.pose.position.x, req.target_pose.pose.position.y, req.target_pose.pose.position.z = xyz
    qx, qy, qz, qw = quat_from_rpy(*[math.radians(a) for a in rpy])
    req.target_pose.pose.orientation.x = qx
    req.target_pose.pose.orientation.y = qy
    req.target_pose.pose.orientation.z = qz
    req.target_pose.pose.orientation.w = qw
    if scaling:
        req.velocity_scaling = scaling[0]
        if len(scaling) > 1:
            req.acceleration_scaling = scaling[1]
    return req


def main():
    service_name = "/grasp_move_server/move_to_pose"
    args = sys.argv[1:]
    if "--service" in args:
        i = args.index("--service")
        service_name = args[i + 1]

    rclpy.init()
    node = ManualMoveClient(service_name)
    # 后台线程持续 spin：等待输入时 TF 监听器和服务回调才能正常收发
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    print(__doc__)
    print(f"服务: {service_name}\n")

    try:
        while rclpy.ok():
            try:
                line = input("move> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            try:
                if cmd in ("q", "quit", "exit"):
                    break
                elif cmd == "c":
                    pose = node.current_pose()
                    if pose:
                        x, y, z, r, p, yw = pose
                        print(f"位置: [{x:.4f}, {y:.4f}, {z:.4f}]  "
                              f"RPY(deg): [{math.degrees(r):.1f}, {math.degrees(p):.1f}, {math.degrees(yw):.1f}]")
                elif cmd == "j":
                    vals = [float(v) for v in parts[1:]]
                    if len(vals) < 6:
                        print("用法: j j1 j2 j3 j4 j5 j6 [v] [a]")
                        continue
                    req = MoveToPose.Request()
                    req.motion_type = "movej"
                    req.joint_target = [math.radians(v) for v in vals[:6]]
                    if len(vals) > 6:
                        req.velocity_scaling = vals[6]
                    if len(vals) > 7:
                        req.acceleration_scaling = vals[7]
                    node.send(req)
                elif cmd in ("p", "l"):
                    vals = [float(v) for v in parts[1:]]
                    if len(vals) < 3:
                        print(f"用法: {cmd} x y z [r p y] [v] [a]")
                        continue
                    xyz = vals[:3]
                    rest = vals[3:]
                    if len(rest) >= 3:
                        rpy, scaling = rest[:3], rest[3:]
                    else:
                        cur = node.current_pose()
                        if not cur:
                            print("姿态省略时需要 TF 可用（或显式给出 r p y）")
                            continue
                        rpy = [math.degrees(a) for a in cur[3:]]
                        scaling = rest
                    node.send(make_pose_req("movej" if cmd == "p" else "movel", xyz, rpy, scaling))
                else:
                    print("未知命令，输入 c / j / p / l / q")
            except ValueError:
                print("参数格式错误，数字请用十进制")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
