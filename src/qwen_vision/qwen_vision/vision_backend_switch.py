#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Qwen / YOLO 感知后端切换器。

同时启动 YOLO 感知和 Qwen 感知时，由本节点保证同一时刻只有一个后端在识别，
并把上层通用的 /vision_perception/set_enabled 转发给当前后端。

接口：
  话题  /vision_backend (std_msgs/String)  发送 "yolo" 或 "qwen" 切换后端
  服务  /vision_perception/set_enabled     SetBool 开关当前后端
  服务  /vision_perception/backend         Trigger 查询当前后端
"""

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class VisionBackendSwitch(Node):
    def __init__(self):
        super().__init__('vision_backend_switch')

        self.declare_parameter('default_backend', 'qwen')
        self.declare_parameter('qwen_enable_service', '/qwen_perception/set_enabled')
        self.declare_parameter('yolo_enable_service', '/yolo_perception/set_enabled')
        self.declare_parameter('service_timeout', 30.0)

        backend = self.get_parameter('default_backend').value.lower().strip()
        self.backend = backend if backend in ('yolo', 'qwen') else 'qwen'
        self.qwen_enable_service = self.get_parameter('qwen_enable_service').value
        self.yolo_enable_service = self.get_parameter('yolo_enable_service').value
        self.service_timeout = float(self.get_parameter('service_timeout').value)

        self._ready = False
        self._activation_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._last_switch_fail_log = 0.0

        self.qwen_cli = self.create_client(SetBool, self.qwen_enable_service)
        self.yolo_cli = self.create_client(SetBool, self.yolo_enable_service)

        self.create_subscription(String, '/vision_backend', self._backend_cb, 1)
        self.create_service(
            SetBool, '/vision_perception/set_enabled', self._enable_cb)
        self.create_service(
            Trigger, '/vision_perception/backend', self._status_cb)

        self.create_timer(2.0, self._init_timer)
        self.get_logger().info(
            f'后端切换器已启动，当前后端: {self.backend}'
            f'（/vision_backend 可切换 yolo/qwen）')

    def _init_timer(self):
        if self._ready:
            return
        if not self._service_available(self._cli_for(self.backend)):
            now = time.monotonic()
            if now - self._last_switch_fail_log > 5.0:
                self._last_switch_fail_log = now
                self.get_logger().warn(
                    f'等待后端服务 {self._service_name(self.backend)} ...')
            return
        threading.Thread(
            target=self._activate, args=(self.backend, True), daemon=True).start()

    # ---------------- 接口 ----------------
    def _backend_cb(self, msg):
        name = (msg.data or '').strip().lower()
        if name not in ('yolo', 'qwen'):
            self.get_logger().warn(f'未知后端: {msg.data}（仅支持 yolo/qwen）')
            return
        with self._state_lock:
            if name == self.backend and self._ready:
                return
            self.backend = name
        self.get_logger().info(f'收到切换请求 -> {name}')
        threading.Thread(
            target=self._activate, args=(name, False), daemon=True).start()

    def _enable_cb(self, request, response):
        with self._state_lock:
            backend = self.backend
        threading.Thread(
            target=self._set_enabled_worker, args=(backend, bool(request.data)),
            daemon=True).start()
        response.success = True
        response.message = '指令已转发'
        return response

    def _status_cb(self, request, response):
        with self._state_lock:
            backend = self.backend
        response.success = True
        response.message = f'当前后端: {backend}（{self._ready}）'
        return response

    # ---------------- 内部逻辑 ----------------
    def _set_enabled_worker(self, backend, enabled):
        if enabled:
            self._activate(backend, initial=False)
        else:
            self._call_set(self._cli_for(backend), False)

    def _activate(self, backend, initial):
        with self._activation_lock:
            desired_cli = self._cli_for(backend)
            if not self._service_available(desired_cli, min(2.0, self.service_timeout)):
                now = time.monotonic()
                if now - self._last_switch_fail_log > 5.0:
                    self._last_switch_fail_log = now
                    self.get_logger().warn(
                        f'目标后端 {backend} 服务尚未就绪，暂不切换，稍后自动重试')
                self._ready = False
                return
            other = 'yolo' if backend == 'qwen' else 'qwen'
            # 先停另一个后端，保证同一时刻只有一个在发布目标
            self._call_set(self._cli_for(other), False, wait=1.0)
            ok = self._call_set(desired_cli, True, wait=self.service_timeout)
            if ok:
                self._ready = True
                self.get_logger().info(f'后端已切换并开启: {backend}')
            else:
                self._ready = False
                self.get_logger().error(
                    f'后端 {backend} 开启失败（服务不可用或超时），'
                    f'请确认对应感知节点已启动')

    def _service_name(self, backend):
        return self.qwen_enable_service if backend == 'qwen' else self.yolo_enable_service

    def _cli_for(self, backend):
        return self.qwen_cli if backend == 'qwen' else self.yolo_cli

    def _service_available(self, cli, wait=0.5):
        try:
            return cli.service_is_ready() or cli.wait_for_service(timeout_sec=wait)
        except Exception:
            return False

    def _call_set(self, cli, enabled, wait=5.0):
        """调用后端 set_enabled 服务并等待结果。返回是否成功。"""
        if cli is None:
            return False
        if not self._service_available(cli, min(wait, 2.0)):
            return False
        req = SetBool.Request()
        req.data = bool(enabled)
        try:
            future = cli.call_async(req)
            deadline = time.monotonic() + max(0.0, wait)
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done():
                return False
            resp = future.result()
            return bool(resp.success)
        except Exception as e:
            self.get_logger().error(f'调用 set_enabled 失败: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = VisionBackendSwitch()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
