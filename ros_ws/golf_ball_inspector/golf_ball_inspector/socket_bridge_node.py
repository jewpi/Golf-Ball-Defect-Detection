#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
socket_bridge_node — ROS2 토픽 → 라즈베리파이 TCP 브릿지 노드 (노트북 실행)

라즈베리파이에 ROS2가 없어도 기존 rpi_server.py를 그대로 사용할 수 있도록
/conveyor/cmd 토픽을 수신해서 TCP 소켓으로 전달합니다.

구독: /conveyor/cmd (std_msgs/String) → "START" / "BAD" / "STOP"
"""

import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RASPBERRY_PI_HOST = '192.168.110.143'
RASPBERRY_PI_PORT = 9999


class SocketBridgeNode(Node):

    def __init__(self):
        super().__init__('socket_bridge')

        self.declare_parameter('rpi_host', RASPBERRY_PI_HOST)
        self.declare_parameter('rpi_port', RASPBERRY_PI_PORT)

        host = self.get_parameter('rpi_host').get_parameter_value().string_value
        port = self.get_parameter('rpi_port').get_parameter_value().integer_value

        self._sock: socket.socket | None = None
        self._sock_lock = threading.Lock()
        self._host = host
        self._port = port

        self._connect()

        self.create_subscription(String, '/conveyor/cmd', self._on_cmd, 10)
        self.get_logger().info(f'[브릿지] 준비 완료 → {host}:{port}')

    def _connect(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((self._host, self._port))
            self._sock = s
            self.get_logger().info(f'[브릿지] 라즈베리파이 연결 완료 ({self._host}:{self._port})')
        except OSError as e:
            self._sock = None
            self.get_logger().error(f'[브릿지] 연결 실패: {e}')

    def _send(self, cmd: str):
        message = cmd.strip().upper() + '\n'
        with self._sock_lock:
            for attempt in range(2):
                try:
                    if self._sock is None:
                        raise OSError('소켓 없음')
                    self._sock.sendall(message.encode('utf-8'))
                    self.get_logger().info(f'[브릿지] 전송: {cmd.strip().upper()}')
                    return
                except OSError:
                    self.get_logger().warn(f'[브릿지] 재연결 시도 ({attempt + 1}/2)')
                    try:
                        if self._sock:
                            self._sock.close()
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.connect((self._host, self._port))
                        self._sock = s
                    except OSError as e:
                        self._sock = None
                        self.get_logger().error(f'[브릿지] 재연결 실패: {e}')

    def _on_cmd(self, msg: String):
        self._send(msg.data)

    def destroy_node(self):
        with self._sock_lock:
            if self._sock:
                self._sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SocketBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
