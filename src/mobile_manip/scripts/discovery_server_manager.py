#!/usr/bin/env python3

import os
import subprocess

import rclpy
from rclpy.node import Node


class DiscoveryServerManager(Node):
    def __init__(self) -> None:
        super().__init__('discovery_server_manager')

        self.declare_parameter('robot_ip', '')
        self.declare_parameter('config_path', '/tmp/super_client_generated.xml')
        self.declare_parameter('server_port', 11811)
        self.declare_parameter(
            'server_prefix',
            '44.53.00.5f.45.50.52.4f.53.49.4d.41',
        )
        self.declare_parameter('stop_ros_daemon', True)

        robot_ip = self.get_parameter('robot_ip').value
        config_path = self.get_parameter('config_path').value
        server_port = int(self.get_parameter('server_port').value)
        server_prefix = self.get_parameter('server_prefix').value
        stop_ros_daemon = bool(self.get_parameter('stop_ros_daemon').value)

        if not robot_ip:
            raise ValueError(
                "Missing required parameter 'robot_ip'. Example: "
                "ros2 run mobile_manip discovery_server_manager --ros-args -p robot_ip:=192.168.1.54"
            )

        xml_content = self._build_xml(robot_ip, server_port, server_prefix)
        self._write_config(config_path, xml_content)
        self._set_environment(config_path, robot_ip, server_port)

        if stop_ros_daemon:
            self._stop_daemon()

        self.get_logger().info('------------------------------------------------')
        self.get_logger().info(f'Super Client configured for: {robot_ip}')
        self.get_logger().info(f'XML generated at: {config_path}')
        self.get_logger().info(f'ROS_DISCOVERY_SERVER={robot_ip}:{server_port}')
        self.get_logger().info('To export in your current shell, run:')
        self.get_logger().info(f'export FASTRTPS_DEFAULT_PROFILES_FILE={config_path}')
        self.get_logger().info(f'export ROS_DISCOVERY_SERVER={robot_ip}:{server_port}')
        self.get_logger().info('------------------------------------------------')

    @staticmethod
    def _build_xml(robot_ip: str, server_port: int, server_prefix: str) -> str:
        return f"""<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <participant profile_name="super_client_profile" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                        <discoveryServersList>
                            <RemoteServer prefix="{server_prefix}">
                                <metatrafficUnicastLocatorList>
                                    <locator>
                                        <udpv4>
                                            <address>{robot_ip}</address>
                                            <port>{server_port}</port>
                                        </udpv4>
                                    </locator>
                                </metatrafficUnicastLocatorList>
                            </RemoteServer>
                        </discoveryServersList>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
"""

    @staticmethod
    def _write_config(config_path: str, xml_content: str) -> None:
        with open(config_path, 'w', encoding='utf-8') as config_file:
            config_file.write(xml_content)

    @staticmethod
    def _set_environment(config_path: str, robot_ip: str, server_port: int) -> None:
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = config_path
        os.environ['ROS_DISCOVERY_SERVER'] = f'{robot_ip}:{server_port}'

    def _stop_daemon(self) -> None:
        result = subprocess.run(
            ['ros2', 'daemon', 'stop'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

        if result.returncode == 0:
            self.get_logger().info('ROS daemon reset.')
        else:
            self.get_logger().warning('ros2 daemon stop returned a non-zero status')


def main(args=None) -> int:
    rclpy.init(args=args)

    try:
        DiscoveryServerManager()
    except Exception as exc:
        rclpy.logging.get_logger('discovery_server_manager').fatal(str(exc))
        rclpy.shutdown()
        return 1

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
