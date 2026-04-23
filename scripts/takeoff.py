import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

_DEFAULT_ROBOTS_YAML = Path(__file__).parents[1] / 'src/multiagent_simulation/config/robots.yaml'


def load_drone_names(path: Path) -> list[str]:
    with open(path) as f:
        return [r['name'] for r in yaml.safe_load(f)['robots']]


class TakeoffNode(Node):
    def __init__(self, drone_names: list[str], altitude: float):
        super().__init__('takeoff_node')
        self.drone_names = drone_names
        self.altitude = altitude

        self._drone_clients = {}
        for name in drone_names:
            self._drone_clients[name] = {
                'set_mode': self.create_client(SetMode, f'/{name}/mavros/set_mode'),
                'arming':   self.create_client(CommandBool, f'/{name}/mavros/cmd/arming'),
                'takeoff':  self.create_client(CommandTOL, f'/{name}/mavros/cmd/takeoff'),
            }

    def _wait(self, client, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return False
        return True

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def run(self):
        for name in self.drone_names:
            clients = self._drone_clients[name]
            self.get_logger().info(f'[{name}] Setting mode GUIDED')
            if not self._wait(clients['set_mode']):
                continue
            result = self._call(clients['set_mode'], SetMode.Request(custom_mode='GUIDED'))
            if not result or not result.mode_sent:
                self.get_logger().error(f'[{name}] Failed to set GUIDED mode')
                continue

            self.get_logger().info(f'[{name}] Arming')
            if not self._wait(clients['arming']):
                continue
            result = self._call(clients['arming'], CommandBool.Request(value=True))
            if not result or not result.success:
                self.get_logger().error(f'[{name}] Arming failed')
                continue

            # ArduPilot needs a moment after arming before accepting takeoff
            time.sleep(1.0)

            self.get_logger().info(f'[{name}] Taking off to {self.altitude} m')
            if not self._wait(clients['takeoff']):
                continue
            req = CommandTOL.Request(
                min_pitch=0.0,
                yaw=0.0,
                latitude=0.0,
                longitude=0.0,
                altitude=self.altitude,
            )
            result = self._call(clients['takeoff'], req)
            if not result or not result.success:
                self.get_logger().error(f'[{name}] Takeoff command rejected')
            else:
                self.get_logger().info(f'[{name}] Takeoff command accepted')


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Take off all drones via MAVROS')
    parser.add_argument('--drones', nargs='+', default=None,
                        help='Drone namespaces (default: read from robots.yaml)')
    parser.add_argument('--robots-yaml', type=Path, default=_DEFAULT_ROBOTS_YAML,
                        help=f'Path to robots.yaml (default: {_DEFAULT_ROBOTS_YAML})')
    parser.add_argument('--altitude', type=float, default=5.0,
                        help='Takeoff altitude in metres (default: 5.0)')
    args, ros_args = parser.parse_known_args()

    drones = args.drones or load_drone_names(args.robots_yaml)

    rclpy.init(args=ros_args or None)
    node = TakeoffNode(drones, args.altitude)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
