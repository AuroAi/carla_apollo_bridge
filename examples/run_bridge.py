#!/usr/bin/env python
import argparse
import os
import yaml

# have to import cyber first or debug output is buggy
from cyber_py import cyber
import carla

from carla_cyber_bridge.bridge import CarlaCyberBridge

if __name__ == '__main__':
    DESCRIPTION = "Sample python script to run carla-cyber bridge."

    PARSER = argparse.ArgumentParser(description=DESCRIPTION,
        formatter_class=argparse.RawTextHelpFormatter)

    PARSER.add_argument('--host', default='172.17.0.1',
        help='IP address of the host server (default: 172.17.0.1)')
    PARSER.add_argument('--port', default='2000',
        help='TCP port to listen on (default: 2000)')
    ARGS = PARSER.parse_args()

    timeout = 3
    client = carla.Client(ARGS.host, int(ARGS.port))
    client.set_timeout(timeout)

    script_path = os.path.dirname(os.path.realpath(__file__))
    params = yaml.safe_load(open(script_path + "/../config/settings.yaml"))["carla"]
    bridge = CarlaCyberBridge(carla_world=client.get_world(), params=params)
    bridge.run()
    