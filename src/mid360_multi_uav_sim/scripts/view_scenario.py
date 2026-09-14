#!/usr/bin/env python3
"""Launch one benchmark with the Gazebo GUI enabled."""

import argparse
import os
import subprocess
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=["OPEN", "MT", "OFFICE", "FOREST", "P01", "P02"])
    parser.add_argument("--sensor", choices=["mid360", "ouster", "paired"], default="mid360")
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--wait-for-start", action="store_true")
    arguments = parser.parse_args()
    scenario_id = arguments.scenario

    package = Path(subprocess.check_output(
        ["rospack", "find", "mid360_multi_uav_sim"], text=True).strip())
    scenario = package / "config" / "benchmarks" / f"{scenario_id}.yaml"
    config = yaml.safe_load(scenario.read_text())
    world = package / "worlds" / f"{config['world']}.world"
    command = [
        "roslaunch", "mid360_multi_uav_sim", "benchmark.launch",
        f"world_file:={world}", f"scenario_file:={scenario}",
        f"vehicle_count:={1 + len(config['targets'])}", "gui:=true",
        f"sensor_model:={arguments.sensor}",
        f"mrs_custom_config:={package / 'config/mrs' / config.get('mrs_custom_config', 'paper_config.yaml')}",
        f"mrs_world_config:={package / 'config/mrs' / config.get('mrs_world_config', 'world_config.yaml')}",
        f"rviz:={'true' if arguments.rviz else 'false'}",
        f"wait_for_start:={'true' if arguments.wait_for_start else 'false'}",
    ]
    print("Launching", scenario_id, "from", config["world"], flush=True)
    os.execvp(command[0], command)


if __name__ == "__main__":
    main()
