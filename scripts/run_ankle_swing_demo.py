import argparse
import sys
import time

from g1_playground.dds import create_dds_topic_to_communicate_with_g1
from g1_playground.robot import UnitreeG1Robot
from g1_playground.policies.ankle_swing import AnkleSwingPolicy


def main():
    parser = argparse.ArgumentParser(description="Run the ankle swing demo policy on the G1 robot.")
    parser.add_argument("network_interface", nargs="?", default=None,
                        help="Network interface for real robot or MuJoCo sim (e.g. enp2s0, lo)")
    parser.add_argument("--dds_channel_id", type=int, default=1,
                        help="DDS domain channel ID (default: 1 for sim, use 0 for real robot)")
    args = parser.parse_args()

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    try:
        input("Press Enter to continue...")
    except KeyboardInterrupt:
        print("\nAborted before start.")
        return

    create_dds_topic_to_communicate_with_g1(args.dds_channel_id, args.network_interface)

    policy = AnkleSwingPolicy()
    robot = UnitreeG1Robot(policy=policy)

    try:
        robot.initialize()
        robot.start()
        print("Running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
        raise
    finally:
        robot.stop()
        print("Robot stopped. Exiting.")


if __name__ == "__main__":
    main()
    sys.exit(0)