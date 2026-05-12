import sys
import time

from g1_playground.dds import create_dds_topic_to_communicate_with_g1
from g1_playground.robot import UnitreeG1Robot
from g1_playground.policies.ankle_swing import AnkleSwingPolicy


def main():
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    try:
        input("Press Enter to continue...")
    except KeyboardInterrupt:
        print("\nAborted before start.")
        return

    create_dds_topic_to_communicate_with_g1(sys.argv)

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