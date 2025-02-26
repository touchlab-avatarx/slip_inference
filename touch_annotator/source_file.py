#!/usr/bin/env python3

# Copyright (c) 2025 Touchlab Limited. All Rights Reserved
# Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

from bag_parser import RosBagParser


if __name__ == "__main__":
    # Specify the path to your ROS2 bag file

    bag_file_path = "/home/container/ros2/src/rig_controller/rig_controller/bags/rig1_bag1_iteration_2_20250212_122343"

    # Create an instance of the parser
    parser = RosBagParser(bag_path=bag_file_path)

    # all these topic have same time stamp
    topic_of_interest = [
        "/robotiq/frames_transformer/wrenches",
        "/robotiq/frames_transformer/data_raw",
        "/robotiq/frames_transformer/frames",
        # "/robotiq/frames_transformer/point_force",
        # "/robotiq/joint_states",
    ]

    try:
        # Parse the bag file without specifying topics (parses all topics)
        parser.parse(
            topics_of_interest=topic_of_interest
        )

        # Print a summary of parsed topics
        parser.print_summary()

        # List available topics from metadata
        available_topics = parser.get_available_topics()
        print(f"\nAvailable topics in the bag file: {available_topics}")

        parsed_message = {
            target_topic: parser.get_topic_messages(target_topic)
            for target_topic in topic_of_interest
        }

        pass

    except FileNotFoundError as e:
        print(e)