import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from collections import defaultdict


class RosBagParser:
    def __init__(self, bag_path):
        """
        Initialize the parser with the bag file path.

        :param bag_path: Path to the ROS2 bag file.
        """
        self.bag_path = bag_path
        self.topic_data = defaultdict(list)
        self.available_topics = {}

    def validate_bag_file(self):
        """Check if the bag file exists."""
        if not os.path.exists(self.bag_path):
            raise FileNotFoundError(f"Bag file path '{self.bag_path}' does not exist.")

    def load_bag_metadata(self):
        """Load metadata from the bag file to determine available topics."""
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path, storage_id="sqlite3"
        )
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, rosbag2_py.ConverterOptions())
        metadata = reader.get_metadata()

        # Access topic name and type from TopicInformation
        self.available_topics = {
            topic_info.topic_metadata.name: topic_info.topic_metadata.type
            for topic_info in metadata.topics_with_message_count
        }

    def parse(self, topics_of_interest=None):
        """
        Parse the ROS2 bag file and extract messages.

        :param topics_of_interest: List of topics to parse (optional). If None, parses all topics.
        """
        self.validate_bag_file()
        self.load_bag_metadata()

        # If no specific topics are requested, parse all available topics
        if topics_of_interest is None:
            topics_of_interest = list(self.available_topics.keys())

        # Storage and converter options for reading
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path, storage_id="sqlite3"
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        # Read messages and store relevant ones
        while reader.has_next():
            topic_name, msg_data, timestamp = reader.read_next()

            # ignoring the serialize timestamp here
            if topic_name in topics_of_interest:
                msg_type = self.available_topics[topic_name]
                msg_class = get_message(msg_type)
                msg = deserialize_message(msg_data, msg_class)
                self.topic_data[topic_name].append(msg)

        print("Parsing complete. Topics and their respective message counts:")
        for topic, messages in self.topic_data.items():
            print(f"{topic}: {len(messages)} messages")

    def get_available_topics(self):
        """
        Retrieve the list of all topics available in the bag file.

        :return: List of topic names.
        """
        return list(self.available_topics.keys())

    def get_topic_messages(self, topic_name):
        """
        Retrieve all messages for a specific topic.

        :param topic_name: The topic name to retrieve messages for.
        :return: List of (message, timestamp) tuples for the topic.
        """
        return self.topic_data.get(topic_name, [])

    def print_summary(self):
        """Print a summary of parsed topics and their message counts."""
        print("Summary of parsed topics:")
        print(
            {
                topic: f"{len(messages)} messages"
                for topic, messages in self.topic_data.items()
            }
        )

