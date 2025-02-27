import numpy as np


class RosMsgToNumpyConverter:
    def __init__(self, recorded_messages):
        """
        Initialize the converter with recorded ROS 2 messages.

        :param recorded_messages: Dictionary with topic names as keys and lists of messages as values.
                                  Example: {"/topic_name": [msg1, msg2, ...]}
        """
        self.recorded_messages = recorded_messages
        self.numpy_data = {}  # Stores converted numpy arrays per topic
        self.timestamps = []  # Stores synchronized timestamps

    def extract_timestamp(self, msg):
        """Extract timestamp from ROS2 message header."""
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return None

    def message_to_numpy(self, msg):
        """Convert different ROS 2 message types to NumPy arrays."""
        if hasattr(msg, "wrenches"):  # geometry_msgs/Wrench[]
            return np.array(
                [
                    [
                        w.force.x, w.force.y, w.force.z,
                        w.torque.x, w.torque.y, w.torque.z
                    ]
                    for w in msg.wrenches
                ],
                dtype=np.float64
            )

        elif hasattr(msg, "multi_array"):  # std_msgs/Float64MultiArray
            return np.array(msg.multi_array.data, dtype=np.float64)  # Extract 1D array

        elif hasattr(msg, "poses"):  # geometry_msgs/PoseArray
            return np.array(
                [
                    [
                        pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                    ]
                    for pose in msg.poses
                ],
                dtype=np.float64
            )

        elif hasattr(msg, "position") and hasattr(msg, "velocity") and hasattr(msg, "effort"):
            # sensor_msgs/JointState
            return np.hstack((
                np.array(msg.position, dtype=np.float64),
                np.array(msg.velocity, dtype=np.float64),
                np.array(msg.effort, dtype=np.float64)
            ))

        return None  # Unsupported message type

    def convert_to_numpy(self):
        """
        Converts all stored ROS 2 messages into NumPy arrays and synchronizes timestamps.
        """
        all_timestamps = set()
        topic_data = {}

        # Extract timestamps and message data
        for topic, msg_list in self.recorded_messages.items():
            topic_numpy_data = []
            topic_timestamps = []

            for msg in msg_list:
                msg_timestamp = self.extract_timestamp(msg)

                if msg_timestamp is None:
                    continue  # Skip messages without a valid timestamp

                msg_np = self.message_to_numpy(msg)

                if msg_np is not None:
                    topic_numpy_data.append(msg_np)
                    topic_timestamps.append(msg_timestamp)

            if topic_numpy_data:
                topic_data[topic] = {
                    "data": np.array(topic_numpy_data, dtype=np.float64),
                    "timestamps": np.array(topic_timestamps, dtype=np.float64),
                }
                all_timestamps.update(topic_timestamps)

        # Create a common synchronized timestamp array
        if all_timestamps:
            self.timestamps = np.array(sorted(all_timestamps), dtype=np.float64)

        # Synchronize all topics to the common timestamps
        for topic, data_dict in topic_data.items():
            topic_timestamps = data_dict["timestamps"]
            topic_values = data_dict["data"]

            # Create an empty matrix with NaNs for padding
            aligned_data = np.full(
                (len(self.timestamps), *topic_values.shape[1:]), np.nan, dtype=np.float64
            )

            last_valid_index = 0

            for i, global_timestamp in enumerate(self.timestamps):
                # Find the closest timestamp in topic_timestamps
                if global_timestamp in topic_timestamps:
                    index = np.where(topic_timestamps == global_timestamp)[0][0]
                    last_valid_index = index  # Update last known value
                else:
                    index = last_valid_index  # Use last known value if no exact match

                aligned_data[i] = topic_values[index]

            self.numpy_data[topic] = aligned_data

        return self.numpy_data, self.timestamps

    def get_numpy_data(self):
        """Retrieve the converted NumPy arrays for each topic."""
        return self.numpy_data

    def get_timestamps(self):
        """Retrieve the synchronized timestamps."""
        return self.timestamps


# Example Usage
if __name__ == "__main__":

    bag_file_path = "/home/container/ros2/src/rig_controller/rig_controller/bags/rig1_bag1_iteration_1_20250212_122335"

    from rig_controller.bag_handler.bag_parser import RosBagParser

    # Create an instance of the parser
    parser = RosBagParser(bag_path=bag_file_path)

    topic_of_interest = [
        "/robotiq/frames_transformer/wrenches",
        "/robotiq/frames_transformer/data_raw",
        "/robotiq/frames_transformer/frames",
        "/robotiq/joint_states",
    ]

    # Parse the bag file without specifying topics (parses all topics)
    parser.parse(topics_of_interest=topic_of_interest)

    # Print a summary of parsed topics
    parser.print_summary()

    # List available topics from metadata
    available_topics = parser.get_available_topics()
    print(f"\n All available topics in the bag file: {available_topics}")

    # Access messages for a specific topic
    parsed_message = {
        topic: parser.get_topic_messages(topic)
        for topic in topic_of_interest
    }

    # Initialize converter with parsed messages
    converter = RosMsgToNumpyConverter(parsed_message)
    synchronized_numpy_data, timestamps = converter.convert_to_numpy()

    # Access synchronized data
    print("Synchronized timestamps:", timestamps)
    for topic, data in synchronized_numpy_data.items():
        print(f"Topic: {topic}, NumPy Shape: {data.shape}, Data Sample: {data[:2]}")
