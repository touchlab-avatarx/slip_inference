import rclpy
from rclpy.node import Node
from touchlab_msgs.msg import Float64MultiArrayStamped
import numpy as np
from collections import deque
import torch


class SlipInferenceNode(Node):
    def __init__(self):
        super().__init__("slip_inference_node")
        self.subscription = self.create_subscription(
            Float64MultiArrayStamped,
            "/robotiq/frames_transformer/data_raw",
            self.inference_callback,
            10,
        )

        # Load the PyTorch model
        self.model = torch.jit.load("/home/container/ros2/src/slip_inference/models/slipv1_2025-04-01_15-10-50.pt")
        self.model.eval()

        # Define sliding window_size
        # window_size should be exactly same as the as one used to  train model with
        self.window_size = 16  # rows
        self.feature_size = 32  # cols (from message)
        self.data_queue = deque(maxlen=self.window_size)

        self.get_logger().info("Slip Inference Node started.")

    def inference_callback(self, msg):
        # Convert incoming message (length 32) to numpy array
        data_np = np.array(msg.multi_array.data)  # Shape: (32,)

        # Sanity check for size
        if data_np.shape[0] != self.feature_size:
            self.get_logger().warn(f"Unexpected input shape: {data_np.shape}")
            return

        # Add new vector to sliding window
        self.data_queue.append(data_np)  # Each entry is shape (32,)

        # Once we have 16 rows, form the (16, 32) input
        if len(self.data_queue) == self.window_size:
            # Convert queue to numpy array of shape (16, 32)
            input_data = np.stack(self.data_queue, axis=0)
            self.run_inference(input_data)


    def run_inference(self, input_data):
        """
        input_data: np.ndarray of shape (16, 32)
        Output: model inference on stacked (32, 32) array of [fft; input_data]
        """

        # Step 1: Center the window (mean subtraction)
        window_centered = input_data - np.mean(input_data, axis=0, keepdims=True)

        # Step 2: Apply Hann window along the time axis
        hann = np.hanning(window_centered.shape[0])[:, np.newaxis]  # Shape: (16, 1)
        window_hann = window_centered * hann

        # Step 3: Compute FFT magnitude
        fft_output = np.abs(np.fft.fft(window_hann, axis=0)) / input_data.shape[0]  # Shape: (16, 32)

        # Step 4: Stack FFT output and original input vertically
        input_combined = np.vstack([fft_output, input_data])  # Shape: (32, 32)

        # Step 5: Convert to tensor and run inference
        input_tensor = torch.tensor(input_combined, dtype=torch.float32).unsqueeze(0).unsqueeze(0)  # Shape: (1, 32, 32)

        with torch.no_grad():
            output = self.model(input_tensor)
            probs = torch.softmax(output, dim=1)  # convert to probabilities
            predicted_class = probs.argmax(dim=1)
            class_map = {0: "touch", 1: "slip", 2: "no_touch"}
            label = class_map[predicted_class.item()]

        self.get_logger().info(f"Inference Output: {label}")


def main(args=None):
    rclpy.init(args=args)
    node = SlipInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
