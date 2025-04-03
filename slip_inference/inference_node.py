import rclpy
from rclpy.node import Node
from touchlab_msgs.msg import Float64MultiArrayStamped
import numpy as np
from collections import deque
import torch
from std_msgs.msg import Int32
import tkinter as tk
from tkinter import font as tkfont


class SlipInferenceNode(Node):
    def __init__(self):
        super().__init__("slip_inference_node")
        self.subscription = self.create_subscription(
            Float64MultiArrayStamped,
            "/robotiq/frames_transformer/data_raw",
            self.inference_callback,
            10,
        )

        self.publisher = self.create_publisher(Int32, 'slip_state', 10)

        # Initialize Tkinter GUI
        self.root = tk.Tk()
        self.gui = SlipGUI(self.root, self)
        
        # Create a timer for GUI updates
        self.gui_timer = self.create_timer(0.1, self.gui.update_gui)

        # Load the PyTorch model
        self.model = torch.jit.load("/home/container/ros2/src/slip_inference/models/slipv1_2025-04-01_15-10-50.pt")
        self.model.eval()

        # Define sliding window_size
        # window_size should be exactly same as the as one used to  train model with
        self.window_size = 16  # rows
        self.feature_size = 32  # cols (from message)
        self.data_queue = deque(maxlen=self.window_size)

        self.get_logger().info("Slip Inference Node started.")

        # self.root = tk.Tk()
        # self.app = SlipGUI(self.root)

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

        self.get_logger().info(f"Inference Output: {predicted_class.item()}")
        msg = Int32()
        msg.data = predicted_class.item()
        self.publisher.publish(msg)

class SlipGUI(Node):
    def __init__(self, root, node):

        self.root = root
        self.node = node
        self.root.title("SLIP DETECTION")

        self.subscription = self.node.create_subscription(
            Int32,
            "slip_state",
            self.update_state,
            10,
        )
        
        # Fullscreen configuration
        self.fullscreen = False
        self.root.attributes('-fullscreen', self.fullscreen)

        # Configure styles
        self.label_font = tkfont.Font(family='Helvetica', size=50, weight='bold')
        self.heading_font = tkfont.Font(family='Helvetica', size=20, weight='bold')
        self.active_color = "green"
        self.inactive_color = "#c30010"

        # Create GUI elements
        self.setup_gui()

        # State variables
        self.current_state = None
        self.last_update_time = self.node.get_clock().now()
        self.timeout = 1.0  # seconds before reverting to inactive

    def setup_gui(self):
        """Initialize all GUI components"""
        # Main container
        self.main_container = tk.Frame(self.root)
        self.main_container.pack(expand=True, fill=tk.BOTH)

        # Heading frame
        self.heading_frame = tk.Frame(self.main_container, bg="#333333")
        self.heading_frame.pack(fill=tk.X)

        self.heading_label = tk.Label(self.heading_frame,
                                    text="SLIP DETECTION INFERENCE",
                                    font=self.heading_font,
                                    bg="#333333",
                                    fg="white",
                                    pady=15)
        self.heading_label.pack(side=tk.LEFT, expand=True)

        # Sensor labels frame
        self.center_frame = tk.Frame(self.main_container)
        self.center_frame.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)

        self.sensor_container = tk.Frame(self.center_frame)
        self.sensor_container.pack(expand=True)

        # Label configuration
        label_config = {
            'font': self.label_font,
            'fg': "black",
            'width': 15,
            'height': 10,
            'highlightthickness': 5,  # Border thickness
            'highlightbackground': "black",  # Border color
            'highlightcolor': "white",
        }

        # Create labels
        self.slip_label = tk.Label(self.sensor_container, text="SLIP",
                                 bg=self.inactive_color, **label_config)
        self.slip_label.grid(row=0, column=0, padx=40, pady=30, sticky="nsew")

        self.touch_label = tk.Label(self.sensor_container, text="TOUCH",
                                  bg=self.inactive_color, **label_config)
        self.touch_label.grid(row=0, column=1, padx=40, pady=30, sticky="nsew")

        self.no_touch_label = tk.Label(self.sensor_container, text="NO TOUCH",
                                     bg=self.inactive_color, **label_config)
        self.no_touch_label.grid(row=0, column=2, padx=40, pady=30, sticky="nsew")

        # Configure grid columns
        for i in range(3):
            self.sensor_container.grid_columnconfigure(i, weight=1)

        # Bind escape key
        self.root.bind('<Escape>', lambda e: self.clean_exit())

    def update_state(self, msg):
        """Update the GUI based on the current state"""
        self.current_state = msg.data
        self.last_update_time = self.node.get_clock().now()

        # Update all labels to inactive first
        self.reset_labels()

        # Activate the appropriate label
        if self.current_state == 0:
            self.touch_label.config(bg=self.active_color)
            self.node.get_logger().info("TOUCH detected")
        elif self.current_state == 1:
            self.slip_label.config(bg=self.active_color)
            self.node.get_logger().info("SLIP detected")
        elif self.current_state == 2:
            self.no_touch_label.config(bg=self.active_color)
            self.node.get_logger().info("NO TOUCH detected")

    def reset_labels(self):
        """Reset all labels to inactive state"""
        self.slip_label.config(bg=self.inactive_color)
        self.touch_label.config(bg=self.inactive_color)
        self.no_touch_label.config(bg=self.inactive_color)

    def update_gui(self):
        """Check for timeout and update GUI"""
        # Check if we need to reset due to timeout
        elapsed = (self.node.get_clock().now() - self.last_update_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.reset_labels()

        # Process Tkinter events
        try:
            self.root.update()
        except tk.TclError:
            self.node.get_logger().info("GUI window closed")
            self.node.destroy_node()
            rclpy.shutdown()

    def clean_exit(self):
        """Shutdown the GUI"""
        self.node.get_logger().info("Closing GUI window")
        self.root.destroy()




def main(args=None):
    rclpy.init(args=args)
    node = SlipInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
