#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 
import tkinter as tk
from tkinter import font as tkfont

class SensorGUI(Node):
    def __init__(self, topic_name='slip_state'):
        super().__init__('slip_detection_gui')
        
        # ROS2 Subscription
        self.subscription = self.create_subscription(
            Int32,
            topic_name,  # Topic name
            self.topic_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Initialize Tkinter
        self.root = tk.Tk()
        self.root.title("SLIP DETECTION")
        
        
        # Fullscreen configuration
        self.fullscreen = True
        self.root.attributes('-fullscreen', self.fullscreen)
        
        # Configure styles
        self.label_font = tkfont.Font(family='Helvetica', size=15, weight='bold')
        self.heading_font = tkfont.Font(family='Helvetica', size=20, weight='bold')
        self.active_color = "green"
        self.inactive_color = "red"

        # Create GUI elements
        self.setup_gui()
        
        # ROS2 timer for GUI updates
        self.timer = self.create_timer(0.1, self.update_gui)
        
        # State variables
        self.current_topic = None
        self.last_update_time = self.get_clock().now()
        self.timeout = 1.0  # seconds before reverting to inactive

        self.get_logger().info("Slip Detection GUI initialized, subscribing to /robotiq/rig1_all/raw")
        
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
            'fg': "white",
            'width': 40,
            'height': 25,
            'highlightthickness': 5, # Border thickness
            'highlightbackground': "black", # Border color
            'highlightcolor': "yellow"
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

    def topic_callback(self, msg):
        """Handle incoming ROS2 messages from slip_state topic"""
        self.current_state = msg.data
        self.last_update_time = self.get_clock().now()
        
        # Update all labels to inactive first
        self.reset_labels()
        
        # Activate the appropriate label
        if self.current_state == 0:
            self.slip_label.config(bg=self.active_color)
            self.get_logger().info("SLIP detected")
        elif self.current_state == 1:
            self.touch_label.config(bg=self.active_color)
            self.get_logger().info("TOUCH detected")
        elif self.current_state == 2:
            self.no_touch_label.config(bg=self.active_color)
            self.get_logger().info("NO TOUCH detected")
        
        # # Update last message time display
        # self.topic_label.config(text=f"Last message: {self.get_clock().now().to_msg().sec} sec")
    
    def reset_labels(self):
        """Reset all labels to inactive state"""
        self.slip_label.config(bg=self.inactive_color)
        self.touch_label.config(bg=self.inactive_color)
        self.no_touch_label.config(bg=self.inactive_color)
    
    def update_gui(self):
        """Check for timeout and update GUI"""
        # Check if we need to reset due to timeout
        elapsed = (self.get_clock().now() - self.last_update_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.reset_labels()
        
        # Process Tkinter events
        try:
            self.root.update()
        except tk.TclError:
            self.get_logger().info("GUI window closed")
            rclpy.shutdown()
    
    def clean_exit(self):
        """Shutdown the node and GUI"""
        self.get_logger().info("Shutting down slip detection GUI")
        self.root.destroy()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        gui_node = SensorGUI(topic_name='slip_state')
        rclpy.spin(gui_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            gui_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()