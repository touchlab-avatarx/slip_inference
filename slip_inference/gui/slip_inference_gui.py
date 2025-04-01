from tkinter import *
import tkinter as tk
from tkinter import font as tkfont
import random  # For simulation only


class SensorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SLIP DETECTION")
        self.root.geometry("400x400")

        # Set full-screen mode
        self.root.attributes("-fullscreen", True)

        # Configure styles
        self.label_font = tkfont.Font(family="Helvetica", size=15, weight="bold")
        self.heading_font = tkfont.Font(family="Helvetica", size=20, weight="bold")
        self.active_color = "green"
        self.inactive_color = "red"

        # Create main container
        self.main_container = tk.Frame(root)
        self.main_container.pack(expand=True, fill=tk.BOTH)

        # Add heading frame
        self.heading_frame = tk.Frame(self.main_container, bg="#333333")
        self.heading_frame.pack(fill=tk.X, pady=(0, 20))

        # Heading label
        self.heading_label = tk.Label(
            self.heading_frame,
            text="SLIP DETECTION INFERENCE",
            font=self.heading_font,
            bg="#333333",
            fg="white",
            pady=20,
        )
        self.heading_label.pack(expand=True)

        # CENTERED CONTENT FRAME
        self.center_frame = tk.Frame(self.main_container)
        self.center_frame.pack(expand=True, fill=tk.BOTH)

        # Create a container just for the sensor labels
        self.sensor_container = tk.Frame(self.center_frame)
        self.sensor_container.pack(expand=True)  # This centers the container

        # Create sensor labels in a horizontal row
        self.slip_label = tk.Label(
            self.sensor_container,
            text="SLIP",
            font=self.label_font,
            bg=self.inactive_color,
            fg="white",
            width=40,
            height=25,
            padx=40,
            pady=30,
            highlightthickness=5,  # Border width
            highlightbackground="black",  # Border color
            highlightcolor="yellow",
        )  # Border color
        self.slip_label.pack(side=tk.LEFT, padx=10)

        self.touch_label = tk.Label(
            self.sensor_container,
            text="TOUCH",
            font=self.label_font,
            bg=self.inactive_color,
            fg="white",
            width=40,
            height=25,
            padx=40,
            pady=30,
            highlightthickness=5,  # Border width
            highlightbackground="black",  # Border color
            highlightcolor="yellow",
        )  # Border color
        self.touch_label.pack(side=tk.LEFT, padx=10)

        self.no_touch_label = tk.Label(
            self.sensor_container,
            text="NO TOUCH",
            font=self.label_font,
            bg=self.inactive_color,
            fg="white",
            width=40,
            height=25,
            padx=40,
            pady=30,
            highlightthickness=5,  # Border width
            highlightbackground="black",  # Border color
            highlightcolor="yellow",
        )  # Border color
        self.no_touch_label.pack(side=tk.LEFT, padx=10)

        self.slip_label.pack(side=tk.LEFT, padx=50, pady=10)
        self.touch_label.pack(side=tk.LEFT, padx=50, pady=10)
        self.no_touch_label.pack(side=tk.LEFT, padx=50, pady=10)

        # Initialize topic states
        self.current_topic = None
        self.topic_timeout = 1000  # Time in ms before reverting to inactive
        self.timeout_id = None

        # Bind exit keys
        self.root.bind("<Escape>", self.clean_exit)

        # For simulation only - replace with your actual topic receiver
        self.topic_reciever()

    def toggle_fullscreen(self, event=None):
        """Toggle fullscreen mode with Escape key"""
        self.root.attributes("-fullscreen", not self.root.attributes("-fullscreen"))

    def update_label(self, topic):
        """Update labels based on received topic"""
        # Cancel any pending timeout
        if self.timeout_id:
            self.root.after_cancel(self.timeout_id)
            self.timeout_id = None

        # Reset all labels
        self.reset_labels()

        # Activate the appropriate label
        if topic == 0:
            self.slip_label.config(bg=self.active_color)
        elif topic == 1:
            self.touch_label.config(bg=self.active_color)
        elif topic == 2:
            self.no_touch_label.config(bg=self.active_color)

        # Set timeout to revert to inactive if no new topic is received
        self.timeout_id = self.root.after(self.topic_timeout, self.reset_labels)

    def reset_labels(self):
        """Reset all labels to inactive state"""
        self.slip_label.config(bg=self.inactive_color)
        self.touch_label.config(bg=self.inactive_color)
        self.no_touch_label.config(bg=self.inactive_color)
        self.timeout_id = None

    # Simulation function - replace with actual topic receiver
    def topic_reciever(self):
        """Simulate receiving topics (for demo only)"""
        # Randomly select a topic (0, 1, or 2)
        topic = random.randint(0, 2)
        self.update_label(topic)

        # Schedule next simulation update
        self.root.after(2000, self.topic_reciever)

    def clean_exit(self, event=None):
        """Clean up resources and exit the application"""
        self.simulation_running = False

        # Cancel any pending timeouts
        if self.timeout_id:
            self.root.after_cancel(self.timeout_id)

        # In a real application, you would also:
        # - Disconnect from network resources
        # - Close serial connections
        # - Stop ROS nodes, etc.

        # Exit the application
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = SensorGUI(root)
    root.mainloop()
