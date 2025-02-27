import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import RectangleSelector

class InteractivePlotter:
    def __init__(self, timestamps, interactive_arrays, normal_arrays):
        self.timestamps = timestamps
        self.interactive_arrays = interactive_arrays
        self.normal_arrays = normal_arrays
        self.current_index = 0
        self.green_line_index = len(timestamps) // 2
        self.selected_line = "red"
        self.zoom_factor = 0.9
        self.x_limits = [timestamps[0], timestamps[-1]]
        self.is_panning = False
        self.pan_start_x = None

        self.fig, self.axes = plt.subplots(len(interactive_arrays) + len(normal_arrays), 1, figsize=(12, 10))
        self._initialize_plots()
        self._connect_events()
        plt.tight_layout()
        plt.show()

    def _initialize_plots(self):
        self.interactive_lines = []
        self.green_lines = []
        for i, array in enumerate(self.interactive_arrays):
            self.axes[i].plot(self.timestamps, array)
            self.axes[i].set_title(f"Interactive Data {i+1}")
            self.axes[i].set_xlabel("Timestamp")

            red_line = self.axes[i].axvline(x=self.timestamps[self.current_index], color='red', linestyle='--', linewidth=2)
            green_line = self.axes[i].axvline(x=self.timestamps[self.green_line_index], color='green', linestyle='--', linewidth=2)
            self.interactive_lines.append(red_line)
            self.green_lines.append(green_line)

        self.normal_lines = []
        for i, array in enumerate(self.normal_arrays):
            line, = self.axes[len(self.interactive_arrays) + i].plot(array[self.current_index], marker='o')
            self.axes[len(self.interactive_arrays) + i].set_ylim(0, 1)
            self.axes[len(self.interactive_arrays) + i].set_title(f"Normal Data {i+1} at Time {self.timestamps[self.current_index]:.2f}")
            self.axes[len(self.interactive_arrays) + i].set_xlabel("Index")
            self.normal_lines.append(line)

    def _connect_events(self):
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)

        for i in range(min(2, len(self.interactive_arrays))):
            RectangleSelector(self.axes[i], self._on_select, useblit=True, interactive=True, rectprops=dict(alpha=0.5, facecolor="red"))

    def _update_plots(self):
        active_index = self.current_index if self.selected_line == "red" else self.green_line_index

        for i, line in enumerate(self.normal_lines):
            line.set_ydata(self.normal_arrays[i][active_index])
            self.axes[len(self.interactive_arrays) + i].set_title(f"Normal Data {i+1} at Time {self.timestamps[active_index]:.2f} ({self.selected_line.upper()} selected)")

        for red_line in self.interactive_lines:
            red_line.set_xdata(self.timestamps[self.current_index])
        for green_line in self.green_lines:
            green_line.set_xdata(self.timestamps[self.green_line_index])

        self.fig.canvas.draw_idle()

    def _on_key(self, event):
        step_size = 1
        if event.key == "right":
            if self.selected_line == "red" and self.current_index < len(self.timestamps) - 1:
                self.current_index += step_size
            elif self.selected_line == "green" and self.green_line_index < len(self.timestamps) - 1:
                self.green_line_index += step_size
            self._update_plots()
        elif event.key == "left":
            if self.selected_line == "red" and self.current_index > 0:
                self.current_index -= step_size
            elif self.selected_line == "green" and self.green_line_index > 0:
                self.green_line_index -= step_size
            self._update_plots()
        elif event.key == "c":
            self._clip_between_lines()

    def _on_click(self, event):
        if event.xdata is None:
            return
        red_dist = abs(event.xdata - self.timestamps[self.current_index])
        green_dist = abs(event.xdata - self.timestamps[self.green_line_index])
        self.selected_line = "red" if red_dist < green_dist else "green"
        print(f"Selected Line: {self.selected_line.upper()}")
        self._update_plots()

    def _on_scroll(self, event):
        zoom_ref = self.timestamps[self.current_index] if self.selected_line == "red" else self.timestamps[self.green_line_index]
        x_min, x_max = self.x_limits
        zoom_scale = self.zoom_factor if event.step > 0 else (1 / self.zoom_factor)
        self.x_limits = [
            max(self.timestamps[0], zoom_ref - (zoom_ref - x_min) * zoom_scale),
            min(self.timestamps[-1], zoom_ref + (x_max - zoom_ref) * zoom_scale)
        ]
        for ax in self.axes[:len(self.interactive_arrays)]:
            ax.set_xlim(self.x_limits)
        self.fig.canvas.draw_idle()

    def _on_release(self, event):
        """Stop panning on mouse release."""
        self.is_panning = False

    def _on_motion(self, event):
        if self.is_panning and self.pan_start_x is not None and event.xdata is not None:
            shift = self.pan_start_x - event.xdata
            self.x_limits = [max(self.timestamps[0], self.x_limits[0] + shift), min(self.timestamps[-1], self.x_limits[1] + shift)]
            for ax in self.axes[:len(self.interactive_arrays)]:
                ax.set_xlim(self.x_limits)
            self.pan_start_x = event.xdata
            self.fig.canvas.draw_idle()

    def _on_select(self, eclick, erelease):
        print(f"Selected Region: Time [{eclick.xdata}, {erelease.xdata}]")

    def _clip_between_lines(self):
        idx_min, idx_max = sorted([self.current_index, self.green_line_index])
        print(f"Clipping data between timestamps {self.timestamps[idx_min]:.2f} and {self.timestamps[idx_max]:.2f}")

def main():
    np.random.seed(42)
    timestamps = np.linspace(0, 100, 1000)
    arr1 = np.random.rand(1000, 10)
    arr2 = np.random.rand(1000, 10)
    arr3 = np.random.rand(1000, 10)
    arr4 = np.random.rand(1000, 10)
    InteractivePlotter(timestamps, [arr1, arr4], [arr2, arr3])

if __name__ == "__main__":
    main()
