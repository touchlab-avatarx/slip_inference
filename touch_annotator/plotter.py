import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import RectangleSelector

# Sample Data (Replace with real NumPy arrays)
np.random.seed(42)
arr1 = np.random.rand(1000, 10)  # Full time series data
arr2 = np.random.rand(1000, 10)  # Variable over time
arr3 = np.random.rand(1000, 10)  # Variable over time

timestamps = np.linspace(0, 100, 1000)  # Simulated timestamps

# Global index for time step navigation
current_index = 0
selected_region = None  # Store selected annotation region

# Initialize Plot
fig, axes = plt.subplots(3, 1, figsize=(10, 8))

# --- Plot Full arr1 ---
axes[0].plot(timestamps, arr1)
axes[0].set_title("Full Time-Series Data (arr1)")
axes[0].set_xlabel("Timestamp")

# --- Plot arr2 & arr3 for the current time step ---
arr2_line, = axes[1].plot(arr2[current_index], marker='o')
axes[1].set_ylim(0, 1)
axes[1].set_title("arr2 at Current Time Step")
axes[1].set_xlabel("Index")

arr3_line, = axes[2].plot(arr3[current_index], marker='o')
axes[2].set_ylim(0, 1)
axes[2].set_title("arr3 at Current Time Step")
axes[2].set_xlabel("Index")


# --- Function to Update Plots on Key Press ---
def update_plots():
    """ Update arr2 and arr3 plots based on the current time step. """
    global current_index
    arr2_line.set_ydata(arr2[current_index])
    arr3_line.set_ydata(arr3[current_index])

    axes[1].set_title(f"arr2 at Time {timestamps[current_index]:.2f}")
    axes[2].set_title(f"arr3 at Time {timestamps[current_index]:.2f}")

    fig.canvas.draw_idle()


def on_key(event):
    """ Handle Left/Right Arrow Key Presses to Navigate Time Steps. """
    global current_index
    if event.key == "right":
        if current_index < arr1.shape[0] - 1:
            current_index += 1
            update_plots()
    elif event.key == "left":
        if current_index > 0:
            current_index -= 1
            update_plots()


fig.canvas.mpl_connect("key_press_event", on_key)


# --- Annotation (Selection on arr1) ---
def on_select(eclick, erelease):
    """ Store selected region from arr1 for clipping arrays. """
    global selected_region
    x_min, x_max = eclick.xdata, erelease.xdata

    # Get nearest time indices
    idx_min = np.abs(timestamps - x_min).argmin()
    idx_max = np.abs(timestamps - x_max).argmin()

    selected_region = (idx_min, idx_max)
    print(
        f"Selected Region: Time [{timestamps[idx_min]:.2f}, {timestamps[idx_max]:.2f}] → Index [{idx_min}, {idx_max}]")

    # Example: Clipping arrays
    clipped_arr1 = arr1[idx_min:idx_max]
    clipped_arr2 = arr2[idx_min:idx_max]
    clipped_arr3 = arr3[idx_min:idx_max]

    print(f"Clipped arr1 Shape: {clipped_arr1.shape}")
    print(f"Clipped arr2 Shape: {clipped_arr2.shape}")
    print(f"Clipped arr3 Shape: {clipped_arr3.shape}")


selector = RectangleSelector(axes[0], on_select, useblit=True,
                             interactive=True, rectprops=dict(alpha=0.5, facecolor="red"))

plt.tight_layout()
plt.show()
