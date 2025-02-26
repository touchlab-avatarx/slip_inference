import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import RectangleSelector

# Sample Data (Replace with actual NumPy arrays)
np.random.seed(42)
arr1 = np.random.rand(1000, 10)  # Full time-series data 1
arr2 = np.random.rand(1000, 10)  # Variable over time
arr3 = np.random.rand(1000, 10)  # Variable over time
arr4 = np.random.rand(1000, 10)  # Full time-series data 2

timestamps = np.linspace(0, 100, 1000)  # Simulated timestamps

# Global variables
current_index = 0  # Red line position (current time step)
green_line_index = 500  # Green line position
selected_line = "red"  # Which line is selected (red or green)
zoom_factor = 0.9  # Factor for zooming in/out
x_limits = [timestamps[0], timestamps[-1]]  # Initial zoom limits

# Initialize Plot
fig, axes = plt.subplots(4, 1, figsize=(12, 10))

# --- Plot Full arr1 and arr4 ---
axes[0].plot(timestamps, arr1)
axes[0].set_title("Full Time-Series Data (arr1)")
axes[0].set_xlabel("Timestamp")

axes[1].plot(timestamps, arr4)
axes[1].set_title("Full Time-Series Data (arr4)")
axes[1].set_xlabel("Timestamp")

# Add Red Line (current time step indicator)
red_line1 = axes[0].axvline(x=timestamps[current_index], color='red', linestyle='--', linewidth=2)
red_line2 = axes[1].axvline(x=timestamps[current_index], color='red', linestyle='--', linewidth=2)

# Add Green Line (clipping region indicator)
green_line1 = axes[0].axvline(x=timestamps[green_line_index], color='green', linestyle='--', linewidth=2)
green_line2 = axes[1].axvline(x=timestamps[green_line_index], color='green', linestyle='--', linewidth=2)

# --- Plot arr2 & arr3 for the current selected time step ---
arr2_line, = axes[2].plot(arr2[current_index], marker='o')
axes[2].set_ylim(0, 1)
axes[2].set_title(f"arr2 at Time {timestamps[current_index]:.2f}")
axes[2].set_xlabel("Index")

arr3_line, = axes[3].plot(arr3[current_index], marker='o')
axes[3].set_ylim(0, 1)
axes[3].set_title(f"arr3 at Time {timestamps[current_index]:.2f}")
axes[3].set_xlabel("Index")


# --- Function to Update Plots ---
def update_plots():
    """ Update arr2 and arr3 plots based on the selected timestamp (red or green). """
    global current_index, green_line_index, selected_line

    # Use selected line's timestamp
    if selected_line == "red":
        active_index = current_index
    else:
        active_index = green_line_index

    # Update arr2 and arr3 plots
    arr2_line.set_ydata(arr2[active_index])
    arr3_line.set_ydata(arr3[active_index])

    # Update Titles
    axes[2].set_title(f"arr2 at Time {timestamps[active_index]:.2f} ({selected_line.upper()} selected)")
    axes[3].set_title(f"arr3 at Time {timestamps[active_index]:.2f} ({selected_line.upper()} selected)")

    # Update Line Positions
    red_line1.set_xdata(timestamps[current_index])
    red_line2.set_xdata(timestamps[current_index])
    green_line1.set_xdata(timestamps[green_line_index])
    green_line2.set_xdata(timestamps[green_line_index])

    fig.canvas.draw_idle()


# --- Handle Key Press Events ---
def on_key(event):
    """ Move the selected line using left/right arrow keys and clip with 'C'. """
    global current_index, green_line_index, selected_line

    step_size = 1  # Fine-grained control

    if event.key == "right":
        if selected_line == "red" and current_index < arr1.shape[0] - 1:
            current_index += step_size
        elif selected_line == "green" and green_line_index < arr1.shape[0] - 1:
            green_line_index += step_size
        update_plots()

    elif event.key == "left":
        if selected_line == "red" and current_index > 0:
            current_index -= step_size
        elif selected_line == "green" and green_line_index > 0:
            green_line_index -= step_size
        update_plots()

    elif event.key == "c":
        clip_between_lines()


fig.canvas.mpl_connect("key_press_event", on_key)


# --- Select Which Line is Active by Clicking ---
def on_click(event):
    """ Switch selection between red and green line based on click position. """
    global selected_line

    if event.xdata is None:
        return  # Ignore clicks outside the plot

    # Get closest timestamp index
    click_pos = event.xdata
    red_dist = abs(click_pos - timestamps[current_index])
    green_dist = abs(click_pos - timestamps[green_line_index])

    # Select the nearest line
    if red_dist < green_dist:
        selected_line = "red"
    else:
        selected_line = "green"

    print(f"Selected Line: {selected_line.upper()}")
    update_plots()


fig.canvas.mpl_connect("button_press_event", on_click)


# --- Zoom Functionality Based on Selected Line ---
def on_scroll(event):
    """ Zoom in/out while keeping the selected line fixed in view. """
    global x_limits

    # Use the selected line as the zoom reference
    zoom_ref = timestamps[current_index] if selected_line == "red" else timestamps[green_line_index]

    x_min, x_max = x_limits

    if selected_line == "red":
        if event.step > 0:  # Zoom in
            x_min = max(timestamps[0], zoom_ref - (zoom_ref - x_min) * zoom_factor)
        else:  # Zoom out
            x_min = max(timestamps[0], zoom_ref - (zoom_ref - x_min) / zoom_factor)
    else:  # Zoom relative to green
        if event.step > 0:  # Zoom in
            x_max = min(timestamps[-1], zoom_ref + (x_max - zoom_ref) * zoom_factor)
        else:  # Zoom out
            x_max = min(timestamps[-1], zoom_ref + (x_max - zoom_ref) / zoom_factor)

    x_limits = [x_min, x_max]

    # Apply zoom to both arr1 and arr4 plots
    axes[0].set_xlim(x_limits)
    axes[1].set_xlim(x_limits)

    fig.canvas.draw_idle()


fig.canvas.mpl_connect("scroll_event", on_scroll)


# --- Annotation (Rectangle Selection on arr1 or arr4) ---
def on_select(eclick, erelease):
    """ Store selected region from arr1 or arr4 for clipping. """
    x_min, x_max = eclick.xdata, erelease.xdata
    idx_min = np.abs(timestamps - x_min).argmin()
    idx_max = np.abs(timestamps - x_max).argmin()

    print(f"Selected Region: Time [{timestamps[idx_min]:.2f}, {timestamps[idx_max]:.2f}] → Index [{idx_min}, {idx_max}]")

selector1 = RectangleSelector(axes[0], on_select, useblit=True, interactive=True, rectprops=dict(alpha=0.5, facecolor="red"))
selector2 = RectangleSelector(axes[1], on_select, useblit=True, interactive=True, rectprops=dict(alpha=0.5, facecolor="red"))



# --- Clip Data Between Red and Green Lines ---
def clip_between_lines():
    """ Clip data between the red and green lines on both arr1 and arr4. """
    idx_min, idx_max = sorted([current_index, green_line_index])

    clipped_arr1 = arr1[idx_min:idx_max]
    clipped_arr2 = arr2[idx_min:idx_max]
    clipped_arr3 = arr3[idx_min:idx_max]
    clipped_arr4 = arr4[idx_min:idx_max]

    print(f"Clipped Between {timestamps[idx_min]:.2f} and {timestamps[idx_max]:.2f}")
    print(
        f"Clipped Shapes: arr1 {clipped_arr1.shape}, arr2 {clipped_arr2.shape}, arr3 {clipped_arr3.shape}, arr4 {clipped_arr4.shape}")


plt.tight_layout()
plt.show()
