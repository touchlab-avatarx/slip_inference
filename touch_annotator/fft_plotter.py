import numpy as np
import matplotlib.pyplot as plt


def process_and_plot_sample(data):
    """
    Given a single raw sample (50 x 84) representing 50 time steps
    of sensor data (84 channels), this function computes the FFT magnitude
    along the time dimension (axis=0) for each channel and plots the result.
    """
    # Compute the FFT along the time dimension (axis=0)
    fft_result = np.fft.fft(data, axis=0)
    # Compute the magnitude of the FFT result
    fft_magnitude = np.abs(fft_result)

    # Plot the FFT magnitude matrix as an image
    plt.figure(figsize=(10, 6))
    plt.imshow(fft_magnitude, aspect='auto', origin='lower', cmap='viridis')
    plt.title("FFT Magnitude along Time Dimension")
    plt.xlabel("Sensor Channels")
    plt.ylabel("Frequency Bins")
    plt.colorbar(label='Magnitude')
    plt.show()


if __name__ == '__main__':
    # Create dummy data of shape (50, 84) for demonstration
    dummy_data = np.random.randn(50, 84)
    process_and_plot_sample(dummy_data)
