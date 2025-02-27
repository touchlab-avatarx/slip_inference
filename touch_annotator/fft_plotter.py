import numpy as np
import matplotlib.pyplot as plt


def process_and_plot_sample(data, sample_rate=150):
    """
    Given a single raw sample (50 x 84) representing 50 time steps
    of sensor data (84 channels), this function computes the FFT magnitude
    along the time dimension (axis=0) for each channel and produces two plots:
      - An image plot of the FFT magnitude (frequency bins vs. sensor channels).
      - A line plot for channel 0 with frequency in Hz on the x-axis and magnitude (in dB)
        on the y-axis for a more human-readable frequency response.
    """
    if data.shape[0] != 50:
        raise ValueError("Input data must have 50 time steps")

    # Compute FFT along time dimension (axis=0)
    fft_result = np.fft.fft(data, axis=0)
    fft_magnitude = np.abs(fft_result)

    # Create a frequency array based on the sampling rate and number of samples (50)
    N = data.shape[0]
    freq = np.fft.fftfreq(N, d=1 / sample_rate)  # Frequency values (in Hz)

    # For a human-friendly plot, only use the positive frequencies.
    pos_mask = freq >= 0
    freq_pos = freq[pos_mask]
    fft_magnitude_pos = fft_magnitude[pos_mask, :]

    # Create a figure with two subplots side by side.
    fig, axs = plt.subplots(1, 2, figsize=(14, 6))

    # Left subplot: Image of FFT magnitude (all channels)
    im = axs[0].imshow(fft_magnitude, aspect='auto', origin='lower', cmap='viridis')
    axs[0].set_title("FFT Magnitude (All Channels)")
    axs[0].set_xlabel("Sensor Channels")
    axs[0].set_ylabel("Frequency Bins")
    fig.colorbar(im, ax=axs[0], label='Magnitude')

    # Right subplot: Detailed frequency plot for one channel (channel 0)
    # Convert magnitude to decibels for better dynamic range visualization
    fft_db = 20 * np.log10(fft_magnitude_pos[:, 0] + 1e-6)  # Adding epsilon to avoid log(0)
    axs[1].plot(freq_pos, fft_db, marker='o')
    axs[1].set_title("FFT Spectrum for Channel 0")
    axs[1].set_xlabel("Frequency (Hz)")
    axs[1].set_ylabel("Magnitude (dB)")
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # Generate dummy data of shape (50, 84) to simulate 50 time steps of 84 sensor channels.
    dummy_data = np.random.randn(50, 84)
    process_and_plot_sample(dummy_data, sample_rate=150)
