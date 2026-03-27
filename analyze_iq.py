#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# --- Configuration ---
FILE_NAME = 'capture.raw'
SAMPLE_RATE = 16e6        # 16 Msps
LPF_CUTOFF = 2e6          # 2 MHz Low-Pass Filter (good for 1-2Mbps FSK/BLE)

def load_and_unpack_iq(filename):
    """
    Reads the raw binary file and reconstructs the complex IQ signal
    by perfectly reversing the C bitwise operations.
    """
    # Read the data as 32-bit unsigned integers
    raw_data = np.fromfile(filename, dtype=np.uint32)
    
    # Your C code packed the data as:
    # [ I_LSB (24) | I_MSB (16) | Q_LSB (8) | Q_MSB (0) ]
    
    i_lsb = (raw_data >> 24) & 0xFF
    i_msb = (raw_data >> 16) & 0xFF
    q_lsb = (raw_data >> 8) & 0xFF
    q_msb = raw_data & 0xFF

    # Reassemble the 16-bit integers
    i_val = (i_msb << 8) | i_lsb
    q_val = (q_msb << 8) | q_lsb

    # Convert to signed 16-bit (two's complement)
    i_val = i_val.astype(np.int16)
    q_val = q_val.astype(np.int16)

    # Convert to complex NumPy array
    iq_signal = i_val.astype(np.float32) + 1j * q_val.astype(np.float32)
    return iq_signal[1024:] # trim messy startup

def main():
    print(f"Loading {FILE_NAME}...")
    try:
        iq = load_and_unpack_iq(FILE_NAME)
    except FileNotFoundError:
        print(f"Error: Could not find {FILE_NAME}. Did you run the USB capture script?")
        return

    num_samples = len(iq)
    capture_time_us = (num_samples / SAMPLE_RATE) * 1e6
    print(f"Loaded {num_samples} samples ({capture_time_us:.2f} microseconds of RF data)")

    # 1. Calculate Signal Power (Amplitude squared) to easily spot the burst
    power = np.abs(iq)**2
    # Normalize power to 0-1 for plotting
    power = power / np.max(power) 

    # 2. Low-Pass Filter the IQ data to remove out-of-band noise
    # This is crucial for 16Msps captures since the FSK bandwidth is only ~1-2 MHz.
    nyquist = SAMPLE_RATE / 2
    b, a = butter(4, LPF_CUTOFF / nyquist, btype='low')
    iq_filtered = filtfilt(b, a, iq)

    # 3. FSK Demodulation (FM Discriminator)
    # We find the instantaneous frequency by taking the angle difference between adjacent samples
    # angle( s[n] * conj(s[n-1]) ) gives the phase shift per sample.
    phase_diff = np.angle(iq_filtered[1:] * np.conj(iq_filtered[:-1]))
    
    # Convert phase shift to Instantaneous Frequency (in Hz)
    inst_freq = phase_diff * (SAMPLE_RATE / (2 * np.pi))

    # --- Plotting ---
    time_axis_us = np.arange(num_samples) * (1e6 / SAMPLE_RATE)
    time_axis_freq_us = time_axis_us[:-1] # One sample shorter due to differentiation

    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    fig.suptitle('Raw IQ Capture Analysis (16 Msps)', fontsize=16)

    # Plot 1: Spectrogram (Waterfall)
    axs[0].set_title("Spectrogram (Shows FSK Frequency Shifts)")
    axs[0].specgram(iq_filtered, NFFT=128, Fs=SAMPLE_RATE/1e6, noverlap=64, cmap='viridis')
    axs[0].set_ylabel('Frequency (MHz)')

    # Plot 2: Power Envelope
    axs[1].set_title("Signal Power Envelope (Find the Burst)")
    axs[1].plot(time_axis_us, power, color='blue', linewidth=1)
    axs[1].set_ylabel('Normalized Power')
    axs[1].grid(True)

    # Plot 3: Demodulated FSK (Instantaneous Frequency)
    axs[2].set_title("Demodulated FSK (Instantaneous Frequency)")
    axs[2].plot(time_axis_freq_us, inst_freq / 1e6, color='red', linewidth=1.5)
    axs[2].set_ylabel('Freq Deviation (MHz)')
    axs[2].set_xlabel('Time (Microseconds)')
    axs[2].grid(True)

    # To make the demodulation plot look cleaner, we can optionally zoom the Y-axis 
    # to typical BLE/Nordic FSK deviation ranges (+/- 1 MHz)
    axs[2].set_ylim([-1.5, 1.5])

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
