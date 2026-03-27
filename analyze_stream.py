#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# --- Configuration ---
FILE_NAME = 'capture.raw'
SAMPLE_RATE = 2e6        # Decimated to 2 Msps
LPF_CUTOFF = 0.8e6       # 800 kHz Low-Pass Filter (Nyquist is 1MHz)

def load_and_unpack_1bit_iq(filename):
    """
    Reads the raw binary stream and unpacks the 1-bit I/Q packed data.
    """
    # Read as little-endian 32-bit unsigned integers
    words = np.fromfile(filename, dtype='<u4')
    words = words[:1024] # just look at the start for now
    
    # We have 16 samples packed per word.
    # Create bit shift arrays based on your C packing logic.
    # Sample 0 is at shifts 31 (I) and 30 (Q)
    # Sample 15 is at shifts 1 (I) and 0 (Q)
    shifts_i = np.arange(31, -1, -2) # [31, 29, 27 ... 1]
    shifts_q = np.arange(30, -1, -2) # [30, 28, 26 ... 0]

    # Extract bits using NumPy broadcasting
    # words[:, None] makes it a 2D array, allowing bitwise ops across the shifts
    i_bits = (words[:, None] >> shifts_i) & 1
    q_bits = (words[:, None] >> shifts_q) & 1

    # Flatten the 2D arrays back into a 1D stream of bits
    i_bits = i_bits.flatten()
    q_bits = q_bits.flatten()

    # Map the sign bits back to analog values
    # In 2's complement, bit 1 means negative, bit 0 means positive
    # Equation: 1.0 - 2.0 * bit  (maps 0 -> +1.0, 1 -> -1.0)
    i_val = 1.0 - 2.0 * i_bits
    q_val = 1.0 - 2.0 * q_bits

    # Form the complex signal
    iq_signal = i_val + 1j * q_val
    return iq_signal

def main():
    print(f"Loading 1-bit stream {FILE_NAME}...")
    try:
        iq = load_and_unpack_1bit_iq(FILE_NAME)
    except FileNotFoundError:
        print(f"Error: Could not find {FILE_NAME}.")
        return

    num_samples = len(iq)
    capture_time_us = (num_samples / SAMPLE_RATE) * 1e6
    print(f"Loaded {num_samples} samples ({capture_time_us:.2f} microseconds of RF data)")

    # 1. Low-Pass Filter the 1-bit data to smooth quantization harmonics
    # This turns our square waves back into smooth phase rotations
    nyquist = SAMPLE_RATE / 2
    b, a = butter(4, LPF_CUTOFF / nyquist, btype='low')
    iq_filtered = filtfilt(b, a, iq)

    # 2. FSK Demodulation (FM Discriminator)
    # Extract instantaneous frequency from the phase diff of adjacent samples
    phase_diff = np.angle(iq_filtered[1:] * np.conj(iq_filtered[:-1]))
    inst_freq = phase_diff * (SAMPLE_RATE / (2 * np.pi))

    # --- Plotting ---
    time_axis_us = np.arange(num_samples) * (1e6 / SAMPLE_RATE)
    time_axis_freq_us = time_axis_us[:-1]

    fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Continuous 1-Bit SDR Capture (2 Msps)', fontsize=16)

    # Plot 1: Spectrogram (Waterfall)
    axs[0].set_title("Spectrogram")
    axs[0].specgram(iq_filtered, NFFT=128, Fs=SAMPLE_RATE/1e6, noverlap=64, cmap='viridis')
    axs[0].set_ylabel('Frequency (MHz)')

    # Plot 2: Demodulated FSK 
    axs[1].set_title("Demodulated FSK (Instantaneous Frequency)")
    axs[1].plot(time_axis_freq_us, inst_freq / 1e6, color='red', linewidth=1.5)
    axs[1].set_ylabel('Freq Deviation (MHz)')
    axs[1].set_xlabel('Time (Microseconds)')
    axs[1].grid(True)
    axs[1].set_ylim([-1.0, 1.0]) # Cap to +/- 1 MHz

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
