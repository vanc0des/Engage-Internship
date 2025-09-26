import numpy as np
import matplotlib.pyplot as plt

# Your data matrix (skipping the first header row [54, 0, 0, 1])
wkF = [
    54, 0, 0, 1,# 54 is the number of frames. it should not be larger than 127
   9,  49,  53,  45,  24,  20,  -2,  15,
   8,  50,  41,  46,  28,  21,  -1,  15,
  10,  51,  26,  47,  26,  22,   6,  16,
  12,  52,  23,  48,  24,  24,   9,  17,
  14,  52,  20,  49,  22,  26,  12,  18,
  16,  53,  17,  51,  21,  27,  17,  18,
  18,  53,  14,  52,  20,  29,  22,  19,
  21,  54,  11,  54,  18,  30,  27,  19,
  22,  54,  11,  54,  18,  32,  29,  20,
  25,  54,  13,  55,  16,  34,  27,  21,
  26,  54,  16,  56,  16,  37,  24,  23,
  28,  54,  18,  56,  15,  39,  23,  24,
  30,  52,  20,  57,  14,  45,  22,  26,
  32,  54,  22,  57,  14,  44,  21,  28,
  33,  58,  24,  57,  15,  36,  20,  30,
  34,  61,  26,  57,  15,  31,  19,  32,
  36,  64,  28,  57,  14,  24,  18,  35,
  38,  66,  29,  57,  14,  20,  18,  38,
  39,  67,  31,  57,  14,  16,  17,  40,
  41,  64,  32,  56,  14,   5,  17,  43,
  42,  55,  35,  57,  14,  -1,  16,  44,
  44,  44,  37,  62,  15,  -3,  14,  35,
  45,  30,  39,  66,  15,   1,  14,  29,
  46,  21,  40,  68,  15,   5,  14,  23,
  47,  19,  42,  70,  16,   9,  14,  19,
  48,  16,  43,  70,  17,  12,  15,  17,
  49,  12,  44,  67,  18,  17,  15,   5,
  49,   9,  46,  59,  20,  24,  15,  -2,
  50,   8,  47,  47,  21,  28,  16,  -2,
  51,  10,  48,  34,  22,  26,  16,   1,
  52,  12,  49,  24,  24,  24,  17,   6,
  52,  14,  50,  21,  26,  22,  18,  10,
  53,  16,  51,  19,  27,  21,  19,  12,
  53,  18,  52,  15,  29,  20,  20,  19,
  54,  21,  54,  12,  30,  18,  19,  24,
  54,  22,  55,  12,  32,  18,  20,  27,
  54,  25,  55,  11,  34,  16,  22,  29,
  54,  26,  56,  14,  37,  16,  24,  26,
  54,  28,  56,  17,  39,  15,  25,  24,
  52,  30,  57,  18,  45,  14,  27,  23,
  54,  32,  57,  21,  44,  14,  29,  21,
  58,  33,  57,  23,  36,  15,  31,  20,
  61,  34,  57,  24,  31,  15,  33,  20,
  64,  36,  57,  26,  24,  14,  36,  19,
  66,  38,  57,  28,  20,  14,  39,  18,
  67,  39,  56,  30,  16,  14,  42,  17,
  64,  41,  56,  32,   5,  14,  45,  17,
  55,  42,  59,  33,  -1,  14,  41,  17,
  44,  44,  64,  35,  -3,  15,  33,  16,
  30,  45,  67,  38,   1,  15,  27,  14,
  21,  46,  68,  39,   5,  15,  22,  14,
  19,  47,  70,  41,   9,  16,  18,  14,
  16,  48,  69,  42,  12,  17,  11,  14,
  12,  49,  63,  44,  17,  18,   1,  15]

# Extract data matrix (skip first 4 header values)
data_matrix = np.array(wkF[4:]).reshape(-1, 8)
num_frames = wkF[0]  # 54 frames

# Joint names corresponding to columns
joint_names = [
    "RF_shoulder", "RF_knee", "LF_shoulder", "LF_knee",
    "RB_shoulder", "RB_knee", "LB_shoulder", "LB_knee"
]

# Extract Left Front Knee data (column index 3)
lf_knee_index = 3
lf_knee_data = data_matrix[:, lf_knee_index]

print(f"Left Front Knee Joint Data ({len(lf_knee_data)} frames):")
print(f"Min: {np.min(lf_knee_data)}°, Max: {np.max(lf_knee_data)}°")
print(f"Range: {np.max(lf_knee_data) - np.min(lf_knee_data)}°")

# Time vector (assuming 1/240 second per frame as in your original code)
t = np.arange(0, num_frames) / 240  # Convert frames to seconds
N = len(t)  # Period in frames

# Number of harmonics to use
K = 5

def compute_fourier_series(y, N, K):
    """Compute Fourier series coefficients for a signal y"""
    fft_coeffs = np.fft.fft(y) / N
    a0 = fft_coeffs[0].real
    ak = 2 * fft_coeffs[1:K+1].real
    bk = -2 * fft_coeffs[1:K+1].imag
    return a0, ak, bk

def reconstruct_signal(t, N, a0, ak, bk):
    """Reconstruct signal from Fourier coefficients"""
    reconstruction = a0 * np.ones_like(t)
    for k, (a, b) in enumerate(zip(ak, bk), 1):
        angle = 2 * np.pi * k * t * 240 / N  # Convert back to frame space
        reconstruction += a * np.cos(angle) + b * np.sin(angle)
    return reconstruction

# Compute Fourier coefficients for Left Front Knee
a0, ak, bk = compute_fourier_series(lf_knee_data, N, K)

print(f"\n=== LEFT FRONT KNEE FOURIER PARAMETERS ===")
print(f"DC Component (a0): {a0:.4f}")
print(f"Cosine coefficients (ak): {ak}")
print(f"Sine coefficients (bk): {bk}")

# Reconstruct signal
lf_knee_reconstructed = reconstruct_signal(t, N, a0, ak, bk)

# Calculate reconstruction error
mse = np.mean((lf_knee_data - lf_knee_reconstructed)**2)
print(f"\nReconstruction MSE: {mse:.4f}")

# Create visualization
plt.figure(figsize=(12, 8))

# Main plot
plt.subplot(2, 1, 1)
plt.plot(t, lf_knee_data, 'o-', label='Original LF Knee', markersize=4, linewidth=2)
plt.plot(t, lf_knee_reconstructed, '--', label='Fourier Reconstruction', linewidth=2)
plt.title('Left Front Knee Joint Angle - Original vs Fourier Reconstruction')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.grid(True, alpha=0.3)
plt.legend()

# Error plot
plt.subplot(2, 1, 2)
error = lf_knee_data - lf_knee_reconstructed
plt.plot(t, error, 'r-', linewidth=2)
plt.title('Reconstruction Error')
plt.xlabel('Time (s)')
plt.ylabel('Error (degrees)')
plt.grid(True, alpha=0.3)
plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)

plt.tight_layout()
plt.show()

# Generate parameterized function code
print(f"\n=== PARAMETERIZED FUNCTION FOR LEFT FRONT KNEE ===")
print("def left_front_knee_angle(t):")
print(f"    \"\"\"")
print(f"    Left Front Knee joint angle as a function of time")
print(f"    t: time in seconds (one cycle = {t[-1]:.4f}s)")
print(f"    Returns: angle in degrees")
print(f"    \"\"\"")
print(f"    import math")
print(f"    # Fourier series parameters")
print(f"    a0 = {a0:.6f}")
print(f"    ak = {list(ak)}")
print(f"    bk = {list(bk)}")
print(f"    ")
print(f"    # Calculate angle")
print(f"    angle = a0")
print(f"    cycle_time = {t[-1]:.6f}  # One complete cycle time")
print(f"    for k in range(1, {K+1}):")
print(f"        omega_k = 2 * math.pi * k / cycle_time")
print(f"        angle += ak[k-1] * math.cos(omega_k * t) + bk[k-1] * math.sin(omega_k * t)")
print(f"    return angle")

# Export the specific parameters for easy use
print(f"\n=== PARAMETERS FOR COPY-PASTE ===")
print(f"LF_KNEE_A0 = {a0:.6f}")
print(f"LF_KNEE_AK = {list(np.round(ak, 6))}")
print(f"LF_KNEE_BK = {list(np.round(bk, 6))}")
print(f"CYCLE_TIME = {t[-1]:.6f}  # seconds")
print(f"NUM_HARMONICS = {K}")

# Show the raw data for verification
print(f"\n=== RAW LEFT FRONT KNEE DATA (for verification) ===")
print("Frame | Angle")
print("------|------")
for i, angle in enumerate(lf_knee_data):
    print(f"{i:5d} | {angle:5.1f}°")
