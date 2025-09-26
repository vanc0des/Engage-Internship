import numpy as np
import matplotlib.pyplot as plt
import time

# === 1) Your complete gait array ===
wkF = [ 
39, 0, 0, 1,
  18,  54,  58,  52,   7,  13,  -2,   9,
  14,  56,  52,  54,  12,  14,  -3,  10,
  15,  57,  46,  55,  17,  15,  -4,  11,
  16,  58,  38,  57,  16,  17,  -2,  12,
  19,  59,  31,  58,  13,  19,   1,  14,
  22,  59,  24,  60,  12,  22,   6,  14,
  26,  60,  21,  61,  10,  24,  12,  16,
  28,  58,  23,  62,   9,  30,  16,  18,
  31,  61,  25,  63,   8,  30,  13,  20,
  34,  67,  28,  63,   7,  23,  12,  24,
  36,  69,  31,  63,   6,  15,  11,  27,
  38,  68,  34,  63,   7,   8,  10,  31,
  41,  65,  36,  62,   7,   3,   9,  35,
  43,  60,  39,  65,   7,  -1,   9,  37,
  46,  55,  41,  70,   7,  -3,   9,  27,
  47,  50,  43,  71,   7,  -5,   9,  23,
  49,  43,  47,  71,   8,  -5,   7,  14,
  51,  35,  48,  69,   8,  -3,   8,   8,
  53,  28,  51,  66,   9,   1,   8,   2,
  54,  18,  52,  63,  12,   7,   9,  -2,
  55,  15,  54,  56,  14,  10,  10,  -3,
  56,  13,  55,  51,  15,  16,  11,  -4,
  57,  16,  57,  43,  17,  16,  12,  -3,
  58,  18,  58,  36,  19,  14,  14,  -2,
  59,  21,  60,  28,  21,  12,  14,   2,
  60,  25,  61,  23,  24,  10,  16,   9,
  60,  28,  62,  22,  27,   9,  18,  15,
  60,  31,  63,  23,  30,   8,  20,  16,
  66,  32,  63,  26,  24,   7,  24,  13,
  68,  35,  63,  29,  19,   6,  27,  11,
  68,  38,  63,  32,  10,   6,  31,  10,
  66,  40,  62,  35,   4,   7,  35,  10,
  62,  42,  65,  37,   0,   7,  37,   9,
  57,  45,  70,  39,  -2,   7,  27,   9,
  52,  47,  71,  41,  -5,   7,  23,   9,
  45,  48,  71,  44,  -5,   8,  14,   9,
  38,  51,  69,  47,  -3,   8,   8,   7,
  30,  52,  66,  49,  -1,   9,   2,   8,
  22,  53,  63,  51,   4,  12,  -2,   8,
]

# === 2) Parse data ===
numFrames = wkF[0]
jointData = np.array(wkF[4:]).reshape(numFrames, 8)

jointNames = [
    "RF_Shoulder", "RF_Knee",
    "LF_Shoulder", "LF_Knee",
    "RB_Shoulder", "RB_Knee",
    "LB_Shoulder", "LB_Knee"
]

# === 3) Transformation function ===
def transform_gait(data, a, b, c):
    """Apply g(x) = a*(f(x-b)-c) for each joint"""
    numJoints = data.shape[1]
    extended_data = np.tile(data, (3, 1))
    transformed_data = np.zeros_like(data)

    for j in range(numJoints):
        shifted = np.roll(extended_data[:, j], -b[j])
        transformed = a[j] * (shifted - c[j])
        transformed_data[:, j] = transformed[:data.shape[0]]
    return transformed_data

# === 4) Visualization function ===
def plot_gait_comparison(original_data, transformed_data, joint_names):
    """Plot original vs transformed gait patterns"""
    fig, axes = plt.subplots(4, 2, figsize=(15, 12))
    fig.suptitle('Original vs Transformed Gait Patterns', fontsize=16)
    
    frames = range(len(original_data))
    
    for i, joint_name in enumerate(joint_names):
        row = i // 2
        col = i % 2
        ax = axes[row, col]
        
        ax.plot(frames, original_data[:, i], 'b-', linewidth=2, label='Original', alpha=0.7)
        ax.plot(frames, transformed_data[:, i], 'r-', linewidth=2, label='Transformed', alpha=0.7)
        
        ax.set_title(joint_name, fontsize=12, fontweight='bold')
        ax.set_xlabel('Frame')
        ax.set_ylabel('Angle')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Set y-axis limits for better visualization
        y_min = min(np.min(original_data[:, i]), np.min(transformed_data[:, i])) - 5
        y_max = max(np.max(original_data[:, i]), np.max(transformed_data[:, i])) + 5
        ax.set_ylim(y_min, y_max)
    
    plt.tight_layout()
    plt.show()

# === 5) Export function ===
def export_to_text_file(array, filename="gallop_gait.txt"):
    """Export array to text file with gallop format (overwrites existing file)"""
    numFrames = array.shape[0]
    
    with open(filename, 'w') as f:
        f.write(f"# Gallop gait data - {numFrames} frames\n")
        f.write(f"# Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"# Transformation applied: g(x) = a*(f(x-b)-c)\n")
        f.write(f"glF = [\n")
        f.write(f"{numFrames}, 0, 0, 1,\n")
        
        for i in range(numFrames):
            row = array[i]
            row_str = "  " + ", ".join(f"{int(val):3}" for val in row) + ","
            f.write(row_str + f"  # Frame {i}\n")
        
        f.write("]\n")
    
    print(f"✅ Gallop gait exported to {filename}")

# === 6) Main execution ===
if __name__ == "__main__":
    print("🔄 Starting gait transformation and visualization...")
    
    # Transformation parameters
    a = [1.2, 1.5, 1.2, 1.5, 1.2, 1.5, 1.2, 1.5]  # Scaling factors
    b = [0, 0, 0, 0, 15, 15, 15, 15]                # Phase shifts
    c = [0, 0, 0, 0, 0, 0, 0, 0]                    # Offsets
    
    print("🔧 Applying transformation to gait data...")
    print(f"Parameters: a={a}")
    print(f"Parameters: b={b}")
    print(f"Parameters: c={c}")
    
    # Apply transformation
    transformed = transform_gait(jointData, a, b, c)
    
    # Show visualization
    print("📊 Displaying gait comparison graphs...")
    plot_gait_comparison(jointData, transformed, jointNames)
    
    # Export transformed data
    export_to_text_file(transformed, "gallop_gait.txt")
    
    print("🎉 Process completed!")
    print("📁 Check 'gallop_gait.txt' for the transformed gait data")
    print("🤖 You can now use this data in your robot control script")
