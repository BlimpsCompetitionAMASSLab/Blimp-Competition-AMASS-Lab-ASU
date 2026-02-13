import matplotlib.pyplot as plt
import numpy as np
import re

def parse_data_file(filepath):
    """Parse the PI data file and extract time and motor commands."""
    times = []
    l_motor = []
    r_motor = []
    u_motor = []
    d_motor = []
    
    with open(filepath, 'r') as f:
        for line in f:
            # Parse line: Position: x,y L Motor: val R Motor: val U Motor: val D Motor: val time: val
            match = re.search(r'L Motor: ([\d.]+) R Motor: ([\d.]+) U Motor: ([\d.]+) D Motor: ([\d.]+) time: ([\d.e+-]+)', line)
            if match:
                l_motor.append(float(match.group(1)))
                r_motor.append(float(match.group(2)))
                u_motor.append(float(match.group(3)))
                d_motor.append(float(match.group(4)))
                times.append(float(match.group(5)))
    
    return np.array(times), np.array(l_motor), np.array(r_motor), np.array(u_motor), np.array(d_motor)

# Parse both data files
data_dir = r'c:\Users\prajj\OneDrive - Arizona State University\ASU\PhD\blimp_ws\blimp_src\logs'
time1, l1, r1, u1, d1 = parse_data_file(f'{data_dir}\\PI_Data_2.txt')
time2, l2, r2, u2, d2 = parse_data_file(f'{data_dir}\\PI_Data_3.txt')

# Create figure with subplots for active motors only (U and D renamed as L and R)
fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
fig.suptitle('Motor Thrust Commands Over Time', fontsize=16, fontweight='bold')

# Plot L Motor (originally U Motor)
axes[0].plot(time1, u1, 'b-', linewidth=1.5, label='PI_Data_2', alpha=0.7)
axes[0].plot(time2, u2, 'r-', linewidth=1.5, label='PI_Data_3', alpha=0.7)
axes[0].axhline(y=1050, color='gray', linestyle='--', linewidth=1, alpha=0.5, label='Neutral (1050)')
axes[0].set_ylabel('L Motor\nPWM', fontsize=11, fontweight='bold')
axes[0].grid(True, alpha=0.3)
axes[0].legend(loc='upper right', fontsize=9)
axes[0].set_ylim([1040, max(max(u1), max(u2)) + 10])

# Plot R Motor (originally D Motor)
axes[1].plot(time1, d1, 'b-', linewidth=1.5, label='PI_Data_2', alpha=0.7)
axes[1].plot(time2, d2, 'r-', linewidth=1.5, label='PI_Data_3', alpha=0.7)
axes[1].axhline(y=1050, color='gray', linestyle='--', linewidth=1, alpha=0.5, label='Neutral (1050)')
axes[1].set_ylabel('R Motor\nPWM', fontsize=11, fontweight='bold')
axes[1].set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
axes[1].grid(True, alpha=0.3)
axes[1].legend(loc='upper right', fontsize=9)
axes[1].set_ylim([1040, max(max(d1), max(d2)) + 10])

plt.tight_layout()

# Save figure
output_path = r'c:\Users\prajj\OneDrive - Arizona State University\ASU\PhD\blimp_ws\plots\thrust_commands_over_time.png'
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"Figure saved to: {output_path}")

plt.show()

# Print statistics
print("\n=== Thrust Command Statistics ===")
print("\nPI_Data_2:")
print(f"  L Motor: min={u1.min():.1f}, max={u1.max():.1f}, mean={u1.mean():.1f}, std={u1.std():.1f}")
print(f"  R Motor: min={d1.min():.1f}, max={d1.max():.1f}, mean={d1.mean():.1f}, std={d1.std():.1f}")
print(f"  Duration: {time1[-1]:.2f} seconds, Samples: {len(time1)}")

print("\nPI_Data_3:")
print(f"  L Motor: min={u2.min():.1f}, max={u2.max():.1f}, mean={u2.mean():.1f}, std={u2.std():.1f}")
print(f"  R Motor: min={d2.min():.1f}, max={d2.max():.1f}, mean={d2.mean():.1f}, std={d2.std():.1f}")
print(f"  Duration: {time2[-1]:.2f} seconds, Samples: {len(time2)}")
