import matplotlib.pyplot as plt
import numpy as np
import re

def parse_imu_data_file(filepath):
    """Parse the IMU data file and extract all variables."""
    times = []
    euler_roll = []
    euler_pitch = []
    euler_yaw = []
    gyro_x = []
    gyro_y = []
    gyro_z = []
    accel_x = []
    accel_y = []
    accel_z = []
    l_motor = []
    r_motor = []
    u_motor = []
    d_motor = []
    
    with open(filepath, 'r') as f:
        for line in f:
            # Parse line format: Euler Angles: r,p,y Gyro: x,y,z Linear Accel: x,y,z L Motor: val R Motor: val U Motor: val D Motor: val time: val
            euler_match = re.search(r'Euler Angles: ([-\d.]+),([-\d.]+),([-\d.]+)', line)
            gyro_match = re.search(r'Gyro: ([-\d.e+-]+),([-\d.e+-]+),([-\d.e+-]+)', line)
            accel_match = re.search(r'Linear Accel: ([-\d.e+-]+),([-\d.e+-]+),([-\d.e+-]+)', line)
            motor_match = re.search(r'L Motor: ([-\d.e+-]+) R Motor: ([-\d.e+-]+) U Motor: ([-\d.e+-]+) D Motor: ([-\d.e+-]+)', line)
            time_match = re.search(r'time: ([-\d.e+-]+)', line)
            
            if euler_match and gyro_match and accel_match and motor_match and time_match:
                # Euler angles
                euler_roll.append(float(euler_match.group(1)))
                euler_pitch.append(float(euler_match.group(2)))
                euler_yaw.append(float(euler_match.group(3)))
                
                # Gyro
                gyro_x.append(float(gyro_match.group(1)))
                gyro_y.append(float(gyro_match.group(2)))
                gyro_z.append(float(gyro_match.group(3)))
                
                # Acceleration
                accel_x.append(float(accel_match.group(1)))
                accel_y.append(float(accel_match.group(2)))
                accel_z.append(float(accel_match.group(3)))
                
                # Motors
                l_motor.append(float(motor_match.group(1)))
                r_motor.append(float(motor_match.group(2)))
                u_motor.append(float(motor_match.group(3)))
                d_motor.append(float(motor_match.group(4)))
                
                # Time
                times.append(float(time_match.group(1)))
    
    return {
        'time': np.array(times),
        'euler_roll': np.array(euler_roll),
        'euler_pitch': np.array(euler_pitch),
        'euler_yaw': np.array(euler_yaw),
        'gyro_x': np.array(gyro_x),
        'gyro_y': np.array(gyro_y),
        'gyro_z': np.array(gyro_z),
        'accel_x': np.array(accel_x),
        'accel_y': np.array(accel_y),
        'accel_z': np.array(accel_z),
        'l_motor': np.array(l_motor),
        'r_motor': np.array(r_motor),
        'u_motor': np.array(u_motor),
        'd_motor': np.array(d_motor)
    }

# Parse all data files
data_dir = r'c:\Users\prajj\OneDrive - Arizona State University\ASU\PhD\blimp_ws\blimp_src\logs'
data4 = parse_imu_data_file(f'{data_dir}\\data_record_4.txt')
data5 = parse_imu_data_file(f'{data_dir}\\data_record_5.txt')

datasets = [data4, data5]
labels = ['Test 1', 'Test 2']
colors = ['blue', 'red']

# Create comprehensive dashboard
fig = plt.figure(figsize=(16, 18))
gs = fig.add_gridspec(7, 2, hspace=0.3, wspace=0.25)

# Title
fig.suptitle('Comprehensive IMU + Motor Control Dashboard', fontsize=18, fontweight='bold', y=0.995)

# Row 1: Euler Angles
ax1 = fig.add_subplot(gs[0, 0])
ax2 = fig.add_subplot(gs[0, 1])
ax3 = fig.add_subplot(gs[1, 0])

for i, (data, label, color) in enumerate(zip(datasets, labels, colors)):
    ax1.plot(data['time'], data['euler_roll'], linewidth=1.5, label=label, color=color, alpha=0.7)
    ax2.plot(data['time'], data['euler_pitch'], linewidth=1.5, label=label, color=color, alpha=0.7)
    ax3.plot(data['time'], data['euler_yaw'], linewidth=1.5, label=label, color=color, alpha=0.7)

ax1.set_ylabel('Roll (°)', fontsize=10, fontweight='bold')
ax1.set_title('Euler Angle - Roll', fontsize=11, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend(loc='best', fontsize=8)

ax2.set_ylabel('Pitch (°)', fontsize=10, fontweight='bold')
ax2.set_title('Euler Angle - Pitch', fontsize=11, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend(loc='best', fontsize=8)

ax3.set_ylabel('Yaw (°)', fontsize=10, fontweight='bold')
ax3.set_title('Euler Angle - Yaw', fontsize=11, fontweight='bold')
ax3.grid(True, alpha=0.3)
ax3.legend(loc='best', fontsize=8)

# Row 2: Gyro Data
ax4 = fig.add_subplot(gs[1, 1])
ax5 = fig.add_subplot(gs[2, 0])
ax6 = fig.add_subplot(gs[2, 1])

for i, (data, label, color) in enumerate(zip(datasets, labels, colors)):
    ax4.plot(data['time'], data['gyro_x'], linewidth=1.5, label=label, color=color, alpha=0.7)
    ax5.plot(data['time'], data['gyro_y'], linewidth=1.5, label=label, color=color, alpha=0.7)
    ax6.plot(data['time'], data['gyro_z'], linewidth=1.5, label=label, color=color, alpha=0.7)

ax4.set_ylabel('ω_x (rad/s)', fontsize=10, fontweight='bold')
ax4.set_title('Gyro - X', fontsize=11, fontweight='bold')
ax4.grid(True, alpha=0.3)
ax4.legend(loc='best', fontsize=8)

ax5.set_ylabel('ω_y (rad/s)', fontsize=10, fontweight='bold')
ax5.set_title('Gyro - Y', fontsize=11, fontweight='bold')
ax5.grid(True, alpha=0.3)
ax5.legend(loc='best', fontsize=8)

ax6.set_ylabel('ω_z (rad/s)', fontsize=10, fontweight='bold')
ax6.set_title('Gyro - Z', fontsize=11, fontweight='bold')
ax6.grid(True, alpha=0.3)
ax6.legend(loc='best', fontsize=8)

# Row 3: Linear Acceleration
ax7 = fig.add_subplot(gs[3, 0])
ax8 = fig.add_subplot(gs[3, 1])
ax9 = fig.add_subplot(gs[4, 0])

for i, (data, label, color) in enumerate(zip(datasets, labels, colors)):
    ax7.plot(data['time'], data['accel_x'], linewidth=1.5, label=label, color=color, alpha=0.7)
    ax8.plot(data['time'], data['accel_y'], linewidth=1.5, label=label, color=color, alpha=0.7)
    ax9.plot(data['time'], data['accel_z'], linewidth=1.5, label=label, color=color, alpha=0.7)

ax7.set_ylabel('a_x (m/s²)', fontsize=10, fontweight='bold')
ax7.set_title('Linear Acceleration - X', fontsize=11, fontweight='bold')
ax7.grid(True, alpha=0.3)
ax7.legend(loc='best', fontsize=8)

ax8.set_ylabel('a_y (m/s²)', fontsize=10, fontweight='bold')
ax8.set_title('Linear Acceleration - Y', fontsize=11, fontweight='bold')
ax8.grid(True, alpha=0.3)
ax8.legend(loc='best', fontsize=8)

ax9.set_ylabel('a_z (m/s²)', fontsize=10, fontweight='bold')
ax9.set_title('Linear Acceleration - Z', fontsize=11, fontweight='bold')
ax9.grid(True, alpha=0.3)
ax9.legend(loc='best', fontsize=8)

# Row 4-6: Motor Commands
ax10 = fig.add_subplot(gs[4, 1])
ax11 = fig.add_subplot(gs[5, 0])
ax12 = fig.add_subplot(gs[5, 1])
ax13 = fig.add_subplot(gs[6, 0])

for i, (data, label, color) in enumerate(zip(datasets, labels, colors)):
    ax10.plot(data['time'], data['l_motor'], linewidth=1.5, label=label, color=color, alpha=0.7)
    ax11.plot(data['time'], data['r_motor'], linewidth=1.5, label=label, color=color, alpha=0.7)
    ax12.plot(data['time'], data['u_motor'], linewidth=1.5, label=label, color=color, alpha=0.7)
    ax13.plot(data['time'], data['d_motor'], linewidth=1.5, label=label, color=color, alpha=0.7)

# Add neutral line for motors
for ax in [ax10, ax11, ax12, ax13]:
    ax.axhline(y=1050, color='gray', linestyle='--', linewidth=1, alpha=0.5)

ax10.set_ylabel('L Motor PWM', fontsize=10, fontweight='bold')
ax10.set_title('Left Motor Command', fontsize=11, fontweight='bold')
ax10.grid(True, alpha=0.3)
ax10.legend(loc='best', fontsize=8)

ax11.set_ylabel('R Motor PWM', fontsize=10, fontweight='bold')
ax11.set_title('Right Motor Command', fontsize=11, fontweight='bold')
ax11.grid(True, alpha=0.3)
ax11.legend(loc='best', fontsize=8)

ax12.set_ylabel('U Motor PWM', fontsize=10, fontweight='bold')
ax12.set_title('Upper Motor Command', fontsize=11, fontweight='bold')
ax12.grid(True, alpha=0.3)
ax12.legend(loc='best', fontsize=8)

ax13.set_ylabel('D Motor PWM', fontsize=10, fontweight='bold')
ax13.set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')
ax13.set_title('Down Motor Command', fontsize=11, fontweight='bold')
ax13.grid(True, alpha=0.3)
ax13.legend(loc='best', fontsize=8)

# Add x-label to bottom plots
ax11.set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')
ax12.set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')

# Save figure
output_path = r'c:\Users\prajj\OneDrive - Arizona State University\ASU\PhD\blimp_ws\plots\imu_motor_dashboard.png'
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"Dashboard saved to: {output_path}")

plt.show()

# Print statistics
print("\n" + "="*80)
print("IMU + MOTOR CONTROL STATISTICS")
print("="*80)

for data, label in zip(datasets, labels):
    print(f"\n{label}:")
    print(f"  Duration: {data['time'][-1]:.2f} seconds, Samples: {len(data['time'])}")
    print(f"\n  Euler Angles (degrees):")
    print(f"    Roll:  mean={data['euler_roll'].mean():.2f}, std={data['euler_roll'].std():.2f}, range=[{data['euler_roll'].min():.2f}, {data['euler_roll'].max():.2f}]")
    print(f"    Pitch: mean={data['euler_pitch'].mean():.2f}, std={data['euler_pitch'].std():.2f}, range=[{data['euler_pitch'].min():.2f}, {data['euler_pitch'].max():.2f}]")
    print(f"    Yaw:   mean={data['euler_yaw'].mean():.2f}, std={data['euler_yaw'].std():.2f}, range=[{data['euler_yaw'].min():.2f}, {data['euler_yaw'].max():.2f}]")
    
    print(f"\n  Gyro (rad/s):")
    print(f"    ω_x: mean={data['gyro_x'].mean():.4f}, std={data['gyro_x'].std():.4f}")
    print(f"    ω_y: mean={data['gyro_y'].mean():.4f}, std={data['gyro_y'].std():.4f}")
    print(f"    ω_z: mean={data['gyro_z'].mean():.4f}, std={data['gyro_z'].std():.4f}")
    
    print(f"\n  Linear Accel (m/s²):")
    print(f"    a_x: mean={data['accel_x'].mean():.4f}, std={data['accel_x'].std():.4f}")
    print(f"    a_y: mean={data['accel_y'].mean():.4f}, std={data['accel_y'].std():.4f}")
    print(f"    a_z: mean={data['accel_z'].mean():.4f}, std={data['accel_z'].std():.4f}")
    
    print(f"\n  Motor Commands (PWM):")
    print(f"    L Motor: mean={data['l_motor'].mean():.1f}, std={data['l_motor'].std():.1f}, range=[{data['l_motor'].min():.1f}, {data['l_motor'].max():.1f}]")
    print(f"    R Motor: mean={data['r_motor'].mean():.1f}, std={data['r_motor'].std():.1f}, range=[{data['r_motor'].min():.1f}, {data['r_motor'].max():.1f}]")
    print(f"    U Motor: mean={data['u_motor'].mean():.1f}, std={data['u_motor'].std():.1f}, range=[{data['u_motor'].min():.1f}, {data['u_motor'].max():.1f}]")
    print(f"    D Motor: mean={data['d_motor'].mean():.1f}, std={data['d_motor'].std():.1f}, range=[{data['d_motor'].min():.1f}, {data['d_motor'].max():.1f}]")

print("\n" + "="*80)
