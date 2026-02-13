#!/usr/bin/env python3
"""
Add Excel export capability to the existing barometer extraction.
Run this after extract_and_plot.py has successfully extracted the data.
"""

import sys
import os
import subprocess

# Check pandas
try:
    import pandas as pd
except ImportError:
    print("Installing pandas and openpyxl...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pandas", "openpyxl"])
    import pandas as pd

# Run the original extraction script and capture its data
import importlib.util

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
extract_script_path = os.path.join(SCRIPT_DIR, "extract_and_plot.py")

# Load the module
spec = importlib.util.spec_from_file_location("extract_module", extract_script_path)
extract_module = importlib.util.module_from_spec(spec)

# Capture the data by modifying matplotlib to not show and getting variables
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend

# Execute the extraction script
print("Running extraction script...")
sys.path.insert(0, SCRIPT_DIR)
exec(open(extract_script_path).read())

# At this point, filtered_timestamps, filtered_values, and kalman_filtered should be available
if 'filtered_timestamps' in locals() and 'filtered_values' in locals():
    print(f"\nExtracting {len(filtered_timestamps)} data points to Excel...")
    
    # Create DataFrame
    df = pd.DataFrame({
        'Time (s)': filtered_timestamps,
        'Raw Height (m)': filtered_values,
        'Kalman Filtered Height (m)': kalman_filtered
    })
    
    # Calculate statistics
    stats_df = pd.DataFrame({
        'Metric': ['Mean (m)', 'Std Dev (m)', 'Min (m)', 'Max (m)', 'Count'],
        'Raw Height': [
            df['Raw Height (m)'].mean(),
            df['Raw Height (m)'].std(),
            df['Raw Height (m)'].min(),
            df['Raw Height (m)'].max(),
            len(df)
        ],
        'Kalman Filtered': [
            df['Kalman Filtered Height (m)'].mean(),
            df['Kalman Filtered Height (m)'].std(),
            df['Kalman Filtered Height (m)'].min(),
            df['Kalman Filtered Height (m)'].max(),
            len(df)
        ]
    })
    
    # Output path
    output_dir = os.path.join(SCRIPT_DIR, "..", "..", "logs")
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "barometer_data_exported.xlsx")
    
    # Save to Excel
    with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
        df.to_excel(writer, sheet_name='Barometer Data', index=False)
        stats_df.to_excel(writer, sheet_name='Statistics', index=False)
        
        # Metadata
        metadata = pd.DataFrame({
            'Property': ['Source Bag', 'Topic', 'Total Messages', 'Filtered Range', 'Export Date'],
            'Value': [
                'rosbag2_2026_02_11-12_19_38_0.db3',
                '/barometer_data',
                len(timestamps) if 'timestamps' in locals() else 'N/A',
                '60-120s (relabeled 0-60s)',
                pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')
            ]
        })
        metadata.to_excel(writer, sheet_name='Metadata', index=False)
    
    print(f"\n✅ Excel file created: {output_file}")
    print(f"   - {len(df)} data points")
    print(f"   - 3 sheets: Barometer Data, Statistics, Metadata")
    print(f"\nStatistics:")
    print(f"  Raw Height: {df['Raw Height (m)'].mean():.3f} ± {df['Raw Height (m)'].std():.3f} m")
    print(f"  Kalman Filtered: {df['Kalman Filtered Height (m)'].mean():.3f} ± {df['Kalman Filtered Height (m)'].std():.3f} m")
else:
    print("ERROR: Could not extract data variables from script")
    sys.exit(1)
