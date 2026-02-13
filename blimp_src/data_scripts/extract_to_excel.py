#!/usr/bin/env python3
"""
Extract barometer data from ROS 2 bag and save to Excel file.
Uses recovery mode to handle malformed/corrupted ROS2 bags.
"""

import sqlite3
import struct
import sys
import os

# Try to import pandas and openpyxl
try:
    import pandas as pd
except ImportError:
    print("ERROR: pandas not installed. Run: pip install pandas openpyxl")
    sys.exit(1)


class KalmanFilter1D:
    """Simple 1D Kalman Filter for height estimation."""
    
    def __init__(self, process_variance=0.01, measurement_variance=0.1, initial_value=0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.value = initial_value
        self.estimate_error = 1.0
    
    def update(self, measurement):
        # Prediction
        self.estimate_error = self.estimate_error + self.process_variance
        # Update
        kalman_gain = self.estimate_error / (self.estimate_error + self.measurement_variance)
        self.value = self.value + kalman_gain * (measurement - self.value)
        self.estimate_error = (1 - kalman_gain) * self.estimate_error
        return self.value


# Recovery functions for malformed database
def _read_varint(buf, offset):
    value = 0
    for i in range(9):
        if offset + i >= len(buf):
            raise ValueError("varint out of range")
        b = buf[offset + i]
        if i == 8:
            return (value << 8) | b, offset + 9
        value = (value << 7) | (b & 0x7F)
        if b < 0x80:
            return value, offset + i + 1
    raise ValueError("invalid varint")


def _serial_type_len(serial_type):
    if serial_type == 0:
        return 0
    if serial_type == 1:
        return 1
    if serial_type == 2:
        return 2
    if serial_type == 3:
        return 3
    if serial_type == 4:
        return 4
    if serial_type == 5:
        return 6
    if serial_type == 6:
        return 8
    if serial_type == 7:
        return 8
    if serial_type == 8:
        return 0
    if serial_type == 9:
        return 0
    if serial_type >= 12:
        return (serial_type - 12) // 2
    return 0


def _parse_record(payload):
    header_size, pos = _read_varint(payload, 0)
    serial_types = []
    while pos < header_size:
        serial_type, pos = _read_varint(payload, pos)
        serial_types.append(serial_type)
    data_pos = header_size
    values = []
    for serial_type in serial_types:
        if serial_type == 0:
            values.append(None)
            continue
        if serial_type == 8:
            values.append(0)
            continue
        if serial_type == 9:
            values.append(1)
            continue
        n = _serial_type_len(serial_type)
        if data_pos + n > len(payload):
            raise ValueError("record data out of range")
        raw = payload[data_pos : data_pos + n]
        data_pos += n
        if serial_type in {1, 2, 3, 4, 5, 6}:
            values.append(int.from_bytes(raw, byteorder="big", signed=True))
        elif serial_type == 7:
            values.append(struct.unpack(">d", raw)[0])
        elif serial_type >= 12:
            if serial_type % 2 == 0:
                values.append(bytes(raw))
            else:
                values.append(raw.decode("utf-8", errors="replace"))
        else:
            values.append(bytes(raw))
    return values


def _recover_from_corrupt_db(db_path):
    """Recover data from possibly corrupt SQLite database."""
    with open(db_path, "rb") as handle:
        data = handle.read()

    page_size = int.from_bytes(data[16:18], "big")
    if page_size == 1:
        page_size = 65536
    reserved_bytes = data[20]
    usable_size = page_size - reserved_bytes

    def read_page(page_no):
        start = (page_no - 1) * page_size
        end = start + page_size
        if start < 0 or end > len(data):
            return None
        return data[start:end]

    def iter_table_records(root_page):
        stack = [root_page]
        while stack:
            page_no = stack.pop()
            page = read_page(page_no)
            if page is None:
                continue
            header_offset = 100 if page_no == 1 else 0
            page_type = page[header_offset]
            if page_type not in {0x05, 0x0D}:
                continue
            header_size = 12 if page_type == 0x05 else 8
            cell_count = int.from_bytes(
                page[header_offset + 3 : header_offset + 5], "big"
            )
            cell_ptr_base = header_offset + header_size
            cell_ptrs = [
                int.from_bytes(
                   page[cell_ptr_base + i * 2 : cell_ptr_base + i * 2 + 2], "big"
                )
                for i in range(cell_count)
            ]
            if page_type == 0x05:
                rightmost = int.from_bytes(
                    page[header_offset + 8 : header_offset + 12], "big"
                )
                if rightmost:
                    stack.append(rightmost)
                for cell_ptr in reversed(cell_ptrs):
                    if cell_ptr == 0:
                        continue
                    if cell_ptr + 4 > len(page):
                        continue
                    left_child = int.from_bytes(page[cell_ptr : cell_ptr + 4], "big")
                    if left_child:
                        stack.append(left_child)
                continue

            for cell_ptr in cell_ptrs:
                if cell_ptr == 0 or cell_ptr >= len(page):
                    continue
                try:
                    payload_size, pos = _read_varint(page, cell_ptr)
                    rowid, pos = _read_varint(page, pos)
                except Exception:
                    continue
                payload_start = pos
                if payload_start >= len(page):
                    continue
                payload_end = payload_start + min(payload_size, page_size - pos)
                if payload_end > len(page):
                    continue
                payload = bytearray(page[payload_start:payload_end])
                try:
                    values = _parse_record(bytes(payload))
                except Exception:
                    continue
                yield rowid, values

    schema = {}
    for _, values in iter_table_records(1):
        if len(values) < 5:
            continue
        typ, name, tbl_name, rootpage, sql = values[:5]
        if typ == "table" and name and rootpage:
            schema[name] = {"rootpage": int(rootpage)}

    topics_rows = []
    for rowid, values in iter_table_records(schema["topics"]["rootpage"]):
        if len(values) >= 2:
            topics_rows.append((values[0], values[1]))  # id, name

    msg_rows = []
    for rowid, values in iter_table_records(schema["messages"]["rootpage"]):
        if len(values) >= 3:
            # topic_id, timestamp, data
            topic_id = values[1] if isinstance(values[1], int) else None
            timestamp = values[0] if isinstance(values[0], int) else None
            data_blob = values[2] if isinstance(values[2], (bytes, bytearray)) else None
            if topic_id and timestamp and data_blob:
                msg_rows.append((int(timestamp), bytes(data_blob), int(topic_id)))

    return topics_rows, msg_rows


def try_decode_float64_msg(data):
    """Try to decode as std_msgs/Float64 (4-byte header + 8-byte double)."""
    if len(data) < 12:
        return None
    try:
        val = struct.unpack_from("<d", data, 4)[0]
        if abs(val) < 1e10:
            return val
    except:
        pass
    return None


def extract_barometer_data(db_path, topic_name="/barometer_data"):
    """Extract barometer data from ROS2 bag database (with recovery for malformed DB)."""
    
    rows = []
    topic_id = None
    
    try:
        # Try normal SQLite access first
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Get topic_id
        cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
        row = cursor.fetchone()
        if not row:
            print(f"ERROR: Topic '{topic_name}' not found in bag")
            conn.close()
            return None, None
        
        topic_id = row[0]
        
        # Get all messages for this topic
        cursor.execute("""
            SELECT timestamp, data 
            FROM messages 
            WHERE topic_id=? 
            ORDER BY timestamp
        """, (topic_id,))
        
        rows = cursor.fetchall()
        conn.close()
        print(f"Found {len(rows)} messages in bag (normal mode)")
        
    except sqlite3.DatabaseError as exc:
        print(f"SQLite error: {exc}")
        print("Attempting raw recovery...")
        
        # Use recovery mode
        topics, raw_rows = _recover_from_corrupt_db(db_path)
        
        # Find barometer topic_id
        for t_id, t_name in topics:
            if "barometer" in str(t_name).lower() or "baro" in str(t_name).lower():
                topic_id = t_id
                break
        
        if topic_id is None and topics:
            topic_id = topics[0][0]  # Use first topic
            print(f"Using first topic: {topics[0][1]}")
        
        if topic_id is None:
            print("ERROR: No topics recovered")
            return None, None
        
        # Filter messages for this topic
        rows = [(ts, data) for ts, data, tid in raw_rows if tid == topic_id]
        rows.sort(key=lambda x: x[0])  # Sort by timestamp
        print(f"Recovered {len(rows)} messages (recovery mode)")
    
    if not rows:
        print(f"ERROR: No messages found for topic")
        return None, None
    
    # Decode messages
    timestamps = []
    values = []
    t0 = None
    
    for ts_ns, data in rows:
        if t0 is None:
            t0 = ts_ns
        t_sec = (ts_ns - t0) * 1e-9
        
        # Try to decode as Float64
        v = try_decode_float64_msg(data)
        if v is not None:
            timestamps.append(t_sec)
            values.append(v)
    
    print(f"Decoded {len(values)} data points")
    if values:
        print(f"Value range: {min(values):.4f} to {max(values):.4f}")
        print(f"Time range: {min(timestamps):.2f}s to {max(timestamps):.2f}s")
    
    return timestamps, values


def main():
    # Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bag_file = os.path.join(script_dir, "rosbag2_2026_02_11-12_19_38_0.db3")
    output_dir = os.path.join(script_dir, "..", "..", "logs")
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "barometer_data_exported.xlsx")
    
    print(f"Reading ROS2 bag: {bag_file}")
    
    # Extract data
    timestamps, values = extract_barometer_data(bag_file)
    
    if timestamps is None or not timestamps:
        print("ERROR: No data extracted")
        return 1
    
    # Filter data (60-120 seconds, relabeled as 0-60)
    filtered_data = []
    for ts, val in zip(timestamps, values):
        if 60 <= ts <= 120:
            new_ts = ts - 60
            filtered_data.append({
                'Time (s)': new_ts,
                'Raw Height (m)': val
            })
    
    print(f"Filtered data: {len(filtered_data)} points (60-120s range, relabeled 0-60s)")
    
    # Apply Kalman filter
    if filtered_data:
        kf = KalmanFilter1D(
            process_variance=0.01,
            measurement_variance=0.15,
            initial_value=filtered_data[0]['Raw Height (m)']
        )
        
        for row in filtered_data:
            row['Kalman Filtered Height (m)'] = kf.update(row['Raw Height (m)'])
    
    # Create DataFrame
    df = pd.DataFrame(filtered_data)
    
    # Add statistics
    if not df.empty:
        raw_mean = df['Raw Height (m)'].mean()
        raw_std = df['Raw Height (m)'].std()
        kalman_mean = df['Kalman Filtered Height (m)'].mean()
        kalman_std = df['Kalman Filtered Height (m)'].std()
        
        stats_df = pd.DataFrame({
            'Metric': ['Mean (m)', 'Std Dev (m)', 'Min (m)', 'Max (m)', 'Count'],
            'Raw Height': [
                raw_mean,
                raw_std,
                df['Raw Height (m)'].min(),
                df['Raw Height (m)'].max(),
                len(df)
            ],
            'Kalman Filtered': [
                kalman_mean,
                kalman_std,
                df['Kalman Filtered Height (m)'].min(),
                df['Kalman Filtered Height (m)'].max(),
                len(df)
            ]
        })
    
    # Save to Excel with multiple sheets
    with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
        df.to_excel(writer, sheet_name='Barometer Data', index=False)
        if not df.empty:
            stats_df.to_excel(writer, sheet_name='Statistics', index=False)
        
        # Add metadata sheet
        metadata = pd.DataFrame({
            'Property': ['Source Bag', 'Topic', 'Total Messages', 'Filtered Range', 'Export Date'],
            'Value': [
                os.path.basename(bag_file),
                '/barometer_data',
                len(timestamps),
                '60-120s (relabeled 0-60s)',
                pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')
            ]
        })
        metadata.to_excel(writer, sheet_name='Metadata', index=False)
    
    print(f"\n✓ Data exported to: {output_file}")
    print(f"  - {len(df)} data points")
    print(f"  - 3 sheets: 'Barometer Data', 'Statistics', 'Metadata'")
    
    if not df.empty:
        print(f"\nStatistics:")
        print(f"  Raw Height: {raw_mean:.3f} ± {raw_std:.3f} m")
        print(f"  Kalman Filtered: {kalman_mean:.3f} ± {kalman_std:.3f} m")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
