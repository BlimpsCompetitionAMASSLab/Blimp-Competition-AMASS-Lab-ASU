#!/usr/bin/env python3
"""
Extract barometer data from a ROS 2 rosbag2 SQLite .db3 file and plot it.
This script reads the bag directly via SQLite + CDR deserialization,
so it does NOT require a full ROS 2 installation.
"""

import sqlite3
import struct
import os
import sys

import matplotlib.pyplot as plt
import numpy as np


class KalmanFilter1D:
    """Simple 1D Kalman Filter for height estimation."""
    
    def __init__(self, process_variance=0.01, measurement_variance=0.1, initial_value=0):
        """
        Initialize Kalman Filter.
        
        Args:
            process_variance: How much the true value is expected to change between steps
            measurement_variance: Measurement noise variance
            initial_value: Initial estimate
        """
        self.process_variance = process_variance  # Q
        self.measurement_variance = measurement_variance  # R
        self.value = initial_value
        self.estimate_error = 1.0  # P
    
    def update(self, measurement):
        """
        Update filter with new measurement and return filtered value.
        
        Args:
            measurement: New measurement value
            
        Returns:
            Filtered value
        """
        # Prediction step
        self.estimate_error = self.estimate_error + self.process_variance
        
        # Update step
        kalman_gain = self.estimate_error / (self.estimate_error + self.measurement_variance)
        self.value = self.value + kalman_gain * (measurement - self.value)
        self.estimate_error = (1 - kalman_gain) * self.estimate_error
        
        return self.value


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
    if header_size < 1 or header_size > len(payload):
        raise ValueError("invalid record header size")
    header_end = header_size
    serial_types = []
    while pos < header_end:
        serial_type, pos = _read_varint(payload, pos)
        serial_types.append(serial_type)
    data_pos = header_end
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


def _local_payload_size(payload_size, usable_size):
    max_local = usable_size - 35
    min_local = ((usable_size - 12) * 32 // 255) - 23
    if payload_size <= max_local:
        return payload_size
    local = min_local + ((payload_size - min_local) % (usable_size - 4))
    if local > max_local:
        local = min_local
    return local


def _parse_create_table(sql):
    if not sql:
        return [], None
    sql = sql.strip()
    if "(" not in sql or ")" not in sql:
        return [], None
    inside = sql[sql.find("(") + 1 : sql.rfind(")")]
    parts = [p.strip() for p in inside.split(",") if p.strip()]
    cols = []
    pk_col = None
    for part in parts:
        tokens = part.split()
        if not tokens:
            continue
        col = tokens[0].strip("\"`[]")
        cols.append(col)
        lowered = part.lower()
        if "primary key" in lowered and "integer" in lowered:
            pk_col = col
    return cols, pk_col


def _recover_from_corrupt_db(db_path):
    with open(db_path, "rb") as handle:
        data = handle.read()

    if len(data) < 100 or not data.startswith(b"SQLite format 3"):
        raise RuntimeError("Not a valid SQLite database file")

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

    def read_overflow(page_no, remaining):
        payload = bytearray()
        next_page = page_no
        while next_page and remaining > 0:
            page = read_page(next_page)
            next_page = int.from_bytes(page[0:4], "big")
            chunk = page[4 : 4 + min(remaining, page_size - 4)]
            payload.extend(chunk)
            remaining -= len(chunk)
        return bytes(payload)

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
                local_payload = _local_payload_size(payload_size, usable_size)
                payload_end = payload_start + min(local_payload, payload_size)
                if payload_end > len(page):
                    continue
                payload = bytearray(page[payload_start:payload_end])
                remaining = payload_size - len(payload)
                if remaining > 0:
                    overflow_ptr_offset = payload_end
                    if overflow_ptr_offset + 4 <= len(page):
                        overflow_page = int.from_bytes(
                            page[overflow_ptr_offset : overflow_ptr_offset + 4], "big"
                        )
                        if overflow_page:
                            payload.extend(read_overflow(overflow_page, remaining))
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
            schema[name] = {"rootpage": int(rootpage), "sql": sql}

    if "topics" not in schema or "messages" not in schema:
        raise RuntimeError("Could not recover topics/messages tables from database")

    topics_sql = schema["topics"]["sql"]
    topics_cols, topics_pk = _parse_create_table(topics_sql)
    topics_rows = []
    for rowid, values in iter_table_records(schema["topics"]["rootpage"]):
        values = list(values) + [None] * (len(topics_cols) - len(values))
        row = dict(zip(topics_cols, values))
        if topics_pk and row.get(topics_pk) is None:
            row[topics_pk] = rowid
        topics_rows.append(
            (
                row.get("id"),
                row.get("name"),
                row.get("type"),
                row.get("serialization_format"),
                row.get("offered_qos_profiles"),
            )
        )

    messages_sql = schema["messages"]["sql"]
    msg_cols, msg_pk = _parse_create_table(messages_sql)
    msg_idx = {name: idx for idx, name in enumerate(msg_cols)}

    rows = []
    for rowid, values in iter_table_records(schema["messages"]["rootpage"]):
        values = list(values) + [None] * (len(msg_cols) - len(values))
        if msg_pk:
            pk_idx = msg_idx.get(msg_pk)
            if pk_idx is not None and values[pk_idx] is None:
                values[pk_idx] = rowid
        topic_id = values[msg_idx.get("topic_id", -1)]
        timestamp = values[msg_idx.get("timestamp", -1)]
        data_blob = values[msg_idx.get("data", -1)]
        if (
            topic_id is None
            or timestamp is None
            or not isinstance(data_blob, (bytes, bytearray))
        ):
            continue
        rows.append((int(timestamp), bytes(data_blob), int(topic_id)))

    return topics_rows, rows

# --- Configuration ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(SCRIPT_DIR, "rosbag2_2026_02_11-12_19_38_0.db3")
OUT_IMAGE = os.path.join(SCRIPT_DIR, "barometer_plot.png")

# ---------------------------------------------------------------------
# 1.  Open the SQLite database and inspect contents
# ---------------------------------------------------------------------
topics = []
rows = []
topic_id = None
topic_name = None
topic_type = None

try:
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()

    # List tables
    cur.execute("SELECT name FROM sqlite_master WHERE type='table'")
    tables = [r[0] for r in cur.fetchall()]
    print(f"Tables: {tables}")

    # Topics
    cur.execute("SELECT * FROM topics")
    topics = cur.fetchall()
    print(f"\nTopics:")
    for t in topics:
        print(f"  {t}")

    # Message count
    cur.execute("SELECT COUNT(*) FROM messages")
    msg_count = cur.fetchone()[0]
    print(f"\nTotal messages: {msg_count}")

    # Identify the barometer topic id
    for t in topics:
        # topics table columns: id, name, type, serialization_format, offered_qos_profiles
        if (
            "barometer" in t[1].lower()
            or "baro" in t[1].lower()
            or "pressure" in t[1].lower()
            or "fluid" in t[1].lower()
        ):
            topic_id = t[0]
            topic_name = t[1]
            topic_type = t[2]
            break

    # If no barometer topic found, just use the first (or only) topic
    if topic_id is None and len(topics) > 0:
        topic_id = topics[0][0]
        topic_name = topics[0][1]
        topic_type = topics[0][2]
        print(
            f"\nNo barometer-specific topic found. Using first topic: {topic_name} ({topic_type})"
        )
    else:
        print(f"\nUsing topic: {topic_name} ({topic_type})")

    if topic_id is None:
        print("No topics found!", file=sys.stderr)
        sys.exit(1)

    # Fetch all messages for this topic
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,),
    )
    rows = cur.fetchall()
    conn.close()
    print(f"Messages on topic: {len(rows)}")
except sqlite3.DatabaseError as exc:
    print(f"SQLite error: {exc}. Attempting raw recovery...")
    topics, raw_rows = _recover_from_corrupt_db(DB_PATH)
    if not topics:
        print("No topics recovered from database.", file=sys.stderr)
        sys.exit(1)
    for t in topics:
        if t[1] and (
            "barometer" in t[1].lower()
            or "baro" in t[1].lower()
            or "pressure" in t[1].lower()
            or "fluid" in t[1].lower()
        ):
            topic_id = t[0]
            topic_name = t[1]
            topic_type = t[2]
            break
    if topic_id is None:
        topic_id = topics[0][0]
        topic_name = topics[0][1]
        topic_type = topics[0][2]
        print(
            f"\nNo barometer-specific topic found. Using first topic: {topic_name} ({topic_type})"
        )
    else:
        print(f"\nUsing topic: {topic_name} ({topic_type})")

    rows = [(ts, data) for ts, data, tid in raw_rows if tid == topic_id]
    rows.sort(key=lambda item: item[0])
    print(f"Messages on topic (recovered): {len(rows)}")

# ---------------------------------------------------------------------
# 2.  Decode CDR data
# ---------------------------------------------------------------------
# ROS 2 messages are serialized in CDR. We'll try to decode the first
# message to figure out the structure, then decode all of them.

if not rows:
    print("No messages to plot!", file=sys.stderr)
    sys.exit(1)

# Print a hex dump of the first message for debugging
first_data = rows[0][1]
print(f"\nFirst message raw length: {len(first_data)} bytes")
print("Hex dump (first 128 bytes):")
hex_str = first_data[:128].hex()
print(" ".join(hex_str[i:i+2] for i in range(0, len(hex_str), 2)))


def try_decode_float64_msg(data):
    """Try to decode as std_msgs/Float64 (CDR: 4-byte header + 8-byte float64)."""
    if len(data) >= 12:
        val = struct.unpack_from("<d", data, 4)[0]
        return val
    return None


def try_decode_float32_msg(data):
    """Try to decode as std_msgs/Float32 (CDR: 4-byte header + 4-byte float32)."""
    if len(data) >= 8:
        val = struct.unpack_from("<f", data, 4)[0]
        return val
    return None


def try_decode_fluidpressure(data):
    """
    Try to decode as sensor_msgs/FluidPressure:
      CDR header (4 bytes)
      std_msgs/Header:
        stamp.sec (int32)   - 4 bytes
        stamp.nanosec (uint32) - 4 bytes
        frame_id (string: 4-byte len + chars + padding)
      fluid_pressure (float64)
      variance (float64)
    """
    if len(data) < 20:
        return None, None
    offset = 4  # skip CDR header
    # Header stamp
    sec = struct.unpack_from("<i", data, offset)[0]
    offset += 4
    nanosec = struct.unpack_from("<I", data, offset)[0]
    offset += 4
    # frame_id string
    str_len = struct.unpack_from("<I", data, offset)[0]
    offset += 4
    frame_id = data[offset : offset + str_len].decode("utf-8", errors="replace").rstrip("\x00")
    offset += str_len
    # Align to 8-byte boundary for float64
    if offset % 8 != 0:
        offset += 8 - (offset % 8)
    if offset + 8 > len(data):
        return None, None
    pressure = struct.unpack_from("<d", data, offset)[0]
    return pressure, frame_id


def try_decode_custom_barometer(data):
    """
    Try to decode a custom barometer message that might have:
      CDR header (4 bytes)
      Header (stamp + frame_id)
      height (float64) or pressure (float64) or temperature (float64)
    Returns dict of extracted float64 values after the header.
    """
    if len(data) < 12:
        return None
    offset = 4  # skip CDR header
    
    # Try to see if it starts with a Header (stamp sec/nanosec + string)
    sec = struct.unpack_from("<i", data, offset)[0]
    nanosec = struct.unpack_from("<I", data, offset + 4)[0]
    
    # Check if sec looks like a reasonable timestamp (year ~2025-2026 epoch)
    has_header = 1700000000 < sec < 1900000000
    
    if has_header:
        offset += 8  # skip stamp
        if offset + 4 > len(data):
            has_header = False
            offset = 4
        else:
            str_len = struct.unpack_from("<I", data, offset)[0]
            offset += 4
            if str_len > 256 or offset + str_len > len(data):
                has_header = False
                offset = 4
            else:
                offset += str_len
                # Align to 8 bytes
                if offset % 8 != 0:
                    offset += 8 - (offset % 8)
    
    # Now extract all remaining float64 values
    values = []
    while offset + 8 <= len(data):
        val = struct.unpack_from("<d", data, offset)[0]
        values.append(val)
        offset += 8
    
    return {"has_header": has_header, "sec": sec if has_header else None, 
            "nanosec": nanosec if has_header else None, "float64_values": values}


# Try each decoder
print("\n--- Attempting to decode first message ---")

# Try FluidPressure
pressure, frame_id = try_decode_fluidpressure(first_data)
print(f"FluidPressure decode: pressure={pressure}, frame_id='{frame_id}'")

# Try custom
custom = try_decode_custom_barometer(first_data)
print(f"Custom decode: {custom}")

# Try simple Float64/Float32
f64 = try_decode_float64_msg(first_data)
f32 = try_decode_float32_msg(first_data)
print(f"Float64 decode: {f64}")
print(f"Float32 decode: {f32}")

# Also print ALL float64 values we can find at every offset
print("\nAll possible float64 values at various offsets:")
for off in range(0, min(len(first_data) - 7, 64)):
    val = struct.unpack_from("<d", first_data, off)[0]
    if abs(val) < 1e10 and abs(val) > 1e-10 and not (val != val):  # skip NaN and extreme
        print(f"  offset {off}: {val}")

# =====================================================================
# 3.  Decode messages
# =====================================================================
# Determine best decoder based on topic type
print(f"\nTopic type string: '{topic_type}'")

timestamps = []
values = []
t0 = None

for ts_ns, data in rows:
    if t0 is None:
        t0 = ts_ns
    t_sec = (ts_ns - t0) * 1e-9
    
    if "FluidPressure" in (topic_type or ""):
        p, _ = try_decode_fluidpressure(data)
        if p is not None:
            timestamps.append(t_sec)
            values.append(p)
    elif "Float64" in (topic_type or ""):
        v = try_decode_float64_msg(data)
        if v is not None:
            timestamps.append(t_sec)
            values.append(v)
    elif "Float32" in (topic_type or ""):
        v = try_decode_float32_msg(data)
        if v is not None:
            timestamps.append(t_sec)
            values.append(v)
    else:
        # Custom or unknown - use the custom decoder, pick the first float64
        result = try_decode_custom_barometer(data)
        if result and result["float64_values"]:
            timestamps.append(t_sec)
            values.append(result["float64_values"][0])

if not values:
    print("Could not decode any values! Trying brute-force float64 at offset 4...")
    for ts_ns, data in rows:
        if t0 is None:
            t0 = ts_ns
        t_sec = (ts_ns - t0) * 1e-9
        v = try_decode_float64_msg(data)
        if v is not None:
            timestamps.append(t_sec)
            values.append(v)

print(f"\nDecoded {len(values)} data points.")
if values:
    print(f"Value range: {min(values):.4f} to {max(values):.4f}")
    print(f"Time range: {min(timestamps):.2f}s to {max(timestamps):.2f}s")

# =====================================================================
# 4.  Filter data
# =====================================================================
filtered_timestamps = []
filtered_values = []

for ts, val in zip(timestamps, values):
    # Keep only data from 60 to 120 seconds
    if 60 <= ts <= 120:
        # Relabel as 0 to 60 seconds
        new_ts = ts - 60
        filtered_timestamps.append(new_ts)
        filtered_values.append(val)

print(f"\nFiltered data: {len(filtered_values)} points (from 0-60 sec range.)")

# =====================================================================
# 5.  Apply Kalman Filter
# =====================================================================
kf = KalmanFilter1D(process_variance=0.01, measurement_variance=0.15, initial_value=filtered_values[0] if filtered_values else 0)
kalman_filtered = []
for val in filtered_values:
    kalman_filtered.append(kf.update(val))

# =====================================================================
# 6.  Plot
# =====================================================================
if filtered_values:
    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Plot raw data
    ax.plot(filtered_timestamps, filtered_values, linewidth=1.0, color="#2196F3", 
            alpha=0.6, label="Raw measurements")
    
    # Plot Kalman filtered data
    ax.plot(filtered_timestamps, kalman_filtered, linewidth=2, color="#FF6F00", 
            label="Kalman filtered")
    
    # Target height and margins
    target_height = 2.0
    margin = 0.3  # ±0.3 m from target
    
    ax.axhline(y=target_height, color="#4CAF50", linestyle="--", linewidth=2, 
               label=f"Target height ({target_height}m)")
    ax.fill_between(filtered_timestamps, target_height - margin, target_height + margin, 
                     color="#4CAF50", alpha=0.2, label=f"Acceptable margin (±{margin}m)")
    
    ax.set_xlabel("Time (s)", fontsize=12, fontweight="bold")
    
    # Smart y-label
    if "FluidPressure" in (topic_type or ""):
        ax.set_ylabel("Pressure (Pa)", fontsize=12, fontweight="bold")
    elif "height" in (topic_name or "").lower():
        ax.set_ylabel("Height (m)", fontsize=12, fontweight="bold")
    else:
        ax.set_ylabel("Value", fontsize=12, fontweight="bold")
    
    ax.set_title(f"Blimp Height Control - {topic_name}\n(Data: 0-60 sec)", 
                 fontsize=13, fontweight="bold")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=10)
    ax.set_xlim(0, 60)
    
    fig.tight_layout()
    
    fig.savefig(OUT_IMAGE, dpi=150, bbox_inches="tight")
    print(f"\nPlot saved to: {OUT_IMAGE}")
    plt.show()
else:
    print("\nNo data in the 0-60 second range to plot.", file=sys.stderr)
    sys.exit(1)
