#! /usr/bin/env python3

from pyocd.core.helpers import ConnectHelper
import struct
import time
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# Location of the RTT control block (update this!)
RTT_CB_ADDR = 0x20000000  # default, might need adjustment

# Unique marker
RTT_ID = b'SEGGER RTT'

# Generic function to read a null-terminated C string from memory
def read_c_string(target, address, max_length=64):
    """
    Read a null-terminated C string from the target memory.
    
    Args:
        target: PyOCD target object
        address: Memory address to read from
        max_length: Maximum number of bytes to read (default 64)
    
    Returns:
        Decoded string or "Invalid" if reading fails
    """
    if address == 0:
        return "NULL"
    
    try:
        # Read bytes from memory
        string_bytes = target.read_memory_block8(address, max_length)
        # Find null terminator
        null_pos = next((i for i, b in enumerate(string_bytes) if b == 0), len(string_bytes))
        # Decode to string
        return bytes(string_bytes[:null_pos]).decode('utf-8', errors='replace')
    except:
        return "Invalid"

class RTTBuffer:
    """Class representing an RTT buffer (up or down)"""
    
    def __init__(self, target, base_address, buffer_index, direction='up'):
        self.target = target
        self.base_address = base_address
        self.buffer_index = buffer_index
        self.direction = direction  # 'up' or 'down'
        self._refresh_fields()
        self.is_scope_buffer = False
        self.scope_format = []
        self.packet_size = 0
        self._parse_scope_format()

    def _parse_scope_format(self):
        """Parse buffer name to determine if it's a scope buffer with structured data"""
        if not self.name or self.name in ["NULL", "Invalid"]:
            return
        
        # Pattern to match scope format: combination of b, f, u1, u2, u4, i1, i2, i4
        pattern = r'^(b|f|u[124]|i[124])+$'
        
        if not re.match(pattern, self.name):
            return
        
        # Parse individual format specifiers
        format_pattern = r'(b|f|u[124]|i[124])'
        matches = re.findall(format_pattern, self.name)
        
        if not matches:
            return
        
        self.is_scope_buffer = True
        self.scope_format = []
        self.packet_size = 0
        
        for match in matches:
            if match == 'b':
                self.scope_format.append(('bool', 1, 'B'))  # unsigned char for bool
                self.packet_size += 1
            elif match == 'f':
                self.scope_format.append(('float', 4, 'f'))  # float
                self.packet_size += 4
            elif match.startswith('u'):
                size = int(match[1])
                if size == 1:
                    self.scope_format.append(('uint8', 1, 'B'))
                elif size == 2:
                    self.scope_format.append(('uint16', 2, 'H'))
                elif size == 4:
                    self.scope_format.append(('uint32', 4, 'I'))
                self.packet_size += size
            elif match.startswith('i'):
                size = int(match[1])
                if size == 1:
                    self.scope_format.append(('int8', 1, 'b'))
                elif size == 2:
                    self.scope_format.append(('int16', 2, 'h'))
                elif size == 4:
                    self.scope_format.append(('int32', 4, 'i'))
                self.packet_size += size
    
    def _refresh_fields(self):
        """Read current buffer state from memory"""
        self.name_ptr = self.target.read32(self.base_address + 0)
        self.buf_ptr = self.target.read32(self.base_address + 4)
        self.buf_size = self.target.read32(self.base_address + 8)
        self.wr_off = self.target.read32(self.base_address + 12)
        self.rd_off = self.target.read32(self.base_address + 16)
        self.flags = self.target.read32(self.base_address + 20)
        self.name = read_c_string(self.target, self.name_ptr, 32)
    
    def read_data(self):
        """Read available data from the buffer and update read offset"""
        self._refresh_fields()

        if(self.direction == 'down'):
            raise NotImplementedError("Reading from a down buffer is not supported!")
        
        if self.wr_off == self.rd_off:
            return b''

        if self.wr_off > self.rd_off:
            size = self.wr_off - self.rd_off
            data = self.target.read_memory_block8(self.buf_ptr + self.rd_off, size)
            new_rd_off = self.wr_off
        else:
            # Wrap-around
            size1 = self.buf_size - self.rd_off
            size2 = self.wr_off
            part1 = self.target.read_memory_block8(self.buf_ptr + self.rd_off, size1)
            part2 = self.target.read_memory_block8(self.buf_ptr, size2)
            data = part1 + part2
            new_rd_off = self.wr_off

        # Write the updated read offset back to the buffer structure
        self.target.write32(self.base_address + 16, new_rd_off)
        self.rd_off = new_rd_off

        return bytes(data)
    
    def read_scope_data(self):
        """Read and parse scope data according to buffer format"""
        if not self.is_scope_buffer:
            raise ValueError(f"Buffer '{self.name}' is not a valid scope buffer. Name must match pattern like 'u4u2b'")
        
        if self.packet_size == 0:
            raise ValueError(f"Invalid scope format for buffer '{self.name}'")
        
        raw_data = self.read_data()
        if not raw_data:
            return []
        
        # Check if we have complete packets
        if len(raw_data) % self.packet_size != 0:
            print(f"Warning: Incomplete packet data. Got {len(raw_data)} bytes, expected multiple of {self.packet_size}")
            # Truncate to complete packets
            raw_data = raw_data[:len(raw_data) - (len(raw_data) % self.packet_size)]
        
        packets = []
        num_packets = len(raw_data) // self.packet_size
        
        for packet_idx in range(num_packets):
            packet_start = packet_idx * self.packet_size
            packet_data = raw_data[packet_start:packet_start + self.packet_size]
            
            parsed_packet = []
            offset = 0
            
            for field_type, field_size, struct_format in self.scope_format:
                field_data = packet_data[offset:offset + field_size]
                
                # Unpack the data according to format (little-endian)
                try:
                    value = struct.unpack('<' + struct_format, field_data)[0]
                    
                    # Convert bool values to actual boolean
                    if field_type == 'bool':
                        value = bool(value)
                    
                    parsed_packet.append({
                        'type': field_type,
                        'value': value
                    })
                except struct.error as e:
                    raise ValueError(f"Failed to parse {field_type} at offset {offset}: {e}")
                
                offset += field_size
            
            packets.append(parsed_packet)
        
        return packets
    
    def is_valid(self):
        """Check if buffer appears to be valid"""
        return (self.buf_ptr != 0 and 
                self.buf_size > 0 and 
                self.buf_size < 0x100000 and  # Reasonable size limit
                self.wr_off < self.buf_size and 
                self.rd_off < self.buf_size)
    
    def print_info(self):
        """Print buffer information"""
        self._refresh_fields()
        print(f"Base address: 0x{self.base_address:08X}")
        print(f"  Name: \"{self.name}\"")
        print(f"  Buffer Ptr:  0x{self.buf_ptr:08X}")
        print(f"  Size:        {self.buf_size}")
        print(f"  Write Off:   {self.wr_off}")
        print(f"  Read Off:    {self.rd_off}")
        print(f"  Flags:       0x{self.flags:08X}")
        print(f"  Name Ptr:    0x{self.name_ptr:08X}")
        if self.is_scope_buffer:
            print(f"  Scope Buffer: YES (packet size: {self.packet_size} bytes)")
            print(f"  Format: {[f'{fmt[0]}({fmt[1]}B)' for fmt in self.scope_format]}")
        if not self.is_valid():
            print(f"  WARNING: Buffer appears invalid")

class RTTControlBlock:
    """Class representing the RTT control block"""
    
    def __init__(self, target, base_address):
        self.target = target
        self.base_address = base_address
        self._parse_control_block()
    
    def _parse_control_block(self):
        """Parse the RTT control block structure"""
        self.id = read_c_string(self.target, self.base_address, 16)
        self.max_up = self.target.read32(self.base_address + 16)
        self.max_down = self.target.read32(self.base_address + 20)
        
        # Create buffer objects
        self.up_buffers = []
        self.down_buffers = []
        
        # Up buffers start at offset 24
        for i in range(self.max_up):
            buf_addr = self.base_address + 24 + i * 24
            self.up_buffers.append(RTTBuffer(self.target, buf_addr, i, 'up'))
        
        # Down buffers follow up buffers
        down_start = self.base_address + 24 + self.max_up * 24
        for i in range(self.max_down):
            buf_addr = down_start + i * 24
            self.down_buffers.append(RTTBuffer(self.target, buf_addr, i, 'down'))
    
    def is_valid(self):
        """Check if control block appears valid"""
        return (self.id.startswith("SEGGER RTT") and 
                self.max_up <= 16 and 
                self.max_down <= 16)
    
    def print_info(self, dump_memory=False, num_words=0x30):
        """Print control block information"""
        if dump_memory:
            print(f"Dumping RTT control block at 0x{self.base_address:08X}:")
            for i in range(num_words):
                addr = self.base_address + i * 4
                val = self.target.read32(addr)
                print(f"0x{addr:08X}: 0x{val:08X}")
        
        print(f"\nSEGGER RTT ID: {self.id!r}")
        print(f"MaxNumUpBuffers: {self.max_up}")
        print(f"MaxNumDownBuffers: {self.max_down}")
        
        if not self.is_valid():
            print("WARNING: Control block appears invalid")
            return
        
        print("\nUp Buffers:")
        for i, buffer in enumerate(self.up_buffers):
            print(f"\nUp buffer [{i}]")
            buffer.print_info()
        
        print("\nDown Buffers:")
        for i, buffer in enumerate(self.down_buffers):
            print(f"\nDown buffer [{i}]")
            buffer.print_info()
    
    def get_up_buffer(self, index):
        """Get up buffer by index"""
        if 0 <= index < len(self.up_buffers):
            return self.up_buffers[index]
        return None
    
    def get_down_buffer(self, index):
        """Get down buffer by index"""
        if 0 <= index < len(self.down_buffers):
            return self.down_buffers[index]
        return None

class RTTScopePlotter:
    """Real-time plotter for RTT scope buffer data"""
    
    def __init__(self, scope_buffer, max_points=1000):
        self.scope_buffer = scope_buffer
        self.max_points = max_points
        
        if not scope_buffer.is_scope_buffer:
            raise ValueError("Buffer is not a scope buffer")
        
        # Create data queues for each field
        self.data_queues = []
        self.field_names = []
        
        for i, (field_type, _, _) in enumerate(scope_buffer.scope_format):
            self.data_queues.append(deque(maxlen=max_points))
            self.field_names.append(f"Field{i}_{field_type}")
        
        # Setup matplotlib
        plt.ion()  # Interactive mode
        self.fig, self.axes = plt.subplots(len(self.data_queues), 1, figsize=(12, 2*len(self.data_queues)))
        if len(self.data_queues) == 1:
            self.axes = [self.axes]  # Make it a list for consistency
        
        self.lines = []
        for i, ax in enumerate(self.axes):
            line, = ax.plot([], [], 'b-')
            ax.set_title(f'{self.field_names[i]}')
            ax.grid(True)
            ax.set_xlim(0, max_points)
            self.lines.append(line)
        
        plt.tight_layout()
        plt.show()
    
    def update_plot(self):
        """Read new data and update plots"""
        try:
            packets = self.scope_buffer.read_scope_data()
            
            if packets:
                for packet in packets:
                    # Add each field value to its corresponding queue
                    for i, field_data in enumerate(packet):
                        if i < len(self.data_queues):
                            self.data_queues[i].append(field_data['value'])
                
                # Update plots
                for i, (line, ax, queue) in enumerate(zip(self.lines, self.axes, self.data_queues)):
                    if queue:
                        x_data = list(range(len(queue)))
                        y_data = list(queue)
                        
                        line.set_data(x_data, y_data)
                        
                        # Auto-scale y-axis
                        if y_data:
                            y_min, y_max = min(y_data), max(y_data)
                            margin = (y_max - y_min) * 0.1 if y_max != y_min else 1
                            ax.set_ylim(y_min - margin, y_max + margin)
                
                # Update x-axis for all plots
                for ax, queue in zip(self.axes, self.data_queues):
                    if queue:
                        ax.set_xlim(0, len(queue))
                
                plt.draw()
                plt.pause(0.001)  # Small pause to update display
                
        except Exception as e:
            print(f"Plot update error: {e}")

# Try to locate RTT CB within a memory window
def find_rtt_cb(target, search_start=0x20000000, search_size=0x1000):
    mem = target.read_memory_block8(search_start, search_size)
    for i in range(0, search_size - len(RTT_ID)):
        if bytes(mem[i:i+len(RTT_ID)]) == RTT_ID:
            return search_start + i
    return None

# Connect to target
session = ConnectHelper.session_with_chosen_probe(target_override="py32f003x8")
session.open()
target = session.target
target.resume()

try:
    rtt_cb_addr = find_rtt_cb(target)
    if not rtt_cb_addr:
        print("RTT control block not found.")
    else:
        print(f"RTT control block found at 0x{rtt_cb_addr:08X}")
        
        # Create RTT control block object
        rtt_cb = RTTControlBlock(target, rtt_cb_addr)
        
        # Print control block info
        rtt_cb.print_info(dump_memory=False)
        
        if not rtt_cb.is_valid():
            print("Invalid RTT control block, exiting...")
        else:
            # Get terminal buffer (buffer 0)
            terminal_buffer = rtt_cb.get_up_buffer(0)
            scope_buffer = rtt_cb.get_up_buffer(1)

            if terminal_buffer and terminal_buffer.is_valid():
                print(f"\nReading from terminal buffer: {terminal_buffer.name}")
                print("=" * 50)
                
                while True:
                    data = terminal_buffer.read_data()
                    if data:
                        print(data.decode(errors='replace'), end='', flush=True)

                    scope_data = scope_buffer.read_scope_data()
                    if scope_data:
                        print(scope_data)

                    time.sleep(0.05)
            else:
                print("No valid terminal buffer found")
finally:
    session.close()
