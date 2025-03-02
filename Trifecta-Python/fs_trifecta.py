"""
    Driver for the Trifecta series of IMU/AHRS/INS devices
    Copyright 2024 4rge.ai and/or Triangle Man LLC
    Usage and redistribution of this code is permitted
    but this notice must be retained in all copies of the code.

    /// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    /// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
    /// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    /// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    /// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
import base64
import serial
import math
import time
import struct

# Constants
FS_MAX_COMMAND_LENGTH = 64  # Maximum command length in bytes
FS_MAX_DATA_LENGTH = 512  # Maximum data length in bytes
FS_MAX_PACKET_QUEUE_LENGTH = 10  # Maximum number of packets in the queue
FS_MAX_PACKET_LENGTH = 512  # Maximum packet length in bytes

FS_REGULAR_PACKET_LENGTH = struct.calcsize('BI18i12f3h2b2i')

# Packet delimiters
FS_SERIAL_PACKET_HEADER = 0x3A  # ':'
FS_SERIAL_PACKET_FOOTER = 0x21  # '!'

# Command terminator
FS_SERIAL_COMMAND_TERMINATOR = 0x3B  # ';'

# Command codes
FS_COMMAND_IDENTIFY = 0x49  # 'I'
FS_COMMAND_STREAM = 0x41  # 'A'
FS_COMMAND_RESTART = 0x52  # 'R'

# UART Configuration Constants
FS_TRIFECTA_SERIAL_BAUDRATE = 2000000
UART_DATA_BITS = 8
UART_PARITY = 'N'  # No parity
UART_STOP_BITS = 1
UART_FLOW_CONTROL = 0  # No hardware flow control

class FsDevice:
    def __init__(self, port):
        """
        Starts device communication over UART at 2 Mbaud.
        :param port: Serial port name (e.g., '/dev/ttyUSB0' or 'COM3')
        """
        self.uart = serial.Serial(
            port=port,
            baudrate=FS_TRIFECTA_SERIAL_BAUDRATE,
            bytesize=UART_DATA_BITS,
            parity=UART_PARITY,
            stopbits=UART_STOP_BITS,
            timeout=0.004,  # 4 ms timeout
            write_timeout=0.004
        )
        print(f"UART initialized on port: {port} Baudrate: {FS_TRIFECTA_SERIAL_BAUDRATE}")
        self.name = "Unknown"
        self.command_queue = []
        self.last_data_string = bytearray(FS_MAX_PACKET_LENGTH)
        self.last_data_length = 0
        self.command_queue_size = 0
        self.packet = FsImuCompositePacket(type=0, time=0)  # Last received packet

    def fs_identify_device(self):
        """
        Sends the identify command over UART and waits for a response.
        Return: True if device detected.
        """
        command = bytearray([FS_COMMAND_IDENTIFY, ord('0'), FS_SERIAL_COMMAND_TERMINATOR])
        self.uart.write(command)
        response = self.uart.read(FS_MAX_DATA_LENGTH)
        if not response:
            return False
        if self.fs_device_parse_packet(response) != 0:
            return False
        return self.name != "Unknown"

    def fs_read_oneshot(self):
        """
        Sends the read command over UART and waits for a response.
        Return: True if data was processed.
        """
        command = bytearray([FS_COMMAND_STREAM, ord('2'), FS_SERIAL_COMMAND_TERMINATOR])
        try:
          self.uart.write(command)
          response = self.uart.read(FS_MAX_DATA_LENGTH)
          if response is not None:
              self.fs_device_parse_packet(response)
        except serial.serialutil.SerialTimeoutException:
          print(f'Timeout on {self.uart.port}, trying again later!')
        except Exception as e:
          print(f'Failed to read on {self.uart.port}: {e}')
          

    def fs_log_output(self, message, *args):
        """Alias for print()"""
        print(message % args)

    def fs_enqueue_into_command_queue(self, cmd_str):
        if len(self.command_queue) >= FS_MAX_COMMAND_LENGTH:
            self.fs_log_output("[Trifecta] Device command queue was full!")
            return -1
        self.command_queue.append(cmd_str)
        return 0

    def fs_segment_commands(self, cmd_buf):
        input_line = bytearray()
        state = 'WW'
        for in_byte in cmd_buf:
            if in_byte == ord(';'):
                cmd = input_line.decode('utf-8').strip('\x00')
                if self.fs_enqueue_into_command_queue(cmd) != 0:
                    return -1
                input_line = bytearray()
                state = 'WW'
            elif in_byte in (ord('\n'), ord('\r'), ord('\0')):
                if state == 'WW':
                    input_line = bytearray()
                else:
                    input_line = bytearray()
                    return 3
            else:
                state = 'RR'
                if len(input_line) < FS_MAX_DATA_LENGTH - 1:
                    input_line.append(in_byte)
        return 0

    def fs_handle_received_commands(self, cmd_buf):
        if self.fs_segment_commands(cmd_buf) != 0:
            self.fs_log_output("[Trifecta] Command parsing failed!")
            return -1
        for cmd in self.command_queue:
            cmd = cmd.strip('\x00')
            cmd_length = len(cmd)
            if cmd_length > 0 and cmd[0] == chr(FS_COMMAND_IDENTIFY):
                self.name = cmd[1:cmd_length]
        self.command_queue.clear()
        return 0

    def fs_base64_decode(self, input_bytes, output_buf):
        try:
            decoded = base64.b64decode(input_bytes)
        except:
            return -1
        if len(decoded) > len(output_buf):
            return -1
        output_buf[:len(decoded)] = decoded
        return len(decoded)

    def obtain_packet_length(self):
        return FS_REGULAR_PACKET_LENGTH

    def fs_update_most_recent_packet(self, packet):
        self.packet.update_from_data(packet)
        return 0

    def segment_packets(self, rx_buf):
        pos = 0
        packets = []
        while pos < len(rx_buf):
            packet_length = self.obtain_packet_length()
            if packet_length == 0 or pos + packet_length > len(rx_buf):
                return -1
            packets.append(rx_buf[pos:pos+packet_length])
            pos += packet_length
        return len(packets)

    def fs_device_process_packets_serial(self, rx_buf):
        scanner_state = 'S'
        start_index = -1
        data = self.last_data_string[:self.last_data_length] + rx_buf
        data_length = len(data)
        self.last_data_length = 0  # Reset after processing

        for index in range(data_length):
            current_char = data[index]
            if scanner_state == 'S':
                if current_char == FS_SERIAL_PACKET_HEADER:
                    start_index = index
                    scanner_state = 'P'
                else:
                    start_index = index
                    scanner_state = 'C'

            elif scanner_state == 'P':
                if current_char == FS_SERIAL_PACKET_FOOTER:
                    segment = data[start_index+1:index]
                    output_buf = bytearray(len(segment))
                    decoded_len = self.fs_base64_decode(segment, output_buf)
                    if decoded_len > 0:
                        self.fs_update_most_recent_packet(output_buf) 
                    scanner_state = 'S'
                elif index == data_length - 1:
                    self.last_data_string[:data_length - start_index] = data[start_index:]
                    self.last_data_length = data_length - start_index

            elif scanner_state == 'C':
                if current_char == FS_SERIAL_COMMAND_TERMINATOR:
                    segment = data[start_index:index+1]
                    self.fs_handle_received_commands(segment)
                    scanner_state = 'S'
                elif current_char == FS_SERIAL_PACKET_HEADER:
                    start_index = index
                    scanner_state = 'P'
                elif index == data_length - 1:
                    self.last_data_string[:data_length - start_index] = data[start_index:]
                    self.last_data_length = data_length - start_index
        return 0

    def fs_device_parse_packet(self, rx_buf):
        packets = self.segment_packets(rx_buf)
        if packets < 0:
            if self.fs_device_process_packets_serial(rx_buf) != 0:
                return -1
        return 0

class FsImuCompositePacket:
    def __init__(self, type=0, time=0):
        self.type = type
        self.time = time

        self.ax0 = self.ay0 = self.az0 = 0
        self.gx0 = self.gy0 = self.gz0 = 0

        self.ax1 = self.ay1 = self.az1 = 0
        self.gx1 = self.gy1 = self.gz1 = 0

        self.ax2 = self.ay2 = self.az2 = 0
        self.gx2 = self.gy2 = self.gz2 = 0

        self.q0 = self.q1 = self.q2 = self.q3 = 0.0
        self.roll = self.pitch = self.yaw = 0

        self.ax = self.ay = self.az = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.rx = self.ry = self.rz = 0.0

        self.grav_x = self.grav_y = self.grav_z = 0
        self.label_1 = self.label_2 = 0
        self.c = self.d = 0

    def update_from_data(self, data):
        self.type = data[0]
        self.time = int.from_bytes(data[1:5], 'little')

        indices = [
            (5, 9), (9, 13), (13, 17), (17, 21), (21, 25), (25, 29),
            (29, 33), (33, 37), (37, 41), (41, 45), (45, 49), (49, 53),
            (53, 57), (57, 61), (61, 65), (65, 69), (69, 73), (73, 77),
            (129, 131), (131, 133), (133, 135), (137, 141), (141, 145)
        ]

        values = [
            'ax0', 'ay0', 'az0', 'gx0', 'gy0', 'gz0', 'ax1', 'ay1', 'az1',
            'gx1', 'gy1', 'gz1', 'ax2', 'ay2', 'az2', 'gx2', 'gy2', 'gz2',
            'grav_x', 'grav_y', 'grav_z', 'c', 'd'
        ]

        for (start, end), attr in zip(indices, values):
            byte_count = end - start
            val = int.from_bytes(data[start:end], byteorder='little', signed=True)
            setattr(self, attr, val)

        self.q0, self.q1, self.q2, self.q3 = struct.unpack('<ffff', data[77:93])
        self.roll, self.pitch, self.yaw = self.fs_q_to_euler_angles(self.q0, self.q1, self.q2, self.q3, degrees=True)
        
        self.ax, self.ay, self.az = struct.unpack('<fff', data[93:105])
        self.vx, self.vy, self.vz = struct.unpack('<fff', data[105:117])
        self.rx, self.ry, self.rz = struct.unpack('<fff', data[117:129])

        self.label_1, self.label_2 = data[135], data[136]

    def fs_q_to_euler_angles(self, q0, q1, q2, q3, degrees=False):
        estRoll = math.atan2(q0 * q1 + q2 * q3, 0.5 - q1**2 - q2**2)
        estPitch = math.asin(-2.0 * (q1 * q3 - q0 * q2))
        estYaw = math.atan2(q1 * q2 + q0 * q3, 0.5 - q2**2 - q3**2)

        if degrees:
            estRoll *= 180.0 / math.pi
            estPitch *= 180.0 / math.pi
            estYaw *= 180.0 / math.pi

        return estRoll, estPitch, estYaw