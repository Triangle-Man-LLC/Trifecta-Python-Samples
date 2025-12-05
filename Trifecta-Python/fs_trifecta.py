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
        response = self.uart.read(FS_MAX_DATA_LENGTH, )
        if not response:
            return False
        if self.fs_device_parse_packet(response) != 0:
            return False
        return self.name != "Unknown"

    def fs_set_stream_state(self, do_stream):
        if do_stream:
            command = bytearray([FS_COMMAND_STREAM, ord('1'), FS_SERIAL_COMMAND_TERMINATOR])
        else:
            command = bytearray([FS_COMMAND_STREAM, ord('0'), FS_SERIAL_COMMAND_TERMINATOR])
        try:
          self.uart.write(command)
        except serial.serialutil.SerialTimeoutException:
          print(f'Timeout on {self.uart.port}, trying again later!')
        except Exception as e:
          print(f'Failed to read on {self.uart.port}: {e}')

    def fs_handle_receive_buffer(self):
        try:
          response = self.uart.read(FS_MAX_DATA_LENGTH)
          if response is not None:
              self.fs_device_parse_packet(response)
        except serial.serialutil.SerialTimeoutException:
          print(f'Timeout on {self.uart.port}, trying again later!')
        except Exception as e:
          print(f'Failed to read on {self.uart.port}: {e}')

    def fs_read_oneshot(self):
        """
        Sends the read command over UART and waits for a response.
        Return: True if data was processed.
        """
        command = bytearray([FS_COMMAND_STREAM, ord('2'), FS_SERIAL_COMMAND_TERMINATOR])
        try:
          self.uart.write(command)
          self.fs_handle_receive_buffer()
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
        # Combine with any leftover data from previous call
        data = self.last_data_string[:self.last_data_length] + rx_buf
        data_length = len(data)
        self.last_data_length = 0  # Reset leftover buffer
        
        index = 0
        while index < data_length:
            current_char = data[index]
            
            # Packet mode (between header and footer)
            if current_char == FS_SERIAL_PACKET_HEADER:
                start_index = index
                # Find matching footer
                end_index = data.find(FS_SERIAL_PACKET_FOOTER, index + 1)
                
                if end_index != -1:
                    # Process complete packet
                    segment = data[start_index+1:end_index]
                    output_buf = bytearray(len(segment))
                    if self.fs_base64_decode(segment, output_buf) > 0:
                        self.fs_update_most_recent_packet(output_buf)
                    index = end_index + 1  # Skip past footer
                    continue
                else:
                    # Incomplete packet, save for next call
                    self.last_data_string[:data_length - start_index] = data[start_index:]
                    self.last_data_length = data_length - start_index
                    break
                    
            # Command mode (until terminator)
            elif current_char != FS_SERIAL_COMMAND_TERMINATOR:
                start_index = index
                # Find command terminator
                end_index = data.find(FS_SERIAL_COMMAND_TERMINATOR, index + 1)
                
                if end_index != -1:
                    # Process complete command
                    segment = data[start_index:end_index+1]
                    self.fs_handle_received_commands(segment)
                    index = end_index + 1  # Skip past terminator
                    continue
                else:
                    # Incomplete command, save for next call
                    self.last_data_string[:data_length - start_index] = data[start_index:]
                    self.last_data_length = data_length - start_index
                    break
                    
            index += 1
            
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

        if self.type in (0, 1, 2, 3):  # composite, 145 bytes
            fmt = '<' + 'f'*18 + 'f'*4 + 'f'*3 + 'f'*3 + 'f'*3 + 'h'*3 + 'b' + 'b' + 'b'*3 + 'b' + 'i'
            fields = struct.unpack(fmt, data[5:145])

            (
                self.ax0, self.ay0, self.az0, self.gx0, self.gy0, self.gz0,
                self.ax1, self.ay1, self.az1, self.gx1, self.gy1, self.gz1,
                self.ax2, self.ay2, self.az2, self.gx2, self.gy2, self.gz2,
                self.q0, self.q1, self.q2, self.q3,
                self.ax, self.ay, self.az,
                self.vx, self.vy, self.vz,
                self.rx, self.ry, self.rz,
                reserved0, reserved1, reserved2,
                self.device_in_motion,
                self.label_2,
                temp0, temp1, temp2,
                self.c,
                self.d
            ) = fields

        elif self.type in (4, 5, 6, 7):  # regular, 85 bytes
            fmt = '<' + 'f'*3 + 'f'*4 + 'f'*3 + 'f'*3 + 'f'*3 + 'h'*3 + 'b' + 'b' + 'b'*3 + 'b' + 'i'
            fields = struct.unpack(fmt, data[5:85])
            (
                self.omega_x0, self.omega_y0, self.omega_z0,
                self.q0, self.q1, self.q2, self.q3,
                self.ax, self.ay, self.az,
                self.vx, self.vy, self.vz,
                self.rx, self.ry, self.rz,
                reserved0, reserved1, reserved2,
                self.device_in_motion,
                self.label_2,
                temp0, temp1, temp2,
                self.c,
                self.d
            ) = fields

        elif self.type in (8, 9, 10, 11):  # composite2, 157 bytes
            fmt = '<' + 'f'*18 + 'f'*4 + 'f'*3 + 'f'*3 + 'f'*3 + 'h'*3 + 'b' + 'b' + 'b'*3 + 'b' + 'i'
            fields = struct.unpack(fmt, data[5:157])
            (
                self.ax0, self.ay0, self.az0, self.gx0, self.gy0, self.gz0,
                self.ax1, self.ay1, self.az1, self.gx1, self.gy1, self.gz1,
                self.ax2, self.ay2, self.az2, self.gx2, self.gy2, self.gz2,
                self.q0, self.q1, self.q2, self.q3,
                self.wx, self.wy, self.wz,
                self.ax, self.ay, self.az,
                self.vx, self.vy, self.vz,
                self.rx, self.ry, self.rz,
                reserved0, reserved1, reserved2,
                self.device_in_motion,
                self.label_2,
                temp0, temp1, temp2,
                self.c,
                self.d
            ) = fields

        self.roll, self.pitch, self.yaw = self.fs_q_to_euler_angles(
            self.q0, self.q1, self.q2, self.q3, degrees=True
        )

    def fs_q_to_euler_angles(self, q0, q1, q2, q3, degrees=False):
        estRoll = math.atan2(q0 * q1 + q2 * q3, 0.5 - q1**2 - q2**2)
        estPitch = math.asin(-2.0 * (q1 * q3 - q0 * q2))
        estYaw = math.atan2(q1 * q2 + q0 * q3, 0.5 - q2**2 - q3**2)

        if degrees:
            estRoll *= 180.0 / math.pi
            estPitch *= 180.0 / math.pi
            estYaw *= 180.0 / math.pi

        return estRoll, estPitch, estYaw