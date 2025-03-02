"""
    In this example, data from the Trifecta device is sent over serial.
    The code in this file is released into the public domain.
    
    /// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    /// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
    /// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    /// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    /// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
import serial  # Import the pyserial module for UART
import serial.tools.list_ports
import time

from fs_trifecta import FsDevice, FsImuCompositePacket

# List all available ports
ports = serial.tools.list_ports.comports()
available_ports = [port.device for port in ports]

# Try initializing each port until successful
for port in available_ports:
    try:
        imu = FsDevice(port=port)
        for i in range(0,5):
            print(f'Trying connection on {port}...')
            initialized = (imu.fs_identify_device() != 0)
            time.sleep(0.05)
        
        if(initialized):
          print(f'Successfully connected on {port}!')
          break  # Exit the loop once successful
    except Exception as e:
        print(f'Failed to initialize on {port}: {e}')
else:
    print('All attempts to initialize the device have failed.')

# Then, enter a main loop which reads IMU data.
while True:
    imu.fs_read_oneshot()
    print(f'IMU Data - Time: {imu.packet.time}, Quaternion: ({imu.packet.q0},{imu.packet.q1},{imu.packet.q2},{imu.packet.q3}), Euler: ({imu.packet.roll}, {imu.packet.pitch}, {imu.packet.yaw})\r')
    
    # Handle your other tasks here e.g. read from other sensors, GPIO, etc.

    time.sleep(0.001)  # Sleep for 1 ms to prevent CPU overloading