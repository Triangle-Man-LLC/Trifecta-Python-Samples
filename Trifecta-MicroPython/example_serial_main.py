"""
    In this example, data from the Trifecta IMU device is received over serial.
    The code in this file is released into the public domain.
    
    /// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    /// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
    /// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    /// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    /// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import time
from fs_trifecta_mp import FsDevice, FsImuCompositePacket

# For micropython, it is good to wait for a bit before starting to avoid bricking your device by accident
# That way you can CTRL + C out of it
time.sleep(2) 

# Create a new device handle(self, uart_id, tx_pin, rx_pin)
imu = FsDevice(uart_id = 1, tx_pin = 18, rx_pin = 26)

# While FsDevice.fs_identify_device() is false, wait 100 ms and call the function again
initialized = False
while(not initialized):
    print(f'Waiting for IMU connection!')
    initialized = (imu.fs_identify_device() != 0)
    time.sleep(0.05)

# Then, enter a main loop which calls FsDevice.
while(True):
    imu.fs_read_oneshot()
    print(f'IMU Data - Time: {imu.packet.time}, Quaternion: ({imu.packet.q0},{imu.packet.q1},{imu.packet.q2},{imu.packet.q3}), Euler: ({imu.packet.roll}, {imu.packet.pitch}, {imu.packet.yaw})\r')
        
    # Handle your other tasks here e.g. read from other sensors, GPIO, etc.

    time.sleep(0.001) # Sleep for 1 ms to prevent CPU overloading

