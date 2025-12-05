"""
    In this example, data from the Trifecta device is sent over serial.
    The code in this file is released into the public domain.
    
    /// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    /// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
    /// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    /// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    /// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
import serial  # Import the pyserial module for UART.
import serial.tools.list_ports
import time

from fs_trifecta import FsDevice

# List all available ports.
ports = serial.tools.list_ports.comports()
available_ports = [port.device for port in ports]

# Try initializing each port until successful.
for port in available_ports:
    try:
        imu = FsDevice(port=port)
        for i in range(0,5):
            print(f'Trying connection on {port}...')
            initialized = (imu.fs_identify_device() == True)
            time.sleep(0.1)
        
            if(initialized):
                print(f'Successfully connected on {port}!')
                imu.fs_set_stream_state(True) # Start streaming.
                break  # Exit the loop once successful.
        if(initialized):
            break # Exit the second loop as well, we only are reading from one IMU only
    except Exception as e:
        print(f'Failed to initialize on {port}: {e}')
else:
    print('All attempts to initialize the device have failed.')
    exit

if (initialized):
    # Then, enter a main loop which reads IMU data.
    counter = 0
    try:
        while True:
            imu.fs_handle_receive_buffer()
            print(f'IMU Data - Time: {imu.packet.time}, Quaternion: ({imu.packet.q0},{imu.packet.q1},{imu.packet.q2},{imu.packet.q3}), Euler: ({imu.packet.roll}, {imu.packet.pitch}, {imu.packet.yaw})\r')
            if (counter % 1000 == 0):
                imu.fs_set_stream_state(True) # [Optional] Send a keepalive signal in case the IMU cable was unplugged etc.
            counter+=1
            # Handle your other tasks here e.g. read from other sensors, GPIO, etc.

            # [Optional] Small delay to prevent CPU overload (adjust as needed).
            time.sleep(0.001)   
    except:
        # Cleanup on shutdown received.
        imu.fs_set_stream_state(False)        
        time.sleep(0.1)
        print("\nIMU stream stopped. Exiting!")