import time
import sys
from pymavlink import mavutil
from serial.serialutil import SerialException

# Connect to the Pixhawk via USB
connection_string = '/dev/ttyUSB0'
baud_rate = 57600
print(f"Connecting to Pixhawk on {connection_string} at {baud_rate} baud...")
try:
    master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
except SerialException as e:
    print(f"Failed to connect: {e}")
    sys.exit(1)

# Wait for the heartbeat from Pixhawk
print("Waiting for heartbeat...")
try:
    master.wait_heartbeat(timeout=10)
    print("Heartbeat received from system %u component %u" % (master.target_system, master.target_component))
except Exception as e:
    print(f"Heartbeat timeout: {e}")
    master.close()
    sys.exit(1)

# Request data streams (e.g., status messages)
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # Rate in Hz
    1    # Start streaming
)

# Function to set home position manually (for indoor testing)
def set_home_position(lat, lon, alt):
    print(f"Setting home position to lat={lat}, lon={lon}, alt={alt}...")
    master.mav.set_home_position_send(
        master.target_system,
        lat * 1e7,  # Latitude in degrees * 1e7
        lon * 1e7,  # Longitude in degrees * 1e7
        alt * 1000, # Altitude in mm
        0, 0, 0,    # Position (x, y, z)
        [0, 0, 0, 0],  # Quaternion
        0, 0, 0     # Velocity
    )
    time.sleep(1)
    print("Home position set.")

# Function to arm the drone
def arm_vehicle():
    print("Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg and msg.result == 0:
        print("Vehicle armed successfully!")
        return True
    else:
        print(f"Failed to arm: {msg.result if msg else 'No response'}")
        return False

# Function to disarm the drone
def disarm_vehicle():
    print("Disarming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg and msg.result == 0:
        print("Vehicle disarmed successfully!")
    else:
        print(f"Failed to disarm: {msg.result if msg else 'No response'}")

# Function to control servo on channel 5
def set_servo_pwm(channel, pwm):
    print(f"Setting servo on channel {channel} to PWM {pwm}...")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, pwm, 0, 0, 0  # Channel 5 (index 4)
    )
    time.sleep(0.5)  # Allow servo to move

# Main script
try:
    # Set manual home position (update with your coordinates, e.g., Bengaluru, India)
    set_home_position(lat=12.9716, lon=77.5946, alt=0)

    # Wait for PreArm checks to clear
    print("Checking PreArm status...")
    start_time = time.time()
    while time.time() - start_time < 30:
        try:
            msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
            if not msg or "PreArm" not in msg.text.decode():
                print("PreArm checks passed!")
                break
            print(f"PreArm: {msg.text.decode()}")
        except SerialException as e:
            print(f"Serial error during PreArm check: {e}")
            break
        time.sleep(1)
    else:
        print("PreArm checks not cleared. Check Mission Planner for details.")
        master.close()
        sys.exit(1)

    # Arm the vehicle
    if not arm_vehicle():
        master.close()
        sys.exit(1)

    # Test servo: move to open (1900 µs) and closed (1100 µs) positions
    print("Starting servo test...")
    set_servo_pwm(channel=5, pwm=1900)  # Open position (adjust PWM as needed)
    time.sleep(2)  # Hold for 2 seconds
    set_servo_pwm(channel=5, pwm=1100)  # Closed position (adjust PWM as needed)
    time.sleep(2)
    set_servo_pwm(channel=5, pwm=1500)  # Neutral position
    print("Servo test complete.")

    # Disarm the vehicle
    disarm_vehicle()

except KeyboardInterrupt:
    print("User interrupted. Disarming and exiting...")
    disarm_vehicle()
except SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    print("Closing connection...")
    master.close()

print("Script completed.")
