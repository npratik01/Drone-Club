import time
from pymavlink import mavutil

# Connect to the Pixhawk via USB
connection_string = '/dev/ttyUSB0'
baud_rate = 57600
print(f"Connecting to Pixhawk on {connection_string} at {baud_rate} baud...")
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

# Wait for the heartbeat from Pixhawk
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received from system %u component %u" % (master.target_system, master.target_component))

# Request data streams (e.g., GPS, attitude)
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # Rate in Hz
    1    # Start streaming
)

# Function to check GPS status
def check_gps_fix():
    print("Checking GPS status...")
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        fix_type = msg.fix_type  # 0-1: no fix, 2: 2D, 3: 3D, 4: DGPS, 5: RTK
        satellites_visible = msg.satellites_visible
        print(f"GPS Fix Type: {fix_type}, Satellites: {satellites_visible}")
        if fix_type >= 3 and satellites_visible >= 6:
            print("GPS 3D fix acquired!")
            return True
        time.sleep(1)

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
    print("Home position set.")

# Function to arm the drone
def arm_vehicle():
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # Arm (1 = arm, 0 = disarm)
        0, 0, 0, 0, 0, 0, 0  # Unused parameters
    )
    # Wait for acknowledgment
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg and msg.result == 0:
        print("Motors armed successfully!")
    else:
        print(f"Failed to arm: {msg.result}")
        exit(1)

# Function to disarm the drone
def disarm_vehicle():
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        0,  # Disarm
        0, 0, 0, 0, 0, 0, 0
    )
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg and msg.result == 0:
        print("Motors disarmed successfully!")
    else:
        print(f"Failed to disarm: {msg.result}")

# Function to test motors at low throttle
def test_motors(throttle=0.1, duration=5):
    print(f"Testing motors at {throttle*100}% throttle for {duration} seconds...")
    # Set to GUIDED mode for motor testing
    master.set_mode('GUIDED')
    time.sleep(1)  # Wait for mode change
    # Send manual control command for throttle
    master.mav.manual_control_send(
        master.target_system,
        0, 0, int(throttle * 1000), 0,  # x, y, z (throttle), yaw
        0  # Buttons
    )
    time.sleep(duration)
    # Stop motors
    master.mav.manual_control_send(
        master.target_system,
        0, 0, 0, 0,  # Zero throttle
        0
    )
    print("Motor test complete.")

# Main script
try:
    # Check for GPS fix or set home position
    if not check_gps_fix():
        print("No GPS fix. Setting manual home position for testing.")
        # Replace with your approximate coordinates (e.g., from a phone GPS)
        set_home_position(lat=37.7749, lon=-122.4194, alt=0)

    # Wait for PreArm checks to clear
    print("Checking PreArm status...")
    while True:
        msg = master.recv_match(type='STATUSTEXT', blocking=True)
        if "PreArm" not in msg.text.decode():
            print("PreArm checks passed!")
            break
        print(f"PreArm: {msg.text.decode()}")
        time.sleep(1)

    # Arm the drone
    arm_vehicle()

    # Test motors at 10% throttle for 5 seconds
    test_motors(throttle=0.1, duration=5)

    # Disarm the drone
    disarm_vehicle()

except KeyboardInterrupt:
    print("User interrupted. Disarming and exiting...")
    disarm_vehicle()
finally:
    print("Closing connection...")
    master.close()

print("Script completed.")
