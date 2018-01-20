from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import os

#Set up option parsing to get connection string
import argparse  

vehicle=None
PUSH_VALUE=20
PITCH_ROLL_QUADRANT = {
    1 : [-PUSH_VALUE, PUSH_VALUE],
    2 : [-PUSH_VALUE, -PUSH_VALUE],
    3 : [PUSH_VALUE, -PUSH_VALUE],
    4 : [PUSH_VALUE, PUSH_VALUE]
}
LOITER_DURATION=0
initial_yaw = 0.0


def connect_to_vehicle(connection_string, wait_ready=True):
    vehicle = connect(connection_string, wait_ready=True)

def avg(lst):
    return sum(l)/len(l)
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.8
    SMOOTH_TAKEOFF_THRUST = 0.56
    
    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.
#    while not vehicle.is_armable:
#        print " Waiting for vehicle to initialise..."
#        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    
    initial_yaw=vehicle.attitude.yaw
    start=time.time()
    print "Arming motors"
    # Copter should arm in GUIDED_NOGPS mode

    vehicle.mode = VehicleMode("GUIDED_NOGPS")
#    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)
    set_attitude(thrust=0.5,duration=5)
    print "Taking off!"


    # vehicle.simple_takeoff(aTargetAltitude)
    
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print " Altitude: ", current_altitude
        if current_altitude >= aTargetAltitude-0.3: # Trigger just below target alt.
            print "Reached target altitude"
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.1)

    # vehicle.mode = VehicleMode("GUIDED_NOGPS")
    print(" Calibrating Back to Initial Yaw")
    calibrate_yaw()
    brake()
    print("[Time Taken = %f s ]" %(time.time()-start))
    # os.system("espeak Calibrated")

def go_to_height(aTargetAltitude):
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print " Altitude: ", current_altitude
        if current_altitude >= aTargetAltitude-0.3: # Trigger just below target alt.
            print "Reached target altitude"
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.1)
    calibrate_yaw()
    brake()
def decendto(aTargetAltitude):
    thrust = 1.0 - DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print " Altitude: ", current_altitude
        if current_altitude <= aTargetAltitude*1.3:
            thrust = 1.0 - SMOOTH_TAKEOFF_THRUST
        if current_altitude <= aTargetAltitude+0.3: # Trigger just below target alt.
            print "Reached target altitude"
            break
        set_attitude(thrust = thrust)
        time.sleep(0.1)
    calibrate_yaw()
    brake()

def brake(duration=3):
    set_attitude()
    vehicle.mode=VehicleMode('GUIDED_NOGPS')

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """
    
    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """
    
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
                                                             0,
                                                             0,                                         #target system
                                                             0,                                         #target component
                                                             0b00000000,                                #type mask: bit 1 is LSB
                                                             to_quaternion(roll_angle, pitch_angle),    #q
                                                             0,                                         #body roll rate in radian
                                                             0,                                         #body pitch rate in radian
                                                             math.radians(yaw_rate),                    #body yaw rate in radian
                                                             thrust)                                    #thrust
    vehicle.send_mavlink(msg)
    # time.sleep(1)                                                     
    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)
        
        # Sleep for the fractional part
        time.sleep(modf[0])
        
        # Send command to vehicle on 1 Hz cycle
        for x in range(0,int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)
def calibrate_yaw():
    final_yaw = vehicle.attitude.yaw
    set_attitude(yaw_rate=(initial_yaw-final_yaw)/2.0,duration=1)
    time.sleep(1)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))
    
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    
    return [w, x, y, z]

def loiter(quadrant):
    pitch_angle, roll_angle = PITCH_ROLL_QUADRANT[quadrant]
    set_attitude(pitch_angle=pitch_angle,roll_angle=roll_angle,duration=LOITER_DURATION)
    time.sleep(2)
    pitch_angle,roll_angle = [-pitch_angle/PUSH_VALUE,-roll_angle/PUSH_VALUE]
    # set_attitude(pitch_angle=pitch_angle,roll_angle=roll_angle)
    brake()
    time.sleep(1)
    calibrate_yaw()
def turn(strng):
    calibrate_yaw()
    angle=0
    if(strng.lower[0]=='r'):
        angle=90.0
    if(strng.lower[0]=='l'):
        angle=-90.0
    set_attitude(yaw_rate=angle/3,duration=2)
    initial_yaw=(initial_yaw+math.radians(angle))%360
    brake()
def strafe(direction,duration=0):
    calibrate_yaw()
    roll_angle=pith_angle=0
    thrust=0.5
    if(strng.lower[0]=='f'):
        pitch_angle=-PUSH_VALUE
    if(strng.lower[0]=='b'):
        pitch_angle=PUSH_VALUE
    if(strng.lower[0]=='l'):
        roll_angle=-PUSH_VALUE
    if(strng.lower[0]=='r'):
        roll_angle=PUSH_VALUE
    if(strng.lower[0]=='u'):
        thrust=0.55
    if(strng.lower[0]=='d'):
        thrust=0.45
    set_attitude(pitch_angle=pitch_angle,roll_angle=roll_angle, thrust=0.5,duration=duration)
    time.sleep(1)
    brake()
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
    parser.add_argument('--connect', 
                       help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None

    connection_string = '127.0.0.1:14551'
#    connection_string= '/dev/ttyACM0'

    #Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)


    #Arm and take of to altitude of 5 meters
    arm_and_takeoff_nogps(5)

    # set_attitude(yaw_rate=-7.5,duration=1)
    # initial_yaw=0.0
    # set_attitude(pitch_angle=20, roll_angle=20)
    for i in [1,2,3,4]:
        loiter(i)
        # time.sleep(2)
    # p,r = 20,20 goto south-east (vehicle facing north)
    time.sleep(1)

    """
    The example is completing. LAND at current location.
    """ 
    time.sleep(1)
    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")

    #Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()

    print("Completed")
