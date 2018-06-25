from dronekit import *
from pymavlink import mavutil

import time
import sys
import argparse
import math
import signal
import _thread
import socket


######################################################################
##                                                                  ##
##                         GLOBAL VARIABLES                         ##
##                                                                  ##
######################################################################
global vehicle
vehicle = None
global moveOnFlag
moveOnFlag = False
global errorCount
errorCount = 0
global MAX_ERROR_COUNT
MAX_ERROR_COUNT = 20
abortLevel = 0.8 # if the batteries fall below this level, the generator has likely stopped
panicLevel = 0.2 # if the batteries fall below this level, batteries are ~1 minute from dying
podLocs = [LocationGlobal(0, 0, 0)] #List of locations that the drone will drop pods off at
podsDropped= 0 # Determines which location the drone will drop the next pod off at
totalPods = 1 # how many pods are there?
podsRecovered = 1-totalPods # Determines which location the drone will return to before hunting for the nearest pod, a negative number signifies a return to home
funnelSize = 0.25 #Radius of the funnel in meters
grabDist = 0.25 #Distance from the pod from where we can grab, in meters
recvData = [] #the recieving thread adds to this in the format ((x, y), z, timestamp), where any value is none if not known

vision = socket.socket()
vision.bind(("127.0.0.1", 6565))


######################################################################
##                                                                  ##
##                        METHOD DEFINITIONS                        ##
##                                                                  ##
######################################################################

# Arm the drone in guided mode
def drone_arm():
    global moveOnFlag
    global errorCount
    
    # Don't arm until autopilot is ready
    print "** Basic pre-arm checks **"
    timer = 20
    while not vehicle.is_armable and timer > 0:
        print "Waiting for drone to initialize..."
        timer -= 2
        time.sleep(2)
    if timer <= 0:
        print "=== Timeout on drone initialization ==="
        end_program()

    # Set the drone to guided mode and confirm
    print "** Setting mode to GUIDED **"
    while not moveOnFlag:
        if errorCount > MAX_ERROR_COUNT:
            print "=== Errors on setting guided mode ==="
            end_program()
        vehicle.mode = VehicleMode("GUIDED")
        print "Wating for GUIDED mode..."
        time.sleep(2)
    moveOnFlag = False

    # Arm the drone and confirm
    print "** Arming the drone **"
    while not moveOnFlag:
        if errorCount > MAX_ERROR_COUNT:
            print "=== Errors on arming the drone ==="
            end_program()
        vehicle.armed = True
        print "Waiting for arming..."
        time.sleep(2)
    moveOnFlag = False

    # Drone is armed and ready to fly
    print "** Ready to takeoff (if safety switch is off...) **"


# Takeoff to specified height in meters
def drone_takeoff(aTargetAltitude):
    global moveOnFlag
    global errorCount

    # Takeoff to target altitude
    print "** Taking off to an altitude of %s meters **" % aTargetAltitude

    while not moveOnFlag:
        if errorCount > MAX_ERROR_COUNT:
            print "=== Errors on drone takeoff ==="
            end_program()
        vehicle.simple_takeoff(aTargetAltitude)
        print "Trying to takeoff..."
        time.sleep(2)
    moveOnFlag = False

    # Wait until the drone reaches at least 95% of the target altitude before
    # continuing with other commands or timeout if it doesn't reach it within
    # a time limit proportional to the altitude
    timer = 3 * aTargetAltitude
    while timer > 0:
        relative_alt = vehicle.location.global_relative_frame.alt
        absolute_alt = vehicle.location.global_frame.alt
        print "Relative Altitude: %s" % relative_alt
        # print "Absolute Altitude: %s" % absolute_alt

        if relative_alt >= aTargetAltitude*0.95:
            print "** Reached targed altitude **"
            break

        timer -= 2
        time.sleep(2)
    if timer <= 0:
        print "=== Timeout on drone altitude check ==="
        end_program()



def get_loc(currentLoc, dNorth, dEast, alt):
    """Returns a location offset by a certain amount from the current position. Inaccurate over very long distances or near the poles."""
    rad = 6378137.0
    dLat = dNorth/rad
    dLon = dEast/(rad*math.cos(math.pi*currentLoc.lat/180))
    return LocationGlobal(currentLoc.lat+(dLat*180/math.pi), currentLoc.lon+(dLon*180/math.pi), currentLoc.alt+alt)


def get_distance_meters(locA, locB):
    """Gets the distance between two location objects, in meters"""
    dlat = locB.lat - locA.lat
    dlon = locB.lon - locA.lon
    dalt = locB.alt - locA.alt
    return math.sqrt((dlat**2) + (dlon**2) + (dalt**2)) * 1.1131195e5


def getNextPodLocation():
    if podsDropped < len(podLocs):
        loc = podLocs[podsDropped]
        podsDropped += 1
        return loc
    else:
        podsDropped += 1
        return home_loc


def getLastPodLocation():
    if podsRecovered < len(podLocs) and podsRecovered >= 0:
        loc = podLocs[podsRecovered]
        podsRecovered += 1
        return loc
    else:
        podsRecovered += 1
        return home_loc


def recvDat(conn):
    dat = conn.recv(1024).decode("ASCII").split("/")
    if 'None' not in dat[0]:
        x = int(dat[0])
    else:
        x = "None"
    if "None" not in dat[1]:
        y = int(dat[1])
    else:
        y = "None"
    if "None" not in dat[2]:
        z = int(dat[2])
    else:
        z = None
    xy = (x,y)
    if "None" in xy:
        xy = None
    recvData = (xy, z, time.time())


def connectVision():
    vision.listen(2) #this makes the server listen for two connections (for some reason, it doesn't work with just one)
    clientSocket, addr = vision.accept()
    _thread.start_new_thread(recvDat, (clientSocket,))


def podDist():
    if time.time() - recvData[2] > 1.5:
        print "Data stale! Is the vision program running?"
        return None
    return recvData[1]


def podRelease():
    pass # blocking until pod is confirmed released, does not return anything


def podGrab():
    pass #same as above, but in reverse


def dropPod():
    loc = getNextPodLocation()
    vehicle.simple_goto(loc)
    wait_for_arrival(loc.lat, loc.lon)
    podRelease()


def getPodLoc():
    return (0, 0) # Returns the (lat, lon) of the nearest broadcasting pod. Returns None if not found


def getPodPos():
    if time.time() - recvData[2] > 1.5:
        print "Data stale! Is the vision program running?"
        return None
    return recvData[0]


def getPodDis():
    pos = getPodPos()
    if pos == None:
        return None
    return ((pos[0]**2)+(pos[1]**2))**0.5


def locatePod():
    pod_acquiesce_attempts = 1
    condition_yaw(0) #We need to be facing north for vision to work, so we should be facing north when trying to find it
    while (not getPodLoc()): #Attempt to find the pod with our eyes
        coords = getPodLoc()
        loc = LocationGlobal(coors[0], coords[1], 5)
        vehicle.simple_goto(loc)
        condition_yaw(0)
        wait_for_arrival(loc.lat, loc.lon) #Go to where the pod says it is
        if getPodLoc():
            break
        print "Pod not found at destination - circling"
        vehicle.parameters['CIRCLE_RADIUS'] = 1000
        vehicle.parameters['CIRCLE_RATE'] = 45
        timer = 0
        while not vehicle.mode == 'CIRCLE': #If we still don't see it, circle the area a few times until we see it
            vehicle.mode = VehicleMode("CIRCLE")
            time.sleep(1)
        condition_yaw(0)
        while timer <= 30:
            if getPodLoc():
                while not vehicle.mode == 'GUIDED':
                    vehicle.mode = VehicleMode("GUIDED")
                    time.sleep(1)
                break
            time.sleep(1)
            timer += 1
        while not vehicle.mode == 'GUIDED':
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
        condition_yaw(0)
        if getPodLoc():
            break
        else:
            print "Pod not sighted - circling mk. II"
            send_ned_velocity(0, -3, 0, 3)
            condition_yaw(0)
            vehicle.parameters['CIRCLE_RADIUS'] = 10000 #If we *still* don't see it, do a wider circle
            vehicle.parameters['CIRCLE_RATE'] = 15
            timer = 0
            while not vehicle.mode == 'CIRCLE':
                vehicle.mode = VehicleMode("CIRCLE")
                time.sleep(1)
            condition_yaw(0)
            while timer <= 90:
                if getPodLoc():
                    while not vehicle.mode == 'GUIDED':
                        vehicle.mode = VehicleMode("GUIDED")
                        time.sleep(1)
                    break
                time.sleep(1)
                timer += 1
            while not vehicle.mode == 'GUIDED':
                vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1)
                condition_yaw(0)
            if getPodLoc():
                break
            #If at first you don't succeed...
            elif pod_acquiesce_attempts < 3: 
                print "Pod not sighted - retrying" #...try, try again
                print "Total attempts: %s" %pod_acquiesce_attempts
                pod_acquiesce_attempts += 1
            else:
                print "Pod not sighted after three attempts - abandoning pod" #...give up
                print "Pod's last known position: %s, %s" %(loc.lat, loc.lon)
                break


def retrievePod(layers=0):
    if layers == 0:
        loc = getLastPodLocation() #Go to last known location of the pod
        vehicle.simple_goto(loc)
        wait_for_arrival(loc.lat, loc.lon)
        locatePod() #Either we find the pod or we've put in our best efforts
    if layers < 3:
        if getPodLoc():
            print "Pod sighted!"
            print "Engaging pod..."
            condition_yaw(0) #make the drone face north, so that the vision system's (x,y) coordinates line up with the NED coordinate system
            kP = -0.5
            dist = 999
            ticker = 60
            if getPodDis() != None:
                dist = getPodDis()
            while dist > funnelSize:
                #The documentation says that facing direction of travel is the default behavior, it will return to default behavior on "changing the command used for movement",
                #but there is no "safe way" to return to default.
                condition_yaw(0) #I have no idea how much the yaw needs to be set, but I'm confident that I cannot change yaw too much
                if getPodLoc():
                    coords = getPodLoc()
                    x_speed = kP*coords[0]
                    y_speed = kP*coords[1]
                    send_ned_velocity_inst(x_speed, y_speed, 0)
                    condition_yaw(0)
                    ticker = 60
                else:
                    send_ned_velocity_inst(0, 0, 0)
                    condition_yaw(0)
                    ticker -= 1
                    time.sleep(1)
                if getPodDis() != None:
                    dist = getPodDis()
                if ticker == 0:
                    break
            if ticker == 0:
                print "Lost pod while engaging"
                locatePod()
            else:
                print "Over pod, moving to grab"
                if podDist() != None:
                    dist1 = podDist()
                while dist1 > grabDist:
                    if getPodLoc():
                        coords = getPodLoc()
                        x_speed = kP*coords[0]
                        y_speed = kP*coords[1]
                        send_ned_velocity_inst(x_speed, y_speed, 0.5)
                        ticker = 60
                    else:
                        send_ned_velocity_inst(0, 0, 0)
                        ticker -= 1
                        time.sleep(1)
                    if podDist() != None:
                        dist1 = podDist()
                    if ticker == 0:
                        break
                if ticker == 0:
                    print "Lost pod while grabbing"
                    locatePod()
                else:
                    send_ned_velocity_inst(0, 0, 0)
                    print "Grabbing pod!"
                    podGrab()
        if ticker == 0: #We've lost the pod for over a minute while trying either to line up or to grab it
            print "Retrying pod grab manuever"
            print "Attempts: %s" %layers
            retrievePod(layers+1)
    else:
        print "Could not retrieve pod - is it discolored or obscured?"
        (lat, lon) = getPodLoc()
        print "Last known location: %s, %s" %(lat, lon)


def checkAbort():
    if batt.level < abortLevel:
        print "Aborting mission due to low battery"
        while not vehicle.mode == 'RTL':
            vehicle.mode = VehicleMode("RTL")
            time.sleep(1)

def checkPanic():
    if batt.level < panicLevel:
        print "**BATTERY CRITICALLY LOW**"
        print "Performing emergency landing"
        print "Position: %s" %vehicle.location.global_frame
        while not vehicle.mode == 'LAND':
            vehicle.mode = VehicleMode("LAND")
            time.sleep(1)

        # Wait for the drone to land
        while True:
            alt = vehicle.location.global_relative_frame.alt
            if alt <= 0.05:
                print "** Drone has landed **"
                break
            time.sleep(1)

        print "Disarming drone..."
        while not moveOnFlag:
            if errorCount > MAX_ERROR_COUNT:
                print "=== Errors on disarming the drone ==="
                end_program()
            vehicle.armed = False
            print "Waiting for disarming..."
            time.sleep(2)
        moveOnFlag = False
        vehicle.close()

        while True: #Continually transmitt location until power fails or manually interrupted
            print_location()
            time.sleep(1)
        



# Display some basic status information about the drone
def drone_info():
    # print " Type: %s" % vehicle.vehicle_type
    print " Armed: %s" % vehicle.armed
    print " Mode: %s" % vehicle.mode.name
    print " System status: %s" % vehicle.system_status.state
    print " GPS: %s" % vehicle.gps_0
    print " Location: %s,%s" % (vehicle.location.global_relative_frame.lat,
                                vehicle.location.global_relative_frame.lon)
    print " Relative Alt: %s" % vehicle.location.global_relative_frame.alt
    print " Absolute Alt: %s" % vehicle.location.global_frame.alt


# Display a full list of information about the drone
def drone_info_complete():
    print "Autopilot Firmware version: %s" % vehicle.version
    print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
    print "Global Location: %s" % vehicle.location.global_frame
    print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
    print "Local Location: %s" % vehicle.location.local_frame    #NED
    print "Attitude: %s" % vehicle.attitude
    print "Velocity: %s" % vehicle.velocity
    print "GPS: %s" % vehicle.gps_0
    print "Groundspeed: %s" % vehicle.groundspeed
    print "Airspeed: %s" % vehicle.airspeed
    print "Gimbal status: %s" % vehicle.gimbal
    print "Battery: %s" % vehicle.battery
    print "EKF OK?: %s" % vehicle.ekf_ok
    print "Last Heartbeat: %s" % vehicle.last_heartbeat
    print "Rangefinder: %s" % vehicle.rangefinder
    print "Rangefinder distance: %s" % vehicle.rangefinder.distance
    print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
    print "Heading: %s" % vehicle.heading
    print "Is Armable?: %s" % vehicle.is_armable
    print "System status: %s" % vehicle.system_status.state
    print "Mode: %s" % vehicle.mode.name    # settable
    print "Armed: %s" % vehicle.armed    # settable


# Prints the location of the drone in a format to use with Google Earth
def print_location():
    lon = vehicle.location.global_frame.lon
    lat = vehicle.location.global_frame.lat
    alt = vehicle.location.global_frame.alt
    groundspeed = vehicle.groundspeed
    airspeed = vehicle.airspeed
    print "%s,%s,%s" % (lon,lat,alt)
    # print "Airspeed: %s" % airspeed
    print "Groundspeed: %s" % groundspeed


# Move in a box shape
def box_maneuver():
    send_ned_velocity(2, 0, 0, 2)
    time.sleep(2)
    send_ned_velocity(0, 2, 0, 2)
    time.sleep(2)
    send_ned_velocity(-2, 0, 0, 2)
    time.sleep(2)
    send_ned_velocity(0, -2, 0, 2)
    time.sleep(2)


# Mavlink code to set the velocity and duration of a movement 
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # handle non-integer durations
    vehicle.send_mavlink(msg)
    time.sleep(duration%1)

    # send command to vehicle on 1 Hz cycle    
    for x in range(0, int(duration)):
        vehicle.send_mavlink(msg)
        print_location()
        time.sleep(1)

    stopmsg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(stopmsg) #  stop the drone after duration runs out


def send_ned_velocity_inst(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    NOTE: This does not have a duration, and drone will go until velocity is overridden or timed out.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)

def condition_yaw(heading, relative=False): #When not relative 0 degrees is north
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


# Check to see if the drone has arrived at the given location
def wait_for_arrival(lat, lon):
    timer = 3600
    while timer > 0:
        latitude = vehicle.location.global_relative_frame.lat
        longitude = vehicle.location.global_relative_frame.lon
        print_location()

        if (latitude >= lat-0.000005 and latitude <= lat+0.000005
        and longitude >= lon-0.000005 and longitude <= lon+0.000005):
            print "** Arrived at GPS destination **"
            break

        timer -= 1
        time.sleep(1)
    if timer <= 0:
        print "=== Timeout on waiting for coordinate arrival ==="
        # end_program()


def wait_for_altitude(alt):
    timer = 3600
    while timer > 0:
        altitude = vehicle.location.global_frame.alt
        if (altitude >= alt - 0.05 and altitude <= alt + 0.05):
            print "** Arrived at specified altitude **"
            break
        timer -= 1
        time.sleep(1)
    if timer <= 0:
        print "== Timeout on waiting for altitude arrival =="


# A simple pause to wait for the user to continue
def pause_for_input():
    raw_input("\n++ PRESS ENTER TO CONTINUE ++\n")


# Signal handler for when a keyboard interrupt occurs
def end_program(*args):
    # Set the mode to RETURN TO LAND and wait before closing
    while not vehicle.mode == 'LAND':
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)

    # Wait for the drone to land
    print "** Drone is landing **"
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print "Altitude: %s" % alt
        if alt <= 0.05:
            print "** Drone has landed **"
            break
        time.sleep(1)

    # Close the connection to the drone and to the vision program
    vehicle.close()
    vision.close()
    
    # Exit the program
    print "\nTEST COMPLETE\n"
    quit()


######################################################################
##                                                                  ##
##                             MAIN CODE                            ##
##                                                                  ##
######################################################################

# Set up interrupt handler
signal.signal(signal.SIGINT, end_program)

# Start simulation and connect to the drone
# HOME LOCATION (Test Site): 35.970264,-79.091461
# SITL command: dronekit-sitl copter-3.3 --home=35.970264,-79.091461,170,353
# print "** Start simulator (SITL) **"
# sitl = dronekit_sitl.start_default()
# connection_string = sitl.connection_string()

# connection_string = 'tcp:127.0.0.1:5760'
# connection_string = 'com3'
connection_string = '/dev/ttyACM0'
baud_rate = 115200
# baud_rate = 57600
print "** Connecting to vehicle on: %s **" % connection_string

# vehicle = connect(connection_string, wait_ready=True)
vehicle = connect(connection_string, baud=baud_rate)
batt = vehicle.battery

# Set up command message listeners
@vehicle.on_message('COMMAND_ACK')
def listener1(self, name, message):
    global moveOnFlag
    global errorCount
    print 'commandId: %s' % message.command
    def resultTable(x):
        return {0 : 'Command ACCEPTED and EXECUTED',
                1 : 'Command TEMPORARY REJECTED/DENIED',
                2 : 'Command PERMANENTLY DENIED',
                3 : 'Command UNKNOWN/UNSUPPORTED',
                4 : 'Command executed, but failed',
                5 : 'WIP: Command being executed',
        }.get(x, 'Unknown Result')
    print 'result: %s' % resultTable(message.result)
    if message.result == 0:
        moveOnFlag = True
        errorCount = 0
    else :
        errorCount += 1
@vehicle.on_message('ACTION_ACK')
def listener2(self, name, message):
    print 'message: %s' % message

# Display drone status
drone_info()

# Set flight parameters
alt = 20    # (in meters)
speed = 5   # (in meters/second)


# Arm the drone and takeoff
drone_arm()
drone_takeoff(30)

# Get the drone's home location
home_loc = vehicle.location
home_lat = home_loc.global_frame.lat
home_lon = home_loc.global_frame.lon
# pause_for_input()


# Goto first target location (O+5m north, O+5m east, O+5m alt)

location1 = getLoc(home_loc, 5, 5, 5)
print "** Going to first target location **"
vehicle.simple_goto(location1, groundspeed=speed)
wait_for_arrival(location1.lat, location1.lon)
# pause_for_input()

# Goto second target location (O-5m north, O+5m east, O+5m alt)
location2 = getLoc(home_loc, -5, 5, 5)
print "** Going to second target location **"
vehicle.simple_goto(location2, groundspeed=speed)
wait_for_arrival(location2.lat, location2.lon)
# pause_for_input()

# Return to the landing zone (O north, O east, O+5m alt)
location3 = getLoc(home_loc, 0, 0, 5)
print "** Returning to the landing zone **"
vehicle.simple_goto(location3, groundspeed=speed)
wait_for_arrival(home_lat, home_lon)
# pause_for_input()

# Fly the drone in a box maneuver
#print "** Flying in a box maneuver **"
#box_maneuver()
#print "** Box maneuver completed **"

# Prepare for landing
drone_info_complete()
print "Ready to land..."
# pause_for_input()
end_program()
