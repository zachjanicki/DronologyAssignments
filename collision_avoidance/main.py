import dronekit_sitl
import dronekit
from dronekit import LocationGlobalRelative
import json
import argparse
import os
import threading
import time
import signal
import util
import logging
import math

_LOG = logging.getLogger(__name__)
_LOG.setLevel(logging.INFO)

fh = logging.FileHandler('main.log', mode='w')
fh.setLevel(logging.INFO)
formatter = logging.Formatter('| %(levelname)6s | %(funcName)8s:%(lineno)2d | %(message)s |')
fh.setFormatter(formatter)
_LOG.addHandler(fh)


DO_CONT = False

# make sure you change this so that it's correct for your system 
ARDUPATH = os.path.join('/', 'home', 'michael', 'git', 'ardupilot')

R = 6373.0


def get_distance_meters(lat1, long1, lat2, long2):
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    long1 = radians(long1)
    long2 = radians(long2)

    dlong = long2 - long1
    dlat = lat2 - lat1

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlong / 2)**2

    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = R * c * 1000
    return distance

# take the second element of a list. used when sorting our drones
def take_second(l):
    return l[1]

def sort_by_height(vehicles, routes, index):
    altitude_list = []
    vindex = 0 # drone number we're looking at
    drone_count = len(vehicles)

    for vroute in routes:
        lat, lon, alt = vroute[index]
        altitude_list.append((vindex, alt)) # list of altitudes for indexed
        # waypoint 
        vindex += 1

    # sort by altitude
    altitude_list = sorted(altitude_list, key=take_second)

    # iterate through the list again and tell each drone to go to a new alt
    for i in range(len(altitude_list)):
        d_id, alt = altitude_list[i]
        newalt = 20 + i*(20 / drone_count)
        go_to_altitude(newalt, vehicles[d_id])

def go_to_altitude(alt, vehicle):
    # get the current location, so when we move we just change altitudes
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    newpoint = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(newpoint, groundspeed=10)
    # check that we made it to a safe height
    while True:
        currheight = vehicle.location.global_relative_frame.alt
        if currheight > alt * 0.95 and currheight < alt * 1.05:
		print("Reached target altitude")
                break
        time.sleep(1)

def load_json(path2file):
    d = None
    try:
        with open(path2file) as f:
            d = json.load(f)
    except Exception as e:
        exit('Invalid path or malformed json file! ({})'.format(e))

    return d


def connect_vehicle(instance, home):
    home_ = tuple(home) + (0,)
    home_ = ','.join(map(str, home_))
    sitl_defaults = os.path.join(ARDUPATH, 'Tools', 'autotest', 'default_params', 'copter.parm')
    sitl_args = ['-I{}'.format(instance), '--home', home_, '--model', '+', '--defaults', sitl_defaults]
    sitl = dronekit_sitl.SITL(path=os.path.join(ARDUPATH, 'build', 'sitl', 'bin', 'arducopter'))
    sitl.launch(sitl_args, await_ready=True)

    tcp, ip, port = sitl.connection_string().split(':')
    port = str(int(port) + instance * 10)
    conn_string = ':'.join([tcp, ip, port])

    vehicle = dronekit.connect(conn_string)
    vehicle.wait_ready(timeout=120)

    return vehicle, sitl


def get_vehicle_id(i):
    return 'drone{}'.format(i)


def state_out_work(dronology, vehicles):
    while DO_CONT:
        for i, v in enumerate(vehicles):
            state = util.StateMessage.from_vehicle(v, get_vehicle_id(i))
            state_str = str(state)
            _LOG.info(state_str)
            dronology.send(state_str)

        time.sleep(1.0)
        

# Returns line segment offset d units from given line
def offset(line, d):
    p, q = line
    x1 = p[0]
    x2 = q[0]
    y1 = p[1]
    y2 = q[1]
    dx = (x2-x1)*d
    dy = (y2-y1)*d
    return ((x1-dy, y1+dx),(x2-dy, y2+dx))

# Returns whether intersection occurs within distance of two lines
# Creates paralell line segments offset from two routes and checks for intersections
def will_collide(line1, line2):
    
    distance = 0.01
    
    l1_upper_bound = offset(line1, distance)
    l1_lower_bound = offset(line1, -distance)
    l2_upper_bound = offset(line2, distance)
    l2_lower_bound = offset(line2, -distance)

    if intersection(l1_upper_bound, l2_upper_bound):
        return True
    elif intersection(l1_upper_bound, l2_lower_bound):
        return True
    elif intersection(l1_lower_bound, l2_upper_bound):
        return True
    elif intersection(l1_lower_bound, l2_lower_bound):
        return True
    else:
        return False
    
# Returns whether or not intersection exists between line segments
def intersection(line1, line2): 
    
    pt1, pt2 = line1
    ptA, ptB = line2
    
    DET_TOLERANCE = 0.00000001

    # the first line is pt1 + r*(pt2-pt1)
    # in component form:
    x1, y1 = pt1;   x2, y2 = pt2
    dx1 = x2 - x1;  dy1 = y2 - y1

    # the second line is ptA + s*(ptB-ptA)
    x, y = ptA;   xB, yB = ptB;
    dx = xB - x;  dy = yB - y;

    DET = (-dx1 * dy + dy1 * dx)

    if math.fabs(DET) < DET_TOLERANCE:
        # return (0,0,0,0,0)
        return False

    # now, the determinant should be OK
    DETinv = 1.0/DET

    # find the scalar amount along the "self" segment
    r = DETinv * (-dy  * (x-x1) +  dx * (y-y1))

    # find the scalar amount along the input line
    s = DETinv * (-dy1 * (x-x1) + dx1 * (y-y1))

    # return the average of the two descriptions
    xi = (x1 + r*dx1 + x + s*dx)/2.0
    yi = (y1 + r*dy1 + y + s*dy)/2.0
    # return ( xi, yi, 1, r, s )
    return True


def main(path_to_config, ardupath=None):
    if ardupath is not None:
        global ARDUPATH
        ARDUPATH = ardupath
    
    global DO_CONT
    DO_CONT = True

    config = load_json(path_to_config)
    dronology = util.Connection()
    dronology.start()

    # A list of sitl instances.
    sitls = []
    # A list of drones. (dronekit.Vehicle)
    vehicles = []
    # A list of lists of lists (i.e., [ [ [lat0, lon0, alt0], ...] ...]
    # These are the waypoints each drone must go to!
    routes = []

    # Example:
    # vehicle0 = vehicles[0]
    # waypoints_for_vehicle0 = routes[0]
    # for waypoint in waypoints_for_vehicle0:
    #    lat, lon, alt = waypoint
    #    vehicle0.simple_goto(lat, lon, alt)

    # The above example obviously won't work... you'll need to write some code to figure out when the current waypoint
    # has been reached and it's time to go to the next waypoint.

    # Define the shutdown behavior
    def stop(*args):
        global DO_CONT
        DO_CONT = False
        w0.join()

        for v, sitl in zip(vehicles, sitls):
            v.close()
            sitl.stop()

        dronology.stop()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    
    # Start up all the drones specified in the json configuration file
    for i, v_config in enumerate(config):
        home = v_config['start']
        vehicle, sitl = connect_vehicle(i, home)

        handshake = util.DroneHandshakeMessage.from_vehicle(vehicle, get_vehicle_id(i))
        dronology.send(str(handshake))

        sitls.append(sitl)
        vehicles.append(vehicle)
        routes.append(v_config['waypoints'])
        
    # Create a thread for sending the state of drones back to Dronology
    w0 = threading.Thread(target=state_out_work, args=(dronology, vehicles))
    # Start the thread.
    w0.start()

    # At this point, all of the "behind the scenes stuff" has been set up.
    # It's time to write some code that:
    #   1. Starts up the drones (set the mode to guided, arm, takeoff)

    for vehicle in vehicles:
        print("Starting vehicle {}".format(vehicle))
        starting_altitude = 20 # unsure about this number
        print("Basic pre-arm checks")
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode = dronekit.VehicleMode("GUIDED")
        vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        vehicle.simple_takeoff(starting_altitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto
        #  (otherwise the command after Vehicle.simple_takeoff will execute
        #   immediately).
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt >= starting_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

   # end the loop above (for v in vehicles)

    # For each set of waypoints
    for waypointindex in range(0, 5):
        
        sort_by_height(vehicles, routes, waypointindex)
        
        # Send each drone to it next waypoint
        for vindex in range(len(vehicles)):
            lat, lon, alt = routes[vindex][waypointindex] 
            waypoint = LocationGlobalRelative(lat, lon, vehicles[vindex].location.global_relative_frame.alt)
            vehicles[vindex].simple_goto(waypoint, groundspeed=10)
        
        # Check if collisions occur
        for i, vehicle in enumerate(vehicles):
            v1_curr = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
            v1_goto = (routes[i][waypointindex][0], routes[i][waypointindex][1])
            v1 = (v1_curr, v1_goto)
            target_altitude = routes[i][waypointindex][2]

            for j, colliding_vehicle in enumerate(vehicles):
                v2_curr = (colliding_vehicle.location.global_relative_frame.lat, colliding_vehicle.location.global_relative_frame.lon)
                v2_goto = (routes[j][waypointindex][0], routes[j][waypointindex][1])
                v2 = (v2_curr, v2_goto)
                
                if not will_collide(v1, v2):
                    print "Descending / ascending to target alt"
                    go_to_altitude(target_altitude, vehicle)
                else:
                    if get_distance_meters(v1_curr,v1_goto,v2_curr,v2_goto) > 20:
                        print "Descending / ascending if not too close"
                        go_to_altitude(target_altitude, vehicle)
                    else:
                        print "Waiting for drone to pass"
                        time.sleep(20.0)
                        go_to_altitude(target_altitude, vehicle)


    # wait until ctrl c to exit
    while DO_CONT:
        time.sleep(5.0)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('path_to_config', type=str, help='the path to the drone configuration file.')
    ap.add_argument('--ardupath', type=str, default=ARDUPATH)
    args = ap.parse_args()
    main(args.path_to_config, ardupath=args.ardupath)
