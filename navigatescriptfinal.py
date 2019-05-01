#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import serial
import time
import datetime
import csv
import threading

# Initialize ROS node
rospy.init_node("test_drive")

# Create ROS publisher
cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)



ser = serial.Serial(
               port='/dev/ttyUSB0',
               baudrate = 9600,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS,
               timeout= .5
)
global bearing
bearing = 0

def bearingf(coord1, coord2):
    """This finds the bearing between two lat-long coordinates"""
    deltalong = coord2[1] - coord1[1]
    x = math.cos(coord2[0] * (math.pi / 180)) * math.sin(deltalong * (math.pi / 180))
    y = math.cos(coord1[0] * (math.pi / 180)) * math.sin(coord2[0] * (math.pi / 180)) - math.sin(
        coord1[0] * (math.pi / 180)) * math.cos(coord2[0] * (math.pi / 180)) * math.cos(deltalong * (math.pi / 180))
    brng = math.atan2(x, y)
    return brng


def distance(coord1, coord2):
    """This finds the bearing between two lat-long coordinates"""
    deltalong = coord2[1] - coord1[1]
    deltalat = coord2[0] - coord1[0]
    a = (math.sin((deltalat * (math.pi / 180)) / 2)) ** 2 + math.cos(coord1[0] * (math.pi / 180)) * math.cos(
        coord2[0] * (math.pi / 180)) * (math.sin((deltalong * (math.pi / 180)) / 2)) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = 6371000 * c
    return d


def angle(coord1, coord2):
    """This finds the angle between two grid coordinates"""
    deltax = coord2[0] - coord1[0]
    deltay = coord2[1] - coord1[1]
    ang = math.atan2(deltax, deltay)
    return ang


def distanceflat(coord1, coord2):
    """This finds the distance between two grid coordinates"""
    deltax = coord2[0] - coord1[0]
    deltay = coord2[1] - coord1[1]
    dist = math.sqrt((deltax) ** 2 + (deltay) ** 2)
    return dist


def gpstogrid(coord1, coord2):
    """This converts GPS coordinates to grid coordinates, with the origin being defined as coord1"""
    r = distance(coord1, coord2)
    theta = bearing(coord1, coord2)
    x = math.sin(theta) * r
    y = math.cos(theta) * r
    return (x, y)


def gridtogps(coord1gps, coord2grid):
    """this converts grid coordinates to gps coordinates"""
    r = distanceflat([0, 0], coord2grid)
    theta = angle([0, 0], coord2grid)

    lat2 = math.asin(
        math.sin(math.radians(coord1gps[0])) * math.cos(r / 6371000) + math.cos(math.radians(coord1gps[0])) * math.sin(
            r / 6371000) * math.cos(theta))
    lon2 = math.radians(coord1gps[1]) + math.atan2(
        math.sin(theta) * math.sin(r / 6371000) * math.cos(math.radians(coord1gps[0])),
        math.cos(r / 6371000) - math.sin(math.radians(coord1gps[0])) * math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return (lat2, lon2)


def drive(linear, angular, length):
    # Initialize ROS message object
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular

    for _ in range(length):  # repeat length  times
        cmd_pub.publish(twist)  # publish message
        rospy.sleep(0.1)  # sleep for 100ms


def navigate(destination):
    a=1
    while not rospy.is_shutdown():
        global bearing
        location1 = gpscoords[-1]
        route = bearingf(location1, destination)
        if route/abs(route)==-1:
            route=2*np.pi+route
        difference = route-bearing
        if abs(difference)>np.pi:
            difference=-(difference/abs(difference))*(2*np.pi-abs(difference))
        print("Bearing is:",(bearing/np.pi *180),"\n Difference is:",(difference/np.pi *180), "location bearing is", (route/np.pi *180))
        time.sleep(1)
        if abs(difference) > (1 / 40) * math.pi:
            direction = difference / abs(difference)
            length = abs(difference)
            drive(0.0, -direction, int(length*10*a))
            time.sleep(1)
        drive(.2, 0.0, 120)
        location2 = gpscoords[-1]
        previousbearing=bearing
        bearing = bearingf(location1, location2)
        if abs(previousbearing-bearing)>math.pi/10 and distance(location2,destination)>10: #this should correct the turn amount if there is increased friction or low battery
            a=abs((difference)/(route-bearing))
        if distance(location2, destination) < 3:
            break



def mapper(corner1, corner2, corner3, corner4, num):
    corner1grid = gpstogrid(corner1, corner1)
    corner2grid = gpstogrid(corner1, corner2)
    corner3grid = gpstogrid(corner1, corner3)
    corner4grid = gpstogrid(corner1, corner4)
    coords = np.array([corner1grid, corner2grid, corner3grid, corner4grid])
    dist1 = []
    for i in range(4):
        length = distance(coords[0], coords[i])
        dist1.append(length)
    maxi = dist1.index(max(dist1))
    zero = dist1.index(0)
    print(dist1)

    del dist1[zero]
    print(dist1)
    mini = dist1.index(min(dist1))
    print(mini)
    x1 = np.linspace(coords[0, 0], coords[mini + 1, 0], num)
    y1 = np.linspace(coords[0, 1], coords[mini + 1, 1], num)
    line1 = np.column_stack([x1, y1])

    indexnums = [0, 1, 2, 3]
    indexnums.remove(zero)
    indexnums.remove(maxi)
    indexnums.remove(mini + 1)
    medium = indexnums.pop(0)

    x2 = np.linspace(coords[medium, 0], coords[maxi, 0], num)
    y2 = np.linspace(coords[medium, 1], coords[maxi, 1], num)
    print(y2)
    line2 = np.column_stack([x2, y2])

    travelcoords = np.empty((num ** 2, 2))
    for i in range(num):
        xline = np.linspace(line1[i, 0], line2[i, 0], num)
        yline = np.linspace(line1[i, 1], line2[i, 1], num)
        line = np.column_stack([xline, yline])
        if i % 2 == 0:
            for j in range(num):
                travelcoords[num * i + j] = line[j]
        if i % 2 != 0:
            for j in range(num):
                travelcoords[num * i + j] = line[num - j - 1]
    for i in range(num ** 2):
        navigate(gridtogps(corner1, travelcoords[i]))




file_name = 'D3S_' + str(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')) + '.csv'

with open('/home/matt/Desktop/Data/'+file_name, "ab") as f:
    writer = csv.writer(f)
    writer.writerow(['Gamma_cps', 'Neutron_cps', 'GPS_Time',
                     'Latitude', 'N_or_S', 'Longitude', 'E_or_W'])



class datarecorder(threading.Thread):

    def __init__(self):
        # calling superclass init
        self._stopevent = threading.Event()
        self._sleepperiod=0.5
        threading.Thread.__init__(self)


    def run(self):
        while not self._stopevent.isSet():
            line = ser.readline()
            time.sleep(0.1)
            if line.startswith('SGM'):
                line = line.split()
                test=("E","W")
                if len(line) == 10:
                    print(line)
                    lat=float(float(str(line[6][:2]))+float(str(line[6][2:]))/60) # translates the coordinate
                    if line[7] == "S":  # multiplies the lat degree by -1 if in the southern hemisphere
                        lat = lat * -1
                    long = float(float(str(line[8][:3])) + float(str(line[8][3:])) / 60)
                    if line[9] == "W":  # multiplies the long degree by -1 if in the western hemisphere
                        long = long * -1
                    coord = (lat, long)
                    gpscoords.append(coord)
                    with open('/home/matt/Desktop/Data/'+file_name, "ab") as f:
                        writer = csv.writer(f)
                        writer.writerow([line[2], line[4], line[5], line[6], line[7], line[8], line[9]])

                        time.sleep(0.1)

                if len(line) < 10:
                    time.sleep(0.1)
            self._stopevent.wait(self._sleepperiod)
    def join(self, timeout=None):
        self._stopevent.set()
        threading.Thread.join(self, timeout)

coord1=51.471131666666665 #enter lat and long here
coord2=-2.617453333333333
bearing=0


def Main():
    global gpscoords
    gpscoords = []
    background = datarecorder()
    background.start()
    while True:
        print("Finding satellites")
        print(gpscoords)
        time.sleep(5)
        if len(gpscoords)>4:
            print("Satellites found")
            navigateto=(coord1,coord2)
            navigate(navigateto)
            break
    background.join()


if __name__ == '__main__':
    Main()
