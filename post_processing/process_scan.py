import io
import os
import sys
import math
import serial
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as axes3d

# conversion factor from servo pwm signal in microseconds
# to servo offset from neutral in degrees
US_DEG = 0.096982146779149

# conversion from distance sensor ADC reading (0-1023)
# to sensor voltage out (0-5V)
SIGNAL_V = 5.0/1023
# calibration curve from sensor voltage to object distance (in cm)
V_CM =   [[2.75, 15],
          [2.51, 20],
          [1.99, 30],
          [1.52, 40],
          [1.23, 50],
          [1.05, 60],
          [0.92, 70],
          [0.81, 80],
          [0.73, 90],
          [0.67, 100],
          [0.59, 110],
          [0.54, 120],
          [0.50, 130],
          [0.48, 140],
          [0.46, 150]]
# distance (cm) from origin of servo axes to sensor read point
D_OFF = 3

# height (cm) from table to origin of servo axes
H_OFF = 0


def convert_data(data):
    SPHERE_DATA = []
    CART_DATA = []

    for data_point in data:
        sphere_point = data_to_sphere(data_point)
        cart_point = sphere_to_cart(sphere_point)
        SPHERE_DATA.append(sphere_point)
        CART_DATA.append(cart_point)

    return SPHERE_DATA, CART_DATA


def plot_scan(data, sphere, cart):

    x, y, z, r = ([],[],[],[])

    for i in range(len(data)):
        if(data[i][2]>250 and data[i][2]<350):
            x.append(cart[i][0])
            y.append(cart[i][1])
            z.append(cart[i][2])
            r.append(sphere[i][0])

    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z, c=r, cmap='viridis')
    plt.show()



def data_to_sphere(data):
    # accepts triplet of [pan angle (us), tilt angle (us), sensor reading (adc)]
    # pan/tilt -> difference in microseconds (directly proportional to angular displacement)
    # between current servo position and neutral (+x axis) along their respective axes of rotation

    # we want a sperical coordinate of the form [r (cm), theta (deg), phi (deg)]
    # r -> distance from the origin (intersection of pan/tilt axes)
    # theta -> azimuth, angular displacement in x-y plane from +x axis
    # phi -> zenith, angular displacement from +z axis normal to x-y plane

    r = adc_to_distance(data[2]) + D_OFF
    t = us_to_angle(data[0])
    p = 90 - us_to_angle(data[1])

    return (r, t, p)


def sphere_to_cart(point):
    # takes a point in spherical coordinates [r (cm), theta (deg), phi (deg)]
    # and converts it to cartesian coordinares [x (cm), y(cm), z(cm)]

    r = point[0]
    t = point[1]
    p = point[2]

    x = r * math.cos(math.radians(t)) * math.sin(math.radians(p))
    y = r * math.sin(math.radians(t)) * math.sin(math.radians(p))
    z = r * math.cos(math.radians(p)) + H_OFF # add H_OFF to make surface of table z = 0

    return (x, y, z)



def us_to_angle(microseconds):
    # convert from us to deg
    degrees = microseconds*US_DEG

    return degrees



def adc_signal(reading):
    # convert from reading to voltage
    v = reading * SIGNAL_V

    return v


def adc_to_distance(reading):
    # convert from reading to voltage
    v = reading * SIGNAL_V

    # interpolating from the calibration curve given in the datasheet
    # approximate the distance between the sensor and the detected object
    d_sensor = linear_interpolate(v, V_CM)

    # origin -> object = origin -> sensor + sensor -> object
    distance = d_sensor + D_OFF

    return distance



def linear_interpolate(xx, points):
    # arrange curve points by ascending x value
    points = sorted(points, key=lambda x: x[0])
    n = len(points)

    # find points pa and pb st pa_x < xx < pb_x
    if points[1][0] <= xx and xx <= points[n-2][0]:
        # initialize point indexing variable and flag
        i = 1
        found = False

        # iterate through list of points until pa/pb bounding xx gave been found
        while(not found and i < n - 2):

            # check if xx falls between the x value of a point with a given index
            # and the x value of the point with the proceeding index
            if points[i][0] <= xx and xx <= points[i+1][0]:
                # if conditions are met, you have found p0 and p1
                pa = points[i]
                pb = points[i+1]
                found = True

            else:
                # increment index to continue iteration
                i += 1

    # account for cases in which xx falls outside boundaries
    elif xx < points[1][0]:
        i = 0
        pa = points[0]
        pb = points[1]
    else:
        i = n-2
        pa = points[n - 2]
        pb = points[n - 1]

    print('\n pa -> p', i, ': ', pa)
    print(' pb -> p', i+1, ': ', pb)

    y0 = pa[1]

    # calculate slope of line from pa to pb
    dy_dx = (pb[1]-pa[1])/(pb[0]-pa[0])
    # calculate change in x from pa to xx
    del_x = xx - pa[0]

    # linearly approximate the y value corresponding to xx
    yy = y0 + dy_dx * del_x

    return yy


def parse_scan(filename):
    source = open(filename, 'r')
    data = []
    lines = source.readlines()
    for line in lines:
        line = line.split('}')[0]
        line = line.strip('@{')
        str_data = line.split(',')
        data.append((int(str_data[0]), int(str_data[1]), int(str_data[2])))

    print(data)

    return data

if __name__ == '__main__':
    scandata = parse_scan('scanres.txt')
    sphere, cart = convert_data(scandata)
    plot_scan(scandata, sphere, cart)
