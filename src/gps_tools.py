#!/usr/bin/python2.7

""" GPS tools """
import pickle

import numpy
from sympy import Point, RegularPolygon, Line
# TODO(alberto) library.
import geodesy.utm # GPS processing

############## Conversion of GPS. ##############
def convert_gps_to_utm(lat, lon):#, alt=nan): # TODO(alberto) nan value.
    """Convert GPS coordinates to UTM.
        Note that in the North emisphere the system reference frame is
        positive x towards East, positive y towards North.

    Args:
        lat: latitude.
        lon: longitude.
        alt: altitude.
    Returns:
        UTM point p: x, y, z.
    Raises:
    """
    return geodesy.utm.fromLatLong(lat, lon).toPoint()
    
def convert_utm_to_gps(utm_point):
    """Convert UTM coordinates to GPS.

    Args:
        utm_point: UTM point p: x, y, z.
    Returns:
        GPS point: latitude, longitude, altitude.
    Raises:
    """
    return utm_point.toMsg()

def set_grid(utm_point, width, height, spacing, orientation=0):
    """Return UTM points of the grid. Points are in the center of the cells. 
        Boundary starts from utm_point to width and height. 
        The grid cell (0,0) is assumed to be centered in utm_point and the
        (width, height) has the highest values.

    Args:
        utm_point: UTM point p: x, y, z. It is assumed that utm_point is 
            starting from the lowest x and y.
        width: width of the grid (meters).
        length: length of the grid (meters).
        spacing: spacing between cells (meters). Assumed regular grid. 
        orientation: orientation (radians) of the grid around utm_point. 
            positive counterclockwise, negative clockwise.
    Returns:
        grid with UTM points of the grid. Note that indices are x,y.
    Raises:
    """
    # TODO(alberto) generalize to different grid shapes?
    # TODO(alberto) sanity checks.

    x_cells = numpy.arange(utm_point.x, utm_point.x+width, spacing)
    y_cells = numpy.arange(utm_point.y, utm_point.y+height, spacing)
    grid = []
    polygon_grid = []
    # Iterate over the x.
    max_y = 0
    max_x = 0
    for i, x in enumerate(x_cells):
        print i
        # TODO make consistent, usually y in rows.
        grid.append([])
        polygon_grid.append([])
        # Offset over the x, according to the column of the grid.
        dx = (float(i) * spacing + spacing / 2.0)
        # Iterate over the y.
        for j, y in enumerate(y_cells):
            print j
            # Current offset on y.
            dy = (float(j) * spacing + spacing / 2.0)
            # Current cell.
            current_x = utm_point.x + dx * numpy.cos(orientation) - dy * numpy.sin(orientation)
            current_y = utm_point.y + dx * numpy.sin(orientation) + dy * numpy.cos(orientation)

            grid[i].append((current_x, current_y))
            cell = RegularPolygon(Point(current_x, current_y), numpy.sqrt(2)*(spacing/2.0), 4, orientation)
            polygon_grid[i].append(cell)
        if i == 0:
            max_y = grid[i][-1]
    max_x = grid[-1][0]
    # TODO(alberto) use gtest.
    # Test. 
    
    with open("test.csv", 'w') as test_file:
        for i in range(len(grid)):
            for j in range(len(grid[i])):
                #zone, band = 11, 'N'
                #zone, band = 13, 'S' # for Durango
                zone, band = 21, 'N' # for Durango
                utm_point_current = geodesy.utm.UTMPoint(easting=grid[i][j][0], 
                    northing=grid[i][j][1], zone=zone,
                    band=band)
                gps_point_current = convert_utm_to_gps(utm_point_current)
                test_file.write("{},{}\n".format(gps_point_current.latitude, 
                    gps_point_current.longitude))
    axis_x = Line((utm_point.x, utm_point.y), max_x) # Segment along axis_x
    axis_y = Line((utm_point.x, utm_point.y), max_y) # Segment along axis_y
    return grid, polygon_grid, (axis_x, axis_y)

def main():
    """Test."""
    #utm_point = convert_gps_to_utm(33.44481, -118.48498)
    #Durango
    #utm_point = convert_gps_to_utm(37.29069,-107.84629)
    #width, height, spacing, orientation = 130, 150, 2.0, (numpy.pi/7)
    #utm_point = convert_gps_to_utm(37.23791, -107.90921)
    utm_point = convert_gps_to_utm(-2.23583, -54.65698)
    # positive angle: counterclockwise
    #width, height, spacing, orientation = 45, 70, 5.0, (numpy.pi/6)
    width, height, spacing, orientation = 2000, 2000, 20.0, 0

    grid, polygon_grid, (axis_x, axis_y) = set_grid(utm_point, width, height, spacing, orientation)

    with open('world.pickle', 'w') as f:  # Python 3: open(..., 'wb')
        pickle.dump([grid, polygon_grid, (axis_x, axis_y)], f)
if __name__ == '__main__':
    main()           
    
