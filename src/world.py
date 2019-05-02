#!/usr/bin/python2.7

"""map.py: Stores the world."""
from datetime import datetime
import glob
import os
import pickle
import sys
import traceback

import numpy
from sympy import Polygon, Point

from matplotlib.ticker import FormatStrFormatter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

from gps_tools import *
from model import Model

class World(object):
    def __init__(self, latitude, longitude, width, height, spacing, 
        orientation=0.0,
        zone=13, band='S', bag_folder="", world_pickle=""): # TODO(alberto) areas not to visit.
        # bag_folder assumed to have as last character '/'
        
        # Create a grid.
        self.utm_point = convert_gps_to_utm(latitude, longitude)
        self.spacing = spacing
        self.orientation = orientation
        if world_pickle:
            self.grid, self.polygon_grid, self.axes = pickle.load(open(world_pickle, "rb" ))
        else:
            self.grid, self.polygon_grid, self.axes = set_grid(self.utm_point, width, height, spacing, orientation)
        self.width = len(self.grid)
        self.height = len(self.grid[0])
        self.zone = zone
        self.band = band

        X = numpy.arange(self.width)
        Y = numpy.arange(self.height)
        X, Y = numpy.meshgrid(X, Y)
        self.cell_locations = numpy.append(X.reshape(X.shape[0]*X.shape[1], 1), Y.reshape(Y.shape[0]*Y.shape[1],1), axis=1)

        # Measurements associated to each cell.
        self.measurements_to_cells = {}
        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                # TODO(alberto) areas not to visit.
                self.measurements_to_cells[','.join([str(i), str(j)])] = []
        
        self.model = Model() # TODO(alberto) parameters.
        if bag_folder:
            list_of_files = glob.glob(bag_folder + '/*bag*')
            latest_file = max(list_of_files, key=os.path.getctime)
            bag_dir, filename = os.path.split(latest_file)
            base_filename = filename.split(os.extsep)
            self.covered_string = os.path.join(bag_dir, base_filename[0] + '_covered.txt')
        else:
            self.experiment_time = datetime.now().strftime('%Y-%m-%d-%H:%M:%S')
            self.covered_string = 'covered_' + self.experiment_time + '.txt'
        self.num_measurements = 0

    def gps_corresponding_to_cell(self, i, j):
        """Return GPS point corresponding to center of the cell of the grid. 

        Args:
            i: integer, column.
            j: integer, row.
        Returns:
            GPS corresponding to i,j.
        Raises:
        """
        utm_point_current = geodesy.utm.UTMPoint(easting=self.grid[i][j][0], 
            northing=self.grid[i][j][1], zone=self.zone,
            band=self.band)
        gps_point_current = convert_utm_to_gps(utm_point_current)
        return gps_point_current
    
    def cell_corresponding_to_gps(self, latitude, longitude, utm=False):
        """Return indices of cell of the grid corresponding to GPS point. 

        Args:
            latitude: float.
            longitude: float.
        Returns:
            i,j cell.
        Raises:
        """
        if not utm:
            utm_point = convert_gps_to_utm(latitude, longitude)
        else:
            utm_point = Point(latitude, longitude) # TODO clean.
        x_distance = self.axes[1].distance(Point(utm_point.x, utm_point.y))
        y_distance = self.axes[0].distance(Point(utm_point.x, utm_point.y))
        i = int((x_distance / self.spacing).evalf())
        j = int((y_distance / self.spacing).evalf())
        if i < len(self.polygon_grid) and j < len(self.polygon_grid[i]):
            return i, j
        else:
            return None
        """
        for i in range(len(self.polygon_grid)):
            for j in range(len(self.polygon_grid[i])):
                if self.polygon_grid[i][j].encloses_point(Point(utm_point.x, utm_point.y)) or self.polygon_grid[i][j].distance(Point(utm_point.x, utm_point.y)) < 1.0:
                    return i, j
        
        return None
        """

    def add_measurement(self, time, location, measurement, 
        location_type='gps'): # TODO(alberto).
        """Add measurement, with UTM coordinates.
        Args:
            time: seconds.
            location: pairs of integer, representing the gps.
            measurement: single measurement.
        """
        try:
            i, j = self.cell_corresponding_to_gps(location[0], location[1])
            #i, j = 0, 0
            utm_point = convert_gps_to_utm(location[0], location[1])
            # TODO(alberto) Check whether the cell should be visited or not.
            #print i, j
            #SANDEEP : Check if this data already exists
            time1 = 0
            if ([time1,(utm_point.x, utm_point.y), measurement] not in self.measurements_to_cells[','.join([str(i), str(j)])]):
                self.measurements_to_cells[','.join([str(i), str(j)])].append([time1, (utm_point.x, utm_point.y), measurement])
                # Sandeep: Writing the data into a file.
                #print "Writing data to file"
                #print self.measurements_to_cells
                #print("%s,%s,%s,%s\n",time,location[0],location[1],measurement)
                with open(self.covered_string,'a') as f_handle:
                    f_handle.write(str(time)+','+str(location[0])+','+str(location[1])+','+str(measurement)+'\n')
                #np.savetxt(f_handle,self.measurements_to_cells, delimiter=',')
                #print "Done writing to the FILE"
                self.num_measurements += 1

        except:
            traceback.print_exc(file=sys.stdout)
            print "Outside the area"

    def predict(self, locations):
        """Return variances for each location, with UTM coordinates.
        Args:
            locations: pairs of integer, representing the cell index.
        """
        utm_coordinates = []
        #print "In world::predict()"
        #print locations
        for l in locations:
            utm_coordinates.append(self.grid[l[0]][l[1]])
        mean, var = self.model.predict(utm_coordinates)
        
        return utm_coordinates, mean, var

    def update(self):
        """Update the model with all the observations."""
        X = []
        y = []
        for i in range(self.width):
            for j in range(self.height):
                k = ','.join([str(i), str(j)])
                m = self.measurements_to_cells[k] 
                if m:
                    for measurement in m:
                        X.append([measurement[1][0], measurement[1][1]]) # TODO(alberto) use also time.
                        y.append([measurement[2]])
        if X and y:
            self.model.fit(X, y)
            #self.map_plot()
            return True
        else:
            return False

    def map_plot(self):
        #X = numpy.linspace(0, self.width-self.spacing, self.width/self.spacing)
        #Y = numpy.linspace(0, self.height-self.spacing, self.height/self.spacing)
        #X, Y = numpy.meshgrid(X, Y)
        X = numpy.zeros(len(self.grid[0]))
        Y = numpy.zeros(len(self.grid))
        Z = numpy.zeros((len(self.grid),len(self.grid[0])))
        V = numpy.zeros((len(self.grid),len(self.grid[0])))
        for i in xrange(len(self.grid)):
            Y[i] = self.grid[i][0][0]
            for j in xrange(len(self.grid[0])):
                X[j] = self.grid[0][j][1]
                datum = (self.grid[i][j][0], self.grid[i][j][1])
                mean, variance = self.model.predict([datum])
                Z[i][j] = mean[0]
                V[i][j] = variance[0]
        plt.figure()
        plt.contourf(X, Y, Z)
        plt.colorbar()
        C = plt.contour(X, Y, Z, 8, colors='black', linewidth=.5)
        plt.gca().yaxis.set_major_formatter(FormatStrFormatter('%.8f'))
        plt.gca().xaxis.set_major_formatter(FormatStrFormatter('%.8f'))
        plt.figure()
        plt.contourf(X, Y, V)
        plt.colorbar()
        C = plt.contour(X, Y, V, 8, colors='black', linewidth=.5)
        plt.gca().yaxis.set_major_formatter(FormatStrFormatter('%.8f'))
        plt.gca().xaxis.set_major_formatter(FormatStrFormatter('%.8f'))
        plt.show()

def main():
    """Test."""
    latitude, longitude = (33.44481, -118.48498)
    width, height, spacing, orientation = 72.2376, 35.6616, 2.0, -0.6084887621843595
    test_map = World(latitude, longitude, width, height, spacing, 
        orientation)

if __name__ == '__main__':
    main()  
   
