#!/usr/bin/python2.7

"""exploration.py: According to the map, select points and evaluate them."""
import itertools
import random
import sys
import traceback
import time
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D

import numpy

from sympy import Polygon, Point, Symbol, Segment

import rospy
import message_filters
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
from heron_sonde.msg import Sonde
#from simulated_sensor.msg import ChloData
from gps_nav.srv import *
from simulated_sensor.msg import Measurement
from adaptive_sampling.msg import NavStatus, SampleCandidate, ExplorerPath

from gps_tools import *
import world

def locate_min(a):
    smallest = min(a)
    return smallest, [index for index, element in enumerate(a) 
                              if smallest == element]

def polygon_area(x,y):
    area = 0.5*numpy.abs(numpy.dot(x,numpy.roll(y,1))-numpy.dot(y,numpy.roll(x,1)))
    return area

class Evaluation(object):
    def __init__(self, weights, scale_factors):
        self.weights = weights
        self.scale_factors = scale_factors
        self.grow_weight = True
        
    def evaluate(self, current_pose, locations, world, time):
        """
        Args:
            current_pose: GPS pose of the robot.
            locations: pairs of integer, representing the cell index.
            world: world model.
        """
        # TODO(alberto) general evaluation class that allows to plugin different
        # criteria.
        locations, mean, variances = world.predict(locations)
        evaluation = [numpy.inf] * len(locations)
        current_pose_utm = convert_gps_to_utm(current_pose[0], current_pose[1])
        #print current_pose_utm
        distances = [numpy.inf] * len(locations)
        for i, l in enumerate(locations):
            # Distance. Poses and locations in meters.
            distances[i] = numpy.linalg.norm(numpy.array(l)-numpy.array([current_pose_utm.x, current_pose_utm.y]))

        max_d = max(distances)
        max_v = max(variances)

        if self.grow_weight:
            self.weights[1] = time / (time + 700.0) # TODO parameter.
            self.weights[0] = 1 - self.weights[1]
            print "WEIGHTS = "
            print self.weights

        #print variances
        for i, l in enumerate(locations):
            # Variance of the location.
            d = distances[i]
            v = variances[i]
            
            # Evaluation function.
            evaluation[i] = (d / max_d) * self.weights [0] + (1 - v / max_v) * self.weights[1]
        smallest, best_indices = locate_min(evaluation) 
        #print evaluation
        return best_indices#evaluation.index(min(evaluation))

class Exploration(object):
    def __init__(self, latitude, longitude, width, height, spacing, orientation, zone, band, weights, scale_factors, time_threshold=1000.0, replan_rate=1.0, plan_window=(12,12), bag_folder="", world_pickle=""):
        # TODO(alberto) general subscribing to topics.

        self.world = world.World(latitude, longitude, width, height, spacing, orientation=orientation, zone=zone, band=band, bag_folder=bag_folder, world_pickle=world_pickle)
        
        self.pub = rospy.Publisher('adaptive_sampling_status', NavStatus, queue_size=1)
        self.sample_pub = rospy.Publisher('/sample_candidates', SampleCandidate, queue_size=1)
        self.path_pub = rospy.Publisher('explorer_path',ExplorerPath, queue_size=1)
        self.waypoint_goto_result_sub = rospy.Subscriber('waypoint_goto_result', Int8, self.goto_waypoint_result_callback)
        #self.data_sub = rospy.Subscriber('/heron2/sonde', Sonde, self.data_callback)
        self.data_sub = rospy.Subscriber('chlorophyll_reading', Measurement, self.data_callback)
        
        #self.gps_sub = message_filters.Subscriber('navsat/fix', NavSatFix)
        #self.data_sub = message_filters.Subscriber('sonde', Sonde)
        #self.data_sub = message_filters.Subscriber('chlorophyll_reading',ChloData)
        #self.ts = message_filters.ApproximateTimeSynchronizer([self.gps_sub, self.data_sub], 10)
        #self.ts.registerCallback(data_callback)

            
        # TODO(alberto) parameters for evaluatio.
        self.evaluation = Evaluation(weights, scale_factors)
        self.curr_time=0 # not general, TODO from actual time.
        self.state = 0 # 0: IDLE, 1: GO_TO.
        self.path_state = 0 # 0: IDLE, 1: GO_TO.
        self.current_gps = None
        self.time_last_exploration_plan_computed = None
        self.candidate_list = []

        #print "Before request in Init"
        #self.request_gps()
        #print "after request in Init"
        self.last_data = None
        self.got_first_data_when_stopped = False
        
        self.time_threshold = time_threshold # Time threshold checked to cancel the last exploration waypoint.
        
        self.replan_rate = replan_rate
        
        self.plan_window = plan_window # (width, height).

    def request_gps(self):
        gps_msg = rospy.wait_for_message('navsat/fix', NavSatFix)
        self.current_gps = (gps_msg.latitude, gps_msg.longitude)
        #print self.current_gps

    def data_callback(self, data):
        #print "In data_callback of chlorophyll_reading"
        if data.header.stamp.secs != 0:
            self.curr_time = data.header.stamp.secs
            self.request_gps()
            if (self.state > 0) or (not self.got_first_data_when_stopped):
                self.last_data = data
                #self.world.add_measurement(data.header.stamp.secs, self.current_gps, data.clorophyll_u)
                gps_location = [data.latitude, data.longitude]
                self.world.add_measurement(data.header.stamp.secs, gps_location, data.data)
                sample_candidate_msg = SampleCandidate()
                sample_candidate_msg.header.stamp = rospy.get_rostime()
                sample_candidate_msg.latitude = gps_location[0]#self.current_gps[0]
                sample_candidate_msg.longitude = gps_location[1]#self.current_gps[1]
                sample_candidate_msg.data = self.last_data.data
                #sample_candidate_msg.data = self.last_data.clorophyll_u
                self.sample_pub.publish(sample_candidate_msg)
                if self.state != 0:
                    self.got_first_data_when_stopped = False
                else:
                    self.got_first_data_when_stopped = True
    
    def goto_waypoint_result_callback(self, msg):
        print "setting self.path_state to 0"
        if msg.data == 1:
            self.path_state = 0
        else:
            # TODO(alberto) recovery mechanism.
            rospy.logerr("goto failed.")
            self.path_state = 0

    def fixed_window(self, current_cell):
        x_values = range(max(0, current_cell[0] - self.plan_window[0] / 2), min(self.world.width-1, current_cell[0] + self.plan_window[0] / 2)+1)
        y_values = range(max(0, current_cell[1] - self.plan_window[1] / 2), min(self.world.height-1, current_cell[1] + self.plan_window[1] / 2)+1)
        #print x_values
        #print y_values
        c1=[[min(x_values)], y_values]
        c2=[[max(x_values)], y_values]
        c3=[x_values, [min(y_values)]]
        c4=[x_values, [max(y_values)]]
        l1=list(itertools.product(*c1))
        l2=list(itertools.product(*c2))
        l3=list(itertools.product(*c3))
        l4=list(itertools.product(*c4))
        candidate_list = list(set(l1+l2+l3+l4))
        self.candidate_list = list(set(self.candidate_list+candidate_list))

    def pick_locations_to_evaluate(self, method="contour"):
        """Returns a list of cells where the robot can go.
        Args:
            method: {fixed, contour}
        Return:
            List of cells.
        """
        current_cell = self.world.cell_corresponding_to_gps(self.current_gps[0], self.current_gps[1])
        current_pose_utm = convert_gps_to_utm(self.current_gps[0], self.current_gps[1])
        if method == "fixed":
            self.fixed_window(current_cell)
        elif method == "contour":
            # Create mesh TODO move to constructor.

            # Predict for the whole world.
            locations, mean, variances = self.world.predict(self.world.cell_locations)
            L = numpy.array(locations)
            #X_utm = L[:,0].reshape(self.world.width, self.world.height)
            #Y_utm = L[:,1].reshape(self.world.width, self.world.height)
            #mean = mean.reshape(self.world.width, self.world.height)
            #V = variances.reshape(self.world.width, self.world.height)
            X_utm = L[:,0].reshape(self.world.height, self.world.width)
            Y_utm = L[:,1].reshape(self.world.height, self.world.width)
            mean = mean.reshape(self.world.height, self.world.width)
            V = variances.reshape(self.world.height, self.world.width)
            # Find contours.
            '''
            plt.figure()
            plt.contourf(X_utm, Y_utm, mean)
            plt.colorbar()
            plt.draw() # TODO have a debug flag.
            plt.pause(0.001)
            plt.figure()
            plt.contourf(X_utm, Y_utm, V)
            plt.colorbar()
            '''
            try:
                C = plt.contour(X_utm, Y_utm, V, 8, colors='black', linewidth=.5)
                contour_with_robot = None
                # TODO not in try block.
                """
                # Find frontier.
                for c in C.collections:
                    #if c.get_paths()[0].contains_point((current_pose_utm.x, current_pose_utm.y)):
                    #p = Polygon(*c.get_paths()[0].to_polygons()[0])
                    p = c.get_paths()[0].to_polygons()[0]
                    p = numpy.array(p)
                    #print "p"
                    #print p
                    #print "contour_with_robot"
                    #print contour_with_robot
                    if contour_with_robot is not None:
                        if polygon_area(p[:,0], p[:,1]) > polygon_area(contour_with_robot[:,0], contour_with_robot[:,1]):
                            contour_with_robot = p
                            contour_obj = c
                    else:
                        contour_with_robot = p
                        contour_obj = c
                """

                contour_with_robot = numpy.array(C.collections[-1].get_paths()[0].to_polygons()[0])
                contour_obj = C.collections[-1]
                # Find locations on the frontier.
                self.candidate_list = []
                if contour_with_robot is not None:
                    contour_obj.set_linewidth(10)
                    #plt.plot(current_pose_utm.x, current_pose_utm.y, '*')
                    #plt.draw()
                    #plt.pause(0.001)

                    #contour_with_robot = Polygon(*contour_with_robot)
                    for i in xrange(len(contour_with_robot)-1):
                    #for s in contour_with_robot.sides:
                        s = Segment(contour_with_robot[i], contour_with_robot[i+1])
                        step_size = s.length / self.world.spacing
                        step_size = 1.0 / step_size.evalf()
                        t = Symbol('t', real=True)
                        #print s
                        #print contour_with_robot
                        #print "The vallue of S = "
                        #print s
                        point_t = s.arbitrary_point()

                        for step in numpy.arange(0.0, 1.0000001, step_size):
                            p = Point(point_t.x.subs(t, step), point_t.y.subs(t, step))
                            cell = self.world.cell_corresponding_to_gps(p.x.evalf(), p.y.evalf(), utm=True)
                            if cell not in self.candidate_list:
                                self.candidate_list.append(cell)

            except ValueError:
                # If no contour is found.
                rospy.logerr("No contour found, fixed window used.")
                self.fixed_window(current_cell)
            
        print "current cell", current_cell
        print "current gps", self.current_gps
        
        #print "Candidate list", self.candidate_list
        return self.candidate_list
        #candidate_locations = [x_values, y_values]
        #return list(itertools.product(*candidate_locations))
    
    def create_potential_field(self, x_goal, y_goal, x_obstacle, y_obstacle, xind, yind, x_mean, y_mean):
        y, x = numpy.mgrid[0:len(yind), 0:len(xind)]

        E = 0.1
        Eta = 1.2500
        T_d = 2

        Fa = E * ((x - x_goal)**2 + (y - y_goal)**2)
        dist = numpy.sqrt((x - x_obstacle)**2 + (y - y_obstacle)**2)
        Fr = numpy.zeros(dist.shape)
        #print "Fa = "
        #print Fa
        for i in range(0,dist.shape[0]):
            for j in range(0, dist.shape[1]):
                if dist[i][j] <= T_d:
                    if dist[i][j] == 0 and j != 0:
                        dist[i][j] = dist[i][j-1]
                    Fr[i][j] = Eta * ((1/dist[i][j] - 1/T_d)**2)
                else:
                    Fr[i][j] = 0

        E_mean = -0.3*Eta
        T_mean = T_d
        dist_mean = numpy.sqrt((x - x_mean)**2 + (y - y_mean)**2)
        F_mean = numpy.zeros(dist_mean.shape)
        for i in range(0,dist_mean.shape[0]):
            for j in range(0, dist_mean.shape[1]):
                if dist_mean[i][j] <= T_mean:
                    if dist_mean[i][j] == 0 and j != 0:
                        dist_mean[i][j] = dist_mean[i][j-1]
                        print "DIstance CHanged"
                    F_mean[i][j] = E_mean * ((1/dist_mean[i][j] - 1/T_mean)**2)
                else:
                    F_mean[i][j] = 0

        print "Fr+Fa+F_mean="
        #print Fr
        print Fr+Fa+F_mean
        print "[x_obstacle,y_obstacle,x_mean,y_mean,x_goal,y_goal]"
        print [x_obstacle,y_obstacle,x_mean,y_mean,x_goal,y_goal]
        return Fr+Fa+F_mean

    def choose_candidate(self, current_cell, prev_cell, potn_field, plan_window):
        y_values = range(max(0, current_cell[0] - plan_window[0] / 2), min(potn_field.shape[0]-1, current_cell[0] + plan_window[0] / 2)+1)
        x_values = range(max(0, current_cell[1] - plan_window[1] / 2), min(potn_field.shape[1]-1, current_cell[1] + plan_window[1] / 2)+1)
        c1=[[min(y_values)], x_values]
        c2=[[max(y_values)], x_values]
        c3=[y_values, [min(x_values)]]
        c4=[y_values, [max(x_values)]]
        l1=list(itertools.product(*c1))
        l2=list(itertools.product(*c2))
        l3=list(itertools.product(*c3))
        l4=list(itertools.product(*c4))
        #print "l4 = "
        #print l4
        candidate_list = list(set(l1+l2+l3+l4))
        candidate_list = [c for c in candidate_list if c != (current_cell[0],current_cell[1])]
        #candidate_list = [c for c in candidate_list if c != (prev_cell[0],prev_cell[1])]
        return candidate_list

    def compute_path_to_dest(self, x_start, y_start, x_goal, y_goal, potn_field, x_offset, y_offset):
        curr_cell = [y_start, x_start]
        plan_window = (5,5)
        path = [[x_start+x_offset, y_start+y_offset]]
        prev_cell = curr_cell
        while (curr_cell != [y_goal, x_goal]):
            print "GEtting candidate List"
            candidate_list = self.choose_candidate(curr_cell, prev_cell, potn_field, plan_window)
            candidate_values = []
            print candidate_list
            min_potn = potn_field[candidate_list[0]]
            min_candidate = candidate_list[0]
            for c in candidate_list:
                if(potn_field[c] < min_potn):
                    min_potn = potn_field[c]
                    min_candidate = c

            print curr_cell
            print prev_cell
            print potn_field
            potn_field[min_candidate] = numpy.max(potn_field)
            prev_cell = curr_cell
            curr_cell = [min_candidate[0],min_candidate[1]]
            path = path + [[min_candidate[1]+x_offset, min_candidate[0]+y_offset]]
        return path

    def find_potential_path(self,cur_cell, dest_cell):
        #Not using this functionality for now. Will return the dest_cell in the path
        print "Find potential path between :"
        print cur_cell
        print dest_cell
        if ((cur_cell[0] == dest_cell[0][0]) and (cur_cell[1] == dest_cell[0][1])):
            print "Return empty list as current and dest are same"
            return [[]]
        path = dest_cell
        return path
        #Not using this functionality for now. Will return the dest_cell in the path
        '''
        if ((cur_cell[0] == dest_cell[0][0]) or (cur_cell[1] == dest_cell[0][1])):
            return path
        xind = range(min(cur_cell[0],dest_cell[0][0]), max(cur_cell[0],dest_cell[0][0])+1)
        yind = range(min(cur_cell[1],dest_cell[0][1]), max(cur_cell[1],dest_cell[0][1])+1)

        x_offset = min(cur_cell[0],dest_cell[0][0])
        y_offset = min(cur_cell[1],dest_cell[0][1])
        loc_list = []
        #print xind
        #print yind
        for j in range(0,len(yind)):
            for i in range(0,len(xind)):
                loc_list.append([xind[i],yind[j]])

        #print loc_list
        locations, mean, variances = self.world.predict(loc_list)
        var = numpy.reshape(variances,(len(yind),len(xind)))
        mean = numpy.reshape(mean,(len(yind),len(xind)))
        #print var
        if(var.shape[0]==2 or var.shape[1]==2):
            return path

        #print min(numpy.min(var[1:-1,1:-1],axis=1))
        (yi,xi) = numpy.where(var[1:-1,1:-1]==(min(numpy.min(var[1:-1,1:-1],axis=1))))
        #Adding 1 as the indices are in terms of size of var[1:-1,1:-1]
        #We need it in terms of var matrix.
        x_obstacle, y_obstacle = xi[0]+1, yi[0]+1
        (yi,xi) = numpy.where(mean[1:-1,1:-1]==(min(numpy.min(mean[1:-1,1:-1],axis=1))))
        x_mean, y_mean = xi[0]+1, yi[0]+1
        x_goal, y_goal = dest_cell[0][0]-x_offset, dest_cell[0][1]-y_offset
        x_start, y_start = cur_cell[0]-x_offset, cur_cell[1]-y_offset

        print "Creating a Potential field"
        potn_field = self.create_potential_field(x_goal, y_goal, x_obstacle, y_obstacle, xind, yind, x_mean, y_mean)
        print "Compute the path using Potential Field"
        path = self.compute_path_to_dest(x_start, y_start, x_goal, y_goal, potn_field, x_offset, y_offset)
        path.pop(0)
        #print "The PATH is : "
        #print path
        return path
        '''


    def spin(self):    
        r = rospy.Rate(self.replan_rate)
        prev_best_candidate = []
        print "SPINNING"
        is_remove = False

        while not rospy.is_shutdown():
        
            print "Beginning of the spin loop"
            if self.state == 0:
                # Update the current pose.
                print "Before request GPS"
                self.request_gps()
                
                msg = NavStatus()
                msg.t = rospy.get_rostime()
                msg.curr_lat = self.current_gps[0] 
                msg.curr_lon = self.current_gps[1]

                # TODO choice of strategy.
                try:
                #if True:
                    rospy.loginfo("first try")
                    # Update the model.
                    #SANDEEP
                    if not is_remove:
                        print "Before world update"
                        if not self.world.update():
                            continue
                        #self.world.update()
                        print "WORLD UPDATE"

                        self.pick_locations_to_evaluate()
                        if not self.candidate_list:
                            # check if candidate list is empty or not.
                            continue
                    #print self.candidate_list
                    #Removing the closer candidates from the list
                    for i in range(0,len(prev_best_candidate)):
                        print "Removing "+str(i)+"th point from the list"
                        if prev_best_candidate[i] in self.candidate_list:
                            self.candidate_list.remove(prev_best_candidate[i])
                            #prev_best_candidate.remove(prev_best_candidate[i])
                        if not self.candidate_list:
                            is_remove = False
                            continue
                    prev_best_candidate = []
                    
                    print "SELECT LOCATIONS"
                    # According to the world, select a pose.
                    best_candidate_indices = self.evaluation.evaluate(self.current_gps, self.candidate_list, self.world, self.curr_time)
                    best_candidate_index = random.choice(best_candidate_indices)
                    print "EVALUATE LOCATIONS"
                    print self.candidate_list[best_candidate_index][0],self.candidate_list[best_candidate_index][1]
                    best_candidate = self.world.gps_corresponding_to_cell(self.candidate_list[best_candidate_index][0],self.candidate_list[best_candidate_index][1])
                    best_candidate_utm = convert_gps_to_utm(best_candidate.latitude,best_candidate.longitude)
                    current_pose_utm = convert_gps_to_utm(self.current_gps[0], self.current_gps[1])
                    dist_best_candidate = numpy.linalg.norm(numpy.array([best_candidate_utm.x,best_candidate_utm.y])-numpy.array([current_pose_utm.x, current_pose_utm.y]))
                    # SANDEEP: Removing the entry as the best candidate is very close to the current location.
                    # Keep the removed candidate and add it to the list in next iteration.
                    # This will help the explorer to choose a new candidate if the current candidate is very close, and also keep the current candidate for next run.
                    # print self.candidate_list
                    if dist_best_candidate < 1.5*self.world.spacing:
                        is_remove = True
                        print "Removing the entry as the best candidate is very close to the current location."
                        print (self.candidate_list[best_candidate_index][0],self.candidate_list[best_candidate_index][1])
                        #self.candidate_list.remove((self.candidate_list[best_candidate_index][0],self.candidate_list[best_candidate_index][1]))
                        prev_best_candidate=list(set(prev_best_candidate + [(self.candidate_list[best_candidate_index][0],self.candidate_list[best_candidate_index][1])]))
                        self.candidate_list.remove((self.candidate_list[best_candidate_index][0],self.candidate_list[best_candidate_index][1]))
                        print prev_best_candidate
                        continue
                    msg.status = 0
                    #We need not keep the candidates that are close to any visited location. Commenting next line.
                    #self.candidate_list = list(set(self.candidate_list + prev_best_candidate))
                    is_remove = False
                    prev_best_candidate = list(set(prev_best_candidate + [(self.candidate_list[best_candidate_index][0],self.candidate_list[best_candidate_index][1])]))
                    best_candidate_index_x = self.candidate_list[best_candidate_index][0]
                    best_candidate_index_y = self.candidate_list[best_candidate_index][1]

                except:
                    try:
                        traceback.print_exc(file=sys.stdout)
                        rospy.loginfo("second try")
                        #l = self.pick_locations_to_evaluate()
                        best_candidate_index = random.randint(0, len(self.candidate_list))
                        best_candidate_index_x = self.candidate_list[best_candidate_index][0] 
                        best_candidate_index_y = self.candidate_list[best_candidate_index][1] 
                    except:
                        traceback.print_exc(file=sys.stdout)
                        rospy.loginfo("third try")
                        best_candidate_index_x = random.randint(0, self.world.width-1)
                        best_candidate_index_y = random.randint(0, self.world.height-1)
                        
                    #best_candidate_index = random.randint(0, len(l))
                    #best_candidate = self.world.gps_corresponding_to_cell(l[best_candidate_index][0],l[best_candidate_index][1])
                    best_candidate = self.world.gps_corresponding_to_cell(best_candidate_index_x,best_candidate_index_y)
                    msg.status = 1

                current_cell = self.world.cell_corresponding_to_gps(self.current_gps[0], self.current_gps[1])
                dest_cell = [[best_candidate_index_x, best_candidate_index_y]]
                path = self.find_potential_path(current_cell, dest_cell)
                #If path has no waypoint, then restart the loop
                if path[0]==[]:
                    continue
                msg.dest_lat = best_candidate.latitude
                msg.dest_lon = best_candidate.longitude

                #Publish path onto the topic
                path_latlon_list = []
                for p in range(0,len(path)):
                    next_latlon = self.world.gps_corresponding_to_cell(path[p][0],path[p][1])
                    path_latlon_list = path_latlon_list + [next_latlon.latitude, next_latlon.longitude]

                path_msg = ExplorerPath()
                path_msg.header.stamp = rospy.get_rostime()
                path_msg.path = path_latlon_list
                self.path_pub.publish(path_msg)


                self.path_state = 0
                print "Length of the path ="
                print len(path)
                print path
                for p in range(0,len(path)):
                    self.state = 1
                    print "Goto to point : "
                    print path[p]
                    if self.path_state == 0:
                        next_candidate = self.world.gps_corresponding_to_cell(path[p][0],path[p][1])
                        print "GOTO", next_candidate
                        # Go to.
                        rospy.loginfo("Call goto service: lat {} lon {}".format(next_candidate.latitude, next_candidate.longitude))
                        #print "goto SERVICE"
                        rospy.wait_for_service('goto')
                        #print "goto SERVICE available"
                        gotoWaypoint = rospy.ServiceProxy('goto', Goto)
                        resp = gotoWaypoint(next_candidate.latitude, next_candidate.longitude) # lat, lon.
                        if resp:
                            result = True
                        else:
                            rospy.logerr("goto service failed.")
                            result = False
                        if result:
                            self.path_state = 1
                        self.time_last_exploration_plan_computed = rospy.get_rostime()
                    #elif self.path_state == 1:
                        current_time = rospy.get_rostime()
                        while ((current_time.secs - self.time_last_exploration_plan_computed.secs) < self.time_threshold) and (self.path_state != 0):
                            current_time = rospy.get_rostime()
                            #print "In sleep"
                            time.sleep(10)
                        self.path_state = 0
                print "setting self.state to 0"
                self.state = 0

            '''
            elif self.state == 1:
                current_time = rospy.get_rostime()
                if (current_time.secs - self.time_last_exploration_plan_computed.secs) > self.time_threshold:
                    # TODO(alberto) ensure that controller stops.
                    self.state = 0
            '''
            r.sleep()    

        
def main():
    #print "Python Matplotlib version"
    #print matplotlib.__version__
    rospy.init_node('adaptive_sampling', anonymous=False)  #initializing the node
    #latitude, longitude = (13.19131, -59.64222)
    #width, height, spacing, orientation = 90.0, 100.0, 2.0, 0.0
    #latitude, longitude = (33.44481, -118.48498)
    #width, height, spacing, orientation = 72.2376, 35.6616, 2.0, -0.6084887621843595
    #latitude, longitude = (37.2919, -107.84686)
    #width, height, spacing, orientation = 50.557, 50, 2.0, -0.6084887621843595
    #latitude, longitude = 37.23831,-107.90936
    #width, height, spacing, orientation = 130, 150, 5.0, (numpy.pi/7)

    #Adding changes for new simulated data

    # PARAMETER TO CHANGE! TODO parameters.

    ##################
    # Simulated data for Durango
    latitude, longitude = 37.23771, -107.90995 #37.2377, -107.90998 #37.23791, -107.90921
    width, height, spacing, orientation = 80, 90, 5.0, 0.0 # 85, 95, 5.0, 0.0
    zone, band = 13, 'S'
    world_pickle = ""
    
    # Real data from https://webmap.ornl.gov/ogc/wcsdown.jsp?dg_id=1000_1
    latitude, longitude = 2.23583, -54.65698
    width, height, spacing, orientation = 2000, 2000, 20.0, 0
    zone, band = 21, 'N'
    world_pickle = "/home/alberto/workspace/mrs2017/kf_simulation/src/adaptive_sampling/src/world.pickle"
    #world_pickle = "/home/sandeep/workspaces/kf_simulation/src/adaptive_sampling/src/world.pickle"


    # positive angle: counterclockwise
    #This was used with Nighthorse_inlet_turbid3.mat
    #width, height, spacing, orientation = 45, 70, 5.0, (numpy.pi/4)
    ###################

    ##################
    # TODO parameter!
    bag_folder = "/media/alberto/3dc9308e-f8a7-416e-9dc3-accc85ad48c5/alberto/workspace/icra2018/"
    #bag_folder = "/home/sandeep/workspaces/kf_simulation/bagfiles/"
    ##################

    ##################
    #Sandeep: Variance should have higher priority, hence higher weights. 
    #Because we are using (1-variance) in the cost function for evaluation.
    weights = [0.3, 0.7]
    scale_factors = [10.5 * numpy.sqrt(2), 1.0]
    ##################
    # END PARAMETER TO CHANGE

    exploration = Exploration(latitude, longitude, width, height, spacing, 
        orientation, zone, band, weights, scale_factors, bag_folder=bag_folder, world_pickle=world_pickle)
    exploration.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
