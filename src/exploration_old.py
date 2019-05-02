#!/usr/bin/python2.7

"""exploration.py: According to the map, select points and evaluate them."""
import itertools
import random

import numpy

import rospy
import message_filters
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
#from heron_water.msg import Sonde
from simulated_sensor.msg import ChloData
from gps_nav.srv import *
from adaptive_sampling.msg import NavStatus

from gps_tools import *
import world

class Evaluation(object):
    def __init__(self, weights, scale_factors):
        self.weights = weights
        self.scale_factors = scale_factors
        
    def evaluate(self, current_pose, locations, world):
        """
        Args:
            current_pose: GPS pose of the robot.
            locations: pairs of integer, representing the cell index.
            world: world model.
        """
        # TODO(alberto) general evaluation class that allows to plugin different
        # criteria.
        locations, variances = world.predict(locations)
        evaluation = [inf] * len(locations)
        current_pose_utm = convert_gps_to_utm(lat, lon)
        for i, l in enumerate(locations):
            # Distance. Poses and locations in meters.
            d = numpy.linalg.norm(l-current_pose)
            
            # Variance of the location.
            v = variances[i]
            
            # Evaluation function.
            evaluation[i] = d / scale_factors[0] * weights [0] + v / scale_factors[1] * weights[1]
        
        return evaluation.index(min(evaluation))

class Exploration(object):
    def __init__(self, latitude, longitude, width, height, spacing, orientation, weights, scale_factors, time_threshold=100.0, replan_rate=1.0, plan_window=(5,5)):
        # TODO(alberto) general subscribing to topics.
        
        self.pub = rospy.Publisher('adaptive_sampling_status', NavStatus, queue_size=1)
        self.waypoint_goto_result_sub = rospy.Subscriber('waypoint_goto_result', Int8, self.goto_waypoint_result_callback)
        self.data_sub = rospy.Subscriber('chlorophyll_reading', ChloData, self.data_callback)
        
        #self.gps_sub = message_filters.Subscriber('navsat/fix', NavSatFix)
        #self.data_sub = message_filters.Subscriber('sonde', Sonde)
        #self.data_sub = message_filters.Subscriber('chlorophyll_reading',ChloData)
        #self.ts = message_filters.ApproximateTimeSynchronizer([self.gps_sub, self.data_sub], 10)
        #self.ts.registerCallback(data_callback)
        self.world = world.World(latitude, longitude, width, height, spacing, 
            orientation)
            
        # TODO(alberto) parameters for evaluatio.
        self.evaluation = Evaluation(weights, scale_factors)
        
        self.state = 0 # 0: IDLE, 1: GO_TO.
        self.current_gps = None
        self.time_last_exploration_plan_computed = None
        #print "Before request in Init"
        #self.request_gps()
        #print "after request in Init"
        
        self.time_threshold = time_threshold # Time threshold checked to cancel the last exploration waypoint.
        
        self.replan_rate = replan_rate
        
        self.plan_window = plan_window # (width, height).

    def request_gps(self):
        gps_msg = rospy.wait_for_message('/navsat/fix', NavSatFix)
        self.current_gps = (gps_msg.latitude, gps_msg.longitude)

    def data_callback(self, data):
        if data.header.stamp.secs != 0:
            gps = rospy.wait_for_message('/navsat/fix', NavSatFix)
            self.world.add_measurement(data.header.stamp.secs, (gps.latitude, gps.longitude), data.data)
    
    def goto_waypoint_result_callback(self, msg):
        if msg.data == 1:
            self.state = 0
        else:
            # TODO(alberto) recovery mechanism.
            rospy.logerr("goto failed.")
            self.state = 0
    
    def pick_locations_to_evaluate(self):
        current_cell = self.world.cell_corresponding_to_gps(self.current_gps[0], self.current_gps[1])
        x_values = range(max(0, current_cell[0] - self.plan_window[0] / 2), min(self.world.width, current_cell[0] + self.plan_window[0] / 2))
        y_values = range(max(0, current_cell[1] - self.plan_window[1] / 2), min(self.world.height, current_cell[1] + self.plan_window[1] / 2))
        candidate_locations = [x_values, y_values]
        return list(itertools.product(*candidate_locations))
    
    def spin(self):    
        rospy.init_node('adaptive_sampling', anonymous=True)  #initializing the node
        r = rospy.Rate(self.replan_rate)
        print "SPINNING"
        while not rospy.is_shutdown():
        
            if self.state == 0:
                # Update the current pose.
                print "Before request GPS"
                self.request_gps()
                
                msg = NavStatus()
                msg.t = rospy.get_rostime()
                msg.curr_lat = self.current_gps[0] 
                msg.curr_lon = self.current_gps[1] 
                try:
                    # Update the model.
                    self.world.update()
                    print "WORLD UPDATE"

                    l = self.pick_locations_to_evaluate()
                    print "SELECT LOCATIONS"
                    # According to the world, select a pose.
                    best_candidate_indices = self.evaluation.evaluate(self.current_gps, l, self.world)
                    print "EVALUATE LOCATIONS"
                    best_candidate = self.world.gps_corresponding_to_cell(best_candidate_indices[0],best_candidate_indices[1])
                    msg.status = 0
                except:
                    try:
                        l = self.pick_locations_to_evaluate()
                        best_candidate_index = random.randint(0, len(l))
                        best_candidate_index_x = l[best_candidate_index][0] 
                        best_candidate_index_y = l[best_candidate_index][1] 
                    except:
                        best_candidate_index_x = random.randint(0, self.world.width-1)
                        best_candidate_index_y = random.randint(0, self.world.height-1)
                    #best_candidate_index = random.randint(0, len(l))
                    #best_candidate = self.world.gps_corresponding_to_cell(l[best_candidate_index][0],l[best_candidate_index][1])
                    best_candidate = self.world.gps_corresponding_to_cell(best_candidate_index_x,best_candidate_index_y)
                    msg.status = 1
            
                msg.dest_lat = best_candidate.latitude
                msg.dest_lon = best_candidate.longitude
                print "GOTO", best_candidate
                # Go to.
                rospy.loginfo("Call goto service: lat {} lon {}".format(best_candidate.latitude, 
                  best_candidate.longitude))
                rospy.wait_for_service('goto')
                gotoWaypoint = rospy.ServiceProxy('goto', Goto)
                resp = gotoWaypoint(best_candidate.latitude, best_candidate.longitude) # lat, lon.
                if resp:
                    result = True
                else:
                    rospy.logerr("goto service failed.")
                    result = False
                if result:
                    self.state = 1
                self.time_last_exploration_plan_computed = rospy.get_rostime()
            elif self.state == 1:
                current_time = rospy.get_rostime()
                if (current_time.secs - self.time_last_exploration_plan_computed.secs) > self.time_threshold:
                    # TODO(alberto) ensure that controller stops.
                    self.state = 0
            r.sleep()    

        
def main():
    latitude, longitude = (33.44481, -118.48498)
    width, height, spacing, orientation = 72.2376, 35.6616, 2.0, -0.6084887621843595
    weights = [0.7, 0.3]
    scale_factors = [81.3816, 1.0]
    exploration = Exploration(latitude, longitude, width, height, spacing, 
        orientation, weights, scale_factors)
    exploration.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
