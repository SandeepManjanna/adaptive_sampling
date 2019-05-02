# It needs roscore at the beginning

from datetime import datetime
import os
import subprocess
import sys
import time
import traceback

import rospy

from simulated_sensor.msg import Measurement

limit = 8500
for i in xrange(5):
    with open('run_exp.log', 'a') as log_file:
        log_file.write(datetime.now().strftime('%Y-%m-%d-%H:%M:%S') + " trying to run {}\n".format(i))
    
    proper_run = False
    while not proper_run:
        gazebo = subprocess.Popen(["roslaunch", "kingfisher_gazebo",
            "base_gazebo_2.launch"])
        rospy.init_node('run_batch')
        time.sleep(30)
        
        for j in xrange(5):
            try:
                traceback.print_exc(file=sys.stdout)
                sensor_msg = rospy.wait_for_message('/kf1/chlorophyll_reading', Measurement, timeout=5)
                proper_run = True
            except rospy.ROSException, e:
                traceback.print_exc(file=sys.stdout)
                proper_run = False
                gazebo.terminate()
                time.sleep(60)
                with open('run_exp.log', 'a') as log_file:
                    log_file.write(datetime.now().strftime('%Y-%m-%d-%H:%M:%S') + " jumped!\n")
                break


    with open('run_exp.log', 'a') as log_file:
        log_file.write(datetime.now().strftime('%Y-%m-%d-%H:%M:%S') + " running {}\n".format(i))
    my_env = os.environ.copy()
    my_env["ROS_NAMESPACE"] = "kf1"
    exploration = subprocess.Popen(["rosrun", 
        "adaptive_sampling", "exploration.py"], env=my_env)
    time.sleep(limit)
    exploration.terminate()
    gazebo.terminate()
    time.sleep(60)
    os.system('pkill -f -9 ros')
    with open('run_exp.log', 'a') as log_file:
        log_file.write(datetime.now().strftime('%Y-%m-%d-%H:%M:%S') + " terminated {}\n".format(i))
