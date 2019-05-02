# README #

README for strategy for sending robot to explore a phenomenon, based on variance.

TODO: Finish the README.

### What is this repository for? ###

* Quick summary
ROS node that given a robot deployed in an environment, collecting some data, sends GPS waypoints to the robot so that the robot explore the phenomenon.

### How do I get set up? ###

* Summary of set up
* Configuration
* Dependencies
* Database configuration
* How to run tests
Example of running in the Gazebo simulator with a simulated kingfisher.
```
roslaunch kingfisher_gazebo base_gazebo_2.launch
roslaunch nk_kingfisher core.launch
rosrun adaptive_sampling helm2drive.py
rosrun adaptive_sampling exploration.py
roslaunch simulated_sensor chlorophyll_publisher_nighthorse.launch
```

* Deployment instructions

### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact