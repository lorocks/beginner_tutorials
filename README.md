# ROS2 C++ Beginner Tutorials
This is a repository of beginner tutorials for ROS2 in C++.
To use the repository, clone the repository and follow steps in each release part.

The repository uses ROS Humble 
<br>

The repository contains:
 - Publisher/Subscriber Example
 - Services_Logging_Launch Example
 - tf2_unitTesting_rosBagFiles Example

## Initial Steps
```bash
# Clone the github repository
# Using http://
  git clone https://github.com/lorocks/beginner_tutorials.git
# Using SSH
  git clone git@github.com:lorocks/beginner_tutorials.git

# Change to workspace directory
  cd beginner_tutorials/  
# Build the ROS2 project sequentially
  colcon build --packages-select cpp_service
# Source the underlay to reduce error in compilation
  source install/setup.sh
# Rebuild the entire project
  colcon build
# Source the underlay
  source install/setup.sh
```

# Part1_Release: Publisher/Subscriber
This part contains two nodes, publisher (talker) and subscriber (listener) written in C++ inside the cpp_pubsub package.
<br>


Run each node in separate terminals
## Run the Publisher Node
```bash
# To run the publisher node
  ros2 run cpp_pubsub talker
```
## Run the Subscriber Node
```bash
# To run the subscriber node
  ros2 run cpp_pubsub talker
```
## cpplint Errors
<chrono> is an unaprroved header which provides clock literals which is needed 

# Part2_Release: Services_Logging_Launch
This part contains edited publisher node with a service to change the count value in the published message.
<br>
A launch file designed to run nodes, talker and listener, and pass parameter to talker if necessary.
<br>
A new package is created, cpp_service, to be used as service descriptions.

## Call the Service
```bash
# Call the service from the console
  ros2 service call /change_counter cpp_service/srv/ChangeCounter '{number: <add number here>}'
```
If the service call throws an error, source the underlay using
```bash
# Source the underlay
  source install/setup.sh
```

## Run using Launch File
Launch parameters
 - ```pub_frequency``` frequency of publishing messages by talker node, default value = 750
```bash
# ros2 launch command
  ros2 launch cpp_pubsub cpp_launch.py 'pub_frequency:=< enter frequency value>'
```
## cpplint Errors
<chrono> is an unaprroved header which provides clock literals which is needed 


# Part3_Release: tf2_unitTests_bagFiles
This part contains a tf2 implementation where static frames are being published. 
Unit tests are written to perform Level 2 unit tests and a ros2 bag record method has been implemented

## Verify tf2 Results
```bash
# Print the frames to the console
  ros2 run tf2_ros tf2_echo world talk
# tf2 view frame tool
  ros2 run tf2_tools view_frames
```

## Unit Tests
```bash
# Run the test
  colcon test
# View results
  nano log/latest_test/cpp_pubsub/stdout_stderr.log 
```

## ROS2 Bag
```bash
# Launch bag
  ros2 launch cpp_pubsub cpp_launch_bag.py
# Launch file but disable ros2 bag
  ros2 launch cpp_pubsub cpp_launch_bag.py ros_bag:=False
# Check information
  ros2 bag info ./results/ros2_bag

# Testing the ros2 bag
# Start listener node
  ros2 run cpp_pubsub listener
# Replay bag with talker node
  ros2 bag play ./results/ros2_bag
```

## cpplint Errors
<chrono> is an unaprroved header which provides clock literals which is needed 


# Generate Doxygen Documentation
```bash
# Download doxygen
  sudo apt-get install doxygen
# Rebuild documentation
  doxygen dconfig
```

# Results
There are three folder inside the /results directory and they contain
<br>
 - cpp_pubsub: cppcheck and cpplint results
 - launch: image of successful launching of nodes talker and listener
 - rqt_console: image of rqt_console with logger levels shown
 - ros2_bag: published messages recorded using ros2 bag
 - tf2frames: output from running view_frame in tf2_tools
