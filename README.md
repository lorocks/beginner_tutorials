# ROS2 C++ Beginner Tutorials
This is a repository of beginner tutorials for ROS2 in C++.
To use the repository, clone the repository and follow steps in each release part.

The repository uses ROS Humble 
<br>

The repository contains:
 - Publisher/Subscriber Example
 - Services_Logging_Launch Example

## Initial Steps
```bash
# Build the ROS2 project
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
 - ```pub_frequency``` frequency of publishing messages by talker node, deafult value = 750
```bash
# ros2 launch command
  ros2 launch cpp_pubsub cpp_launch.py 'pub_frequency:=< enter frequency value>'
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