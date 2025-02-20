Tasks
1. Publisher/Subscriber
- Creation of a ROS2 package using C++ or Python
- Creation of a ROS2 node that publishes the value of a sine wave on a topic at a given frequency
- The node parses the following parameters from a yaml file:
    - Publisher frequency
    - Amplitude
    - Angular frequency
    - Phase   

- Creation of a ROS2 node that subscribes to that topic and log the value
- Nodes should be configured using the generate parameter library: generate parameter library
- The node should  run with a python launch file
2.  Service
- Extend the previously created nodes by adding a custom service
- This service receives as input the file path of an image and returns the same image but in grayscale
3.  Extra: Dockerfile
- The candidate provides a Dockerfile that allows to build and run the code from within the docker
- The instruction on how to use it are written in the README

Evaluation Criteria
- Repo is accessible and clonable with no issues
- The repo can be built using the standard ROS2 official procedure (e.g., rosdep install, colcon build, etc.)
- The README is clear and reports all the instructions required to run
- The code is written following the ROS2 documentation and standard practices
- Error handling
- Unit tests are written (using gtest or pytest), a minimal example is required, no 100% coverage.
- The README provides instructions on how to use the publisher and/or services (e.g., how to visualize the sine wave and the image)
- Coding quality (hint: use formatter and linter)
- The complexity of the chosen programming language will be taken into account
- (Extra): The code is written in both C++ and Python
- (Extra): GitHub actions are building the repo
- (Extra): GitHub actions apply linting and formatting rules
- (Extra): Good git commit strategy