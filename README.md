# nav

trajectory_capture:
    two nodes
        one to capture the data from amr and store - will be run upon service call
        one to visualize the saved data

turtlebot_interfaces:
    Custom Service TrajectoryRequest - helps imporve modularity
        takes in filename and the duration in seconds
        returns success as a flag and a string message for error messages

        ros2 service call /trajectory_capture_service turtlebot_interfaces/srv/TrajectoryRequest "{filename: "test.yaml",  duration: 40.0}"

turtlebot_simulations
    Forked from ROBOTIS/turtlebot3
    used for naviagtion and SLAM of AMR