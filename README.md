This project is part of a ROS2 examination designed to assess the understanding of core ROS2 concepts through a TurtleBot3 simulation in Gazebo. The exam is divided into three parts — Publisher/Subscriber, Service, and Action — each requiring the creation and use of ROS2 nodes in Python or C++.

Part A: Implements a circle_publisher node to send velocity commands for circular motion and an odom_logger subscriber node to read and log robot odometry.

Part B: Introduces a square_service_server that makes the robot move in a square when triggered, along with a square_service_client to call the service.

Part C: Features a rotate_action_server to rotate the robot by a specified angle with periodic feedback, and a rotate_action_client to send goals and monitor progress.

All components are structured within a clean ROS2 package, demonstrating practical use of topics, services, and actions.
Objectives

To test understanding of ROS2 fundamental concepts such as Publisher, Subscriber, Interface, Service, and Action through the simulation software and implementation.
Instructions

    Use ROS2 Python or C++.
    Each task will be tested in the TurtleBot3 Gazebo simulation.
    Code must be properly organized in a ROS2 package with clear node names.

Section A: Publisher and Subscriber

    Task A1(Publisher): Create a node name "circle_publisher" that continuously publishes desire velocity commands geometry_msgs/msg/Twist to make Turtlebot3 move in a circle of radius ~0.5 meters.

    Task A2(Subscriber): Create a node name "odom_logger" that subscribes to /odom and prints:
        Robot's position(x, y)
        Robot's orientation(yaw angle)

Section B: Service

    Task B1(Service Server): Create a service server node named "square_service_server" with service type std_srvs/srv/Empty.
        When called, the robot should move in a square path with 0.5 meter per side using the velocity commands.
        After finishing, the robot should stop.

    Task B2(Service Client): Create a client node "square_service_client" to call your created service.

Section C: Action

    Task C1(Action Server): Create an action server node named "rotate_action_server" using a custom action definition Rotate.action:
    Image

    The server should:
        Rotate the Turtlebot3 in place by publishing /cmd_vel.
        Track the remaining angle of the robot for calculating the proper velocity through the simple P controller by the following concept:

    Image

        Publish feedback every 0.1 second (10 Hz).
        Stop and succeed when finished.

    Task C2(Action Client): Create a client node "rotate_action_client" that:
        Sends a goal angle (e.g., +3.14 radians or 180 degrees).
        Prints feedback (remaining angle).
        Prints the result when done: "Goal reached successfully" or "Goal aborted".
