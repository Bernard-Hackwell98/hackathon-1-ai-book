# Assessment Methods for ROS 2 for Humanoid Robotics Module

## Chapter 1: Introduction to ROS 2 for Humanoid Robotics

### Quiz Questions

1. What does ROS 2 stand for?
   - a) Robot Operating System 2
   - b) Robot Operation System 2
   - c) Robotic Operating System 2
   - d) Robot Operating Service 2

2. Which middleware standard does ROS 2 use for communication?
   - a) MQTT
   - b) DDS (Data Distribution Service)
   - c) AMQP
   - d) HTTP

3. What is the main advantage of ROS 2 over ROS 1 for real-time humanoid systems?
   - a) Better graphics support
   - b) Improved real-time performance and security
   - c) More programming languages supported
   - d) Simpler installation process

### Practical Demonstrations

1. Research and list three different DDS implementations that can be used with ROS 2.
2. Explain the difference between a topic and a service in ROS 2.
3. Why is real-time performance important for humanoid robots?

## Chapter 2: ROS 2 Communication Primitives

### Quiz Questions

1. What is the main difference between a publisher and a subscriber in ROS 2?
   - a) Publishers send data, subscribers receive data
   - b) Publishers receive data, subscribers send data
   - c) Publishers and subscribers both send data
   - d) Publishers and subscribers both receive data

2. Which type of communication pattern is used for request-response communication in ROS 2?
   - a) Topics
   - b) Services
   - c) Actions
   - d) Messages

3. What is the purpose of Quality of Service (QoS) settings in ROS 2?
   - a) To improve graphics rendering
   - b) To configure communication behavior for different requirements
   - c) To manage user authentication
   - d) To optimize storage usage

### Practical Demonstrations

1. Create a publisher node that publishes sensor data (e.g., temperature readings) and a subscriber that processes this data.
2. Implement a service that performs a calculation based on two input parameters.
3. Design an action server that simulates a long-running task with feedback.
4. Modify the AI decision example to include obstacle detection and path planning.

## Chapter 3: Robot Modeling with URDF

### Quiz Questions

1. What does URDF stand for?
   - a) Unified Robot Description Format
   - b) Universal Robot Design Format
   - c) Unified Robotic Development Framework
   - d) Universal Robot Description File

2. Which joint type allows rotational movement with a range of motion?
   - a) Fixed
   - b) Continuous
   - c) Revolute
   - d) Prismatic

3. What is the purpose of the `<inertial>` tag in URDF?
   - a) To define visual appearance
   - b) To specify collision properties
   - c) To define mass properties for simulation
   - d) To set joint limits

### Practical Demonstrations

1. Modify the simplified humanoid model to add fingers to the hands.
2. Create a URDF model for a different type of robot (e.g., wheeled robot).
3. Research and explain how XACRO can simplify complex URDF models.
4. Investigate how URDF models are used with the ROS 2 robot state publisher.

## Comprehensive Assessment

### Final Project

Students should complete a comprehensive project that integrates concepts from all three chapters:

1. Create a URDF model of a simple humanoid robot
2. Implement ROS 2 nodes that control the robot's movement
3. Use appropriate communication primitives (topics, services, or actions) to coordinate between different components
4. Document the design decisions and explain how the system meets real-time requirements
5. Demonstrate the system in simulation (using Gazebo or a similar simulator)

### Rubric

- **Technical Understanding (40%)**: Demonstrates clear understanding of ROS 2 concepts, architecture, and communication primitives
- **Implementation Quality (30%)**: Code is well-structured, follows best practices, and functions correctly
- **Documentation (20%)**: Clear explanations, proper use of comments, and well-structured documentation
- **Creativity and Innovation (10%)**: Creative approach to problem-solving and innovative use of ROS 2 features