# Glossary of ROS 2 Terms

## A

**Action**: A goal-oriented communication pattern that includes feedback during execution and result reporting upon completion. Ideal for long-running tasks where the client needs to know the progress and final outcome.

**Architecture**: The fundamental structure of ROS 2, built on the Data Distribution Service (DDS) standard, which provides a publish-subscribe communication model.

## B

**Build System**: The system used to compile and build ROS 2 packages, typically using CMake and colcon for dependency management.

## C

**Client Library**: Libraries that allow nodes to be written in different programming languages (e.g., rclpy for Python, rclcpp for C++).

**Communication Primitives**: The different ways nodes can exchange information in ROS 2: topics, services, and actions.

**Configuration**: The process of setting up nodes, parameters, and system behavior for specific use cases.

## D

**DDS (Data Distribution Service)**: The middleware standard that ROS 2 uses for communication between nodes. It provides a standardized way for nodes to communicate with each other, regardless of the programming language or operating system.

**Dependency Management**: The system for managing and resolving dependencies between different ROS 2 packages and libraries.

## E

**Executor**: Manages the execution of callbacks from multiple nodes. Determines how and when the callbacks are executed, allowing for different threading models and execution strategies.

## H

**Hardware Abstraction**: The layer that provides a standard interface between hardware and software, hiding the complexity of specific hardware implementations.

## J

**Joint**: In URDF, defines how links connect and move relative to each other. Common types include revolute, continuous, prismatic, and fixed joints.

## L

**Lifecycle**: The well-defined states that ROS 2 nodes go through: Unconfigured, Inactive, Active, and Finalized.

**Link**: In URDF, represents the rigid parts of a robot (e.g., torso, arms, legs).

## M

**Middleware**: Software that provides common services and capabilities to applications beyond what's offered by the operating system. In ROS 2, DDS serves as the middleware.

**Message**: The data structures passed between nodes in ROS 2 communication.

**Middleware**: Software that connects different applications or systems. In ROS 2, DDS acts as the middleware for communication.

## N

**Node**: The fundamental building block of a ROS 2 system. Each node is a process that performs a specific task and communicates with other nodes through messages.

## P

**Publisher**: A node that sends messages to a topic in the publisher-subscriber communication pattern.

**Package**: A collection of related ROS 2 functionality that is distributed as a single entity.

## Q

**QoS (Quality of Service)**: Settings that allow fine-tuning of communication behavior for real-time requirements, including reliability, durability, history, deadline, and lifespan.

## S

**Service**: A request-response communication pattern in ROS 2 where a client sends a request to a service server, which processes the request and returns a response.

**Subscriber**: A node that receives messages from a topic in the publisher-subscriber communication pattern.

## T

**Topic**: Named buses over which nodes exchange messages using a publisher-subscriber paradigm where publishers send messages to a topic and subscribers receive messages from a topic.

## U

**URDF (Unified Robot Description Format)**: An XML-based format used in ROS to describe robot models, defining the physical and visual properties of a robot including its links, joints, and other components.

## V

**Version**: The specific release of ROS 2 (e.g., Humble Hawksbill, Rolling Ridley).

## W

**Workspace**: A directory containing multiple ROS 2 packages that are built together.

## X

**XACRO**: XML Macros, a macro language that allows parameterization and reuse of URDF elements, making complex models more manageable.