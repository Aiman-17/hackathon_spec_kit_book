---
title: "ROS 2 in Depth: Nodes, Topics, Services, Actions"
sidebar_position: 7
description: "Explore the core ROS 2 communication concepts of nodes, topics, services, and actions in depth."
tags: [ros, 2, in, depth, nodes, topics, services, actions]
---

## Summary

This chapter provides a comprehensive exploration of the core communication concepts in ROS 2: nodes, topics, services, and actions. You will learn how to design and implement robust ROS 2 systems by understanding the role and implementation of each of these key building blocks. Through practical coding examples and system architecture diagrams, you will gain the knowledge to create scalable, modular, and fault-tolerant robotic applications using the powerful ROS 2 framework.

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain the purpose and functionality of ROS 2 nodes, topics, services, and actions
- Implement a ROS 2 publisher node that sends sensor data at 10 Hz
- Analyze the communication flow in a ROS 2 system with multiple nodes, topics, and services
- Evaluate the trade-offs between using topics, services, and actions for different robotic use cases
- Create a ROS 2 action server and client to execute complex, long-running tasks

## Prerequisites

- Basic understanding of ROS 2 concepts and setup (covered in previous chapters)
- Familiarity with Python programming and object-oriented design

## ROS 2 Nodes: The Building Blocks

ROS 2 nodes are the fundamental building blocks of a robotic system. A node is a standalone process that performs a specific function, such as reading sensor data, processing information, or controlling a robot's actuators. Nodes communicate with each other using various ROS 2 communication primitives, which we'll explore in the following sections.

### Defining and Launching Nodes

To create a ROS 2 node, you'll need to define a Python script that inherits from the `rclpy.Node` class. This class provides the necessary methods and attributes to manage the node's lifecycle, handle callbacks, and interact with other ROS 2 entities.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Node initialization code here

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

:::tip
Use meaningful names for your nodes to help with debugging and maintainability. The name should describe the node's primary function.
:::

To launch a ROS 2 node, you can use the `ros2 run` command in your terminal:

```bash
ros2 run my_package my_node
```

### Node Lifecycle and Callbacks

ROS 2 nodes have a well-defined lifecycle, with the following key stages:

1. **Initialization**: The node is created and any necessary setup is performed.
2. **Spinning**: The node enters the event loop, where it can handle callbacks and process data.
3. **Shutdown**: The node is gracefully terminated and any necessary cleanup is executed.

Nodes can define custom callbacks to handle various events, such as:

- **Timer callbacks**: Executed at a specified frequency, useful for periodic tasks.
- **Subscription callbacks**: Triggered when a message is received on a topic.
- **Service callbacks**: Invoked when a client requests a service.
- **Action callbacks**: Used to handle the various stages of an action execution.

By leveraging these callbacks, you can create complex, event-driven robotic systems in ROS 2.

## ROS 2 Topics: Publish-Subscribe Communication

ROS 2 topics are the primary means of asynchronous, one-to-many communication between nodes. A node can publish messages to a topic, and other nodes can subscribe to that topic to receive the data.

### Defining Topics and Messages

To use a topic, you'll need to define a message type that describes the data being transmitted. ROS 2 provides a wide range of built-in message types, and you can also create custom message types using the `.msg` file format.

```python
from geometry_msgs.msg import Twist

class TurtleBot(Node):
    def __init__(self):
        super().__init__('turtlebot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.1
        self.publisher_.publish(msg)
```

In this example, the `TurtleBot` node publishes `Twist` messages to the `cmd_vel` topic at a rate of 10 Hz.

### Subscribing to Topics

To receive data from a topic, you'll need to create a subscriber node and define a callback function to handle the incoming messages.

```python
from sensor_msgs.msg import LaserScan

class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            10)

    def laser_scan_callback(self, msg):
        print(f'Received laser scan data: {msg}')
```

This `LaserScanSubscriber` node subscribes to the `scan` topic and prints the received `LaserScan` messages.

### Topic Quality of Service (QoS)

ROS 2 topics support various Quality of Service (QoS) policies that allow you to fine-tune the communication behavior based on your application's needs. These policies include:

- **Reliability**: Determines whether messages should be delivered reliably or with best-effort.
- **Durability**: Specifies whether messages should be stored for late-joining subscribers.
- **History**: Controls how many messages should be stored for late-joining subscribers.

You can configure the QoS policies when creating publishers and subscribers.

```python
self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10, qos_profile=QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
))
```

By understanding and leveraging QoS policies, you can create more robust and scalable ROS 2 systems.

## ROS 2 Services: Request-Response Communication

ROS 2 services provide a synchronous, request-response communication mechanism between nodes. A node can offer a service, and other nodes can request that service by sending a request message and waiting for a response.

### Defining Services and Messages

Services are defined using `.srv` files, which specify the request and response message types.

```
# my_srv/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

In your Python code, you can create a service server and client using the generated message types.

```python
from my_srv.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        return response

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        self.request = AddTwoInts.Request()

    def send_request(self):
        self.request.a = 2
        self.request.b = 3
        self.future = self.cli.call_async(self.request)
```

In this example, the `AddTwoIntsServer` node offers the `add_two_ints` service, while the `AddTwoIntsClient` node sends a request to the server and waits for the response.

## ROS 2 Actions: Complex, Long-Running Tasks

ROS 2 actions provide a higher-level communication mechanism for executing complex, long-running tasks. Actions are similar to services, but they support additional functionality, such as cancellation, progress feedback, and result reporting.

### Defining Actions and Messages

Actions are defined using `.action` files, which specify the goal, result, and feedback message types.

```
# my_action/action/MoveRobot.action
# Goal
geometry_msgs/Pose target_pose
---
# Result
bool success
---
# Feedback
float32 distance_remaining
```

In your Python code, you can create an action server and client using the generated message types.

```python
from my_action.action import MoveRobot

class MoveRobotActionServer(Node):
    def __init__(self):
        super().__init__('move_robot_action_server')
        self.action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        # Implement the long-running task here
        feedback = MoveRobot.Feedback()
        feedback.distance_remaining = 1.0
        goal_handle.publish_feedback(feedback)

        result = MoveRobot.Result()
        result.success = True
        return result

class MoveRobotActionClient(Node):
    def __init__(self):
        super().__init__('move_robot_action_client')
        self.action_client = ActionClient(self, MoveRobot, 'move_robot')

    def send_goal(self):
        goal_msg = MoveRobot.Goal()
        goal_msg.target_pose.position.x = 2.0
        goal_msg.target_pose.position.y = 3.0
        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        print(f'Feedback received: {feedback_msg.feedback.distance_remaining}')
```

In this example, the `MoveRobotActionServer` node implements a long-running "move robot" task, while the `MoveRobotActionClient` node sends a goal to the server and receives progress feedback.

## Key Takeaways

- ROS 2 nodes are the fundamental building blocks of a robotic system, responsible for specific functionalities.
- ROS 2 topics enable asynchronous, one-to-many communication using the publish-subscribe pattern.
- ROS 2 services provide synchronous, request-response communication between nodes.
- ROS 2 actions support complex, long-running tasks with additional features like cancellation and progress feedback.
- Understanding the strengths and trade-offs of each ROS 2 communication primitive is crucial for designing robust and scalable robotic applications.

## Glossary

- **Node**: A standalone process in a ROS 2 system that performs a specific function.
- **Topic**: A named bus over which nodes exchange messages using the publish-subscribe pattern.
- **Service**: A synchronous request-response communication mechanism between nodes.
- **Action**: A higher-level communication mechanism for executing complex, long-running tasks.
- **Quality of Service (QoS)**: Policies that control the behavior of ROS 2 communication primitives.
- **Callback**: A function that is executed in response to a specific event or message.

## Review Questions

1. Explain the purpose and functionality of ROS 2 nodes, topics, services, and actions.
2. Implement a ROS 2 publisher node that sends sensor data at 10 Hz. Describe the key steps involved.
3. Analyze the communication flow in a ROS 2 system with multiple nodes, topics, and services. Identify the advantages and disadvantages of each communication primitive.
4. Evaluate the trade-offs between using topics, services, and actions for different robotic use cases. Provide specific examples.
5. Create a ROS 2 action server and client to execute a complex, long-running task. Explain how the action mechanism differs from using a service.