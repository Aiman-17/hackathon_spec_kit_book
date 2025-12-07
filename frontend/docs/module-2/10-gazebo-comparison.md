---
title: Gazebo Classic vs. Gazebo Garden (Ignition)
slug: gazebo-comparison
sidebar_position: 10
description: "Explore the differences between Gazebo Classic and Gazebo Garden (Ignition), two popular robotics simulation environments."
tags: [gazebo, classic, vs., gazebo, garden]
---

## Summary

This chapter provides a comprehensive comparison of Gazebo Classic and Gazebo Garden (Ignition), two prominent robotics simulation environments. It delves into the key architectural differences, feature sets, and use cases of these platforms, enabling readers to make informed decisions when selecting the appropriate simulation tool for their robotics projects. Through practical code examples and system diagrams, the chapter equips readers with the knowledge to effectively leverage these simulation environments in their development workflows.

## Learning Objectives

By the end of this chapter, readers will be able to:

- Explain the core architectural differences between Gazebo Classic and Gazebo Garden (Ignition)
- Implement a ROS 2 node that interacts with both Gazebo Classic and Gazebo Garden (Ignition) simulations
- Analyze the performance and resource utilization characteristics of Gazebo Classic and Gazebo Garden (Ignition)
- Evaluate the suitability of Gazebo Classic and Gazebo Garden (Ignition) for different robotics use cases

## Prerequisites

- Familiarity with ROS 2 and its core concepts
- Basic understanding of robotics simulation and 3D rendering
- Proficiency in Python programming

## Gazebo Classic vs. Gazebo Garden (Ignition)

### Architectural Differences

Gazebo Classic and Gazebo Garden (Ignition) are both popular robotics simulation environments, but they differ in their underlying architectures. Gazebo Classic is built on the Open Dynamics Engine (ODE) for physics simulation, while Gazebo Garden (Ignition) utilizes the Bullet Physics Engine.

:::note
Gazebo Garden (Ignition) is the successor to Gazebo Classic, providing a more modular and extensible architecture.
:::

#### Gazebo Classic Architecture

The Gazebo Classic architecture is centered around the following key components:

1. **ODE**: The Open Dynamics Engine (ODE) is responsible for the physics simulation, handling collisions, joint dynamics, and other physical interactions.
2. **OGRE**: The Object-Oriented Graphics Rendering Engine (OGRE) is used for the 3D rendering of the simulation environment.
3. **ROS Integration**: Gazebo Classic provides tight integration with the Robot Operating System (ROS), allowing seamless communication between the simulation and ROS nodes.

#### Gazebo Garden (Ignition) Architecture

The Gazebo Garden (Ignition) architecture is more modular and extensible, with the following key components:

1. **Bullet Physics Engine**: Gazebo Garden (Ignition) utilizes the Bullet Physics Engine for improved physics simulation performance and accuracy.
2. **Rendering Engines**: Gazebo Garden (Ignition) supports multiple rendering engines, including Ogre 2, Ignition Rendering, and others, providing more flexibility.
3. **Ignition Transport**: Ignition Transport is a new communication middleware that replaces the ROS integration, offering a more lightweight and efficient messaging system.

### Feature Comparison

Gazebo Classic and Gazebo Garden (Ignition) offer a range of features for robotics simulation, with some key differences:

1. **Physics Simulation**: Gazebo Garden (Ignition) generally provides more accurate and performant physics simulation compared to Gazebo Classic, thanks to the Bullet Physics Engine.
2. **Rendering**: Gazebo Garden (Ignition) offers more rendering engine options, allowing for better visual fidelity and customization.
3. **ROS Integration**: Gazebo Classic has a more mature and well-established ROS integration, while Gazebo Garden (Ignition) uses the Ignition Transport middleware.
4. **Extensibility**: Gazebo Garden (Ignition) is designed to be more modular and extensible, with a plugin-based architecture that allows for easier integration of custom components.

### Code Examples

Let's explore how to interact with both Gazebo Classic and Gazebo Garden (Ignition) using ROS 2 Python code:

```python
# Gazebo Classic ROS 2 Node
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

class GazeboClassicNode(Node):
    def __init__(self):
        super().__init__('gazebo_classic_node')
        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')

    def spawn_robot(self, robot_description, robot_name, robot_pose):
        request = SpawnEntity.Request()
        request.name = robot_name
        request.xml = robot_description
        request.initial_pose = robot_pose
        self.spawn_entity_client.call_async(request)

    def delete_robot(self, robot_name):
        request = DeleteEntity.Request()
        request.name = robot_name
        self.delete_entity_client.call_async(request)

# Gazebo Garden (Ignition) ROS 2 Node
import rclpy
from rclpy.node import Node
from ignition_msgs.srv import SpawnEntity, DeleteEntity

class GazeboGardenNode(Node):
    def __init__(self):
        super().__init__('gazebo_garden_node')
        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')

    def spawn_robot(self, robot_description, robot_name, robot_pose):
        request = SpawnEntity.Request()
        request.name = robot_name
        request.sdf = robot_description
        request.pose = robot_pose
        self.spawn_entity_client.call_async(request)

    def delete_robot(self, robot_name):
        request = DeleteEntity.Request()
        request.name = robot_name
        self.delete_entity_client.call_async(request)

The code above demonstrates how to interact with Gazebo Classic and Gazebo Garden (Ignition) using ROS 2 Python nodes. The key differences are the service types used (`gazebo_msgs.srv` vs. `ignition_msgs.srv`) and the parameter names (`xml` vs. `sdf`) for the `SpawnEntity` service.

```

### System Architecture Comparison

To further illustrate the differences between Gazebo Classic and Gazebo Garden (Ignition), let's compare their system architectures using Mermaid diagrams:

```mermaid
graph LR
    subgraph Gazebo Classic
        ROS2Node -- ROS Topics --> GazeboClassic
        GazeboClassic -- ODE --> Physics
        GazeboClassic -- OGRE --> Rendering
    end
    subgraph Gazebo Garden (Ignition)
        ROS2Node -- Ignition Transport --> GazeboGarden
        GazeboGarden -- Bullet --> Physics
        GazeboGarden -- Ignition Rendering --> Rendering
    end
```

The key differences in the system architectures are the use of ODE vs. Bullet for physics simulation, OGRE vs. Ignition Rendering for 3D rendering, and the communication middleware (ROS Topics vs. Ignition Transport).

## Key Takeaways

- Gazebo Classic is built on the ODE physics engine and OGRE rendering, while Gazebo Garden (Ignition) uses the Bullet physics engine and supports multiple rendering engines.
- Gazebo Garden (Ignition) offers a more modular and extensible architecture, with a plugin-based design and improved physics simulation performance.
- Gazebo Classic has a more mature and well-established ROS integration, while Gazebo Garden (Ignition) uses the Ignition Transport middleware.
- The choice between Gazebo Classic and Gazebo Garden (Ignition) depends on the specific requirements of the robotics project, such as physics simulation accuracy, rendering needs, and ROS integration requirements.

## Glossary

1. **Gazebo Classic**: The original version of the Gazebo robotics simulation environment, built on the ODE physics engine and OGRE rendering.
2. **Gazebo Garden (Ignition)**: The successor to Gazebo Classic, featuring a more modular and extensible architecture with the Bullet physics engine and support for multiple rendering engines.
3. **ODE (Open Dynamics Engine)**: A physics engine used in Gazebo Classic for simulating collisions, joint dynamics, and other physical interactions.
4. **Bullet Physics Engine**: The physics engine used in Gazebo Garden (Ignition) for improved physics simulation performance and accuracy.
5. **OGRE (Object-Oriented Graphics Rendering Engine)**: The 3D rendering engine used in Gazebo Classic.
6. **Ignition Rendering**: The rendering engine used in Gazebo Garden (Ignition), providing more flexibility and customization options.
7. **Ignition Transport**: The communication middleware used in Gazebo Garden (Ignition) to replace the ROS integration in Gazebo Classic.

## Review Questions

1. Explain the key architectural differences between Gazebo Classic and Gazebo Garden (Ignition).
2. Implement a ROS 2 node that can interact with both Gazebo Classic and Gazebo Garden (Ignition) simulations, demonstrating the differences in the service types and parameters.
3. Analyze the performance and resource utilization characteristics of Gazebo Classic and Gazebo Garden (Ignition) for a complex robotics simulation scenario.
4. Evaluate the suitability of Gazebo Classic and Gazebo Garden (Ignition) for different robotics use cases, such as industrial automation, legged locomotion, and aerial vehicle simulation.