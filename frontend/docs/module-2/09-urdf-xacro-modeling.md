---
title: "URDF & XACRO for Robot Modeling"
sidebar_position: 9
description: Explore the Unified Robot Description Format (URDF) and XACRO for creating detailed robot models in simulation environments.
tags: [urdf, xacro, robot, modeling, simulation]
---

## Summary

This chapter delves into the Unified Robot Description Format (URDF) and XACRO, two powerful tools for creating detailed robot models in simulation environments. You will learn how to use URDF to define the physical and visual properties of a robot, and how XACRO can be used to simplify and modularize the URDF description. Through practical examples and code snippets, you will gain the knowledge to build your own robot models and integrate them into simulation frameworks like Gazebo or RViz.

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain the purpose and structure of URDF and XACRO files for robot modeling
- Implement a complete URDF description for a simple robotic arm, including links, joints, and visual/collision properties
- Analyze the advantages of using XACRO to modularize and parameterize URDF descriptions
- Evaluate the process of integrating a URDF-based robot model into a ROS 2 simulation environment
- Create a custom URDF/XACRO model for a new robot design and integrate it into a ROS 2 simulation

## Prerequisites

- Familiarity with ROS 2 and its core concepts (nodes, topics, services, etc.)
- Understanding of basic 3D modeling and coordinate systems
- Proficiency in Python programming

## Main Content

### The Unified Robot Description Format (URDF)

The Unified Robot Description Format (URDF) is an XML-based language used to describe the physical and visual properties of a robot. URDF files provide a standardized way to represent the robot's links, joints, sensors, and other components, allowing for seamless integration with simulation environments and control frameworks.

#### URDF Structure

A URDF file is composed of several key elements:

- `<robot>`: The root element that contains the entire robot description.
- `<link>`: Defines a physical component of the robot, such as a body part or a sensor.
- `<joint>`: Describes the connection between two links, including the type of joint (e.g., revolute, prismatic) and its properties.
- `<visual>`: Specifies the visual representation of a link, including its geometry, color, and material.
- `<collision>`: Defines the collision geometry of a link, used for physics-based simulations.

:::tip
URDF files can be easily visualized using tools like RViz, allowing you to inspect the robot's structure and joints.
:::

#### Practical Example: A Simple Robotic Arm

Let's consider a simple robotic arm with three links and two revolute joints. Here's an example of the URDF description for this robot:

```xml
<?xml version="1.0"?>
<robot name="robotic_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="shoulder_link">
    <!-- Link properties -->
  </link>

  <link name="elbow_link">
    <!-- Link properties -->
  </link>

  <joint name="shoulder_joint" type="revolute">
    <!-- Joint properties -->
  </joint>

  <joint name="elbow_joint" type="revolute">
    <!-- Joint properties -->
  </joint>
</robot>

In this example, we define the `base_link` with its visual and collision properties, as well as the `shoulder_link` and `elbow_link`. The `shoulder_joint` and `elbow_joint` are then specified as revolute joints, connecting the links.

### XACRO: Extending URDF with Parameterization

While URDF provides a robust way to describe robot models, the XML-based syntax can become cumbersome for complex robots. This is where XACRO (XML Macros) comes into play. XACRO is a preprocessor that allows you to create modular, parameterized URDF descriptions, making them easier to maintain and extend.

#### XACRO Advantages

- **Parameterization**: XACRO enables you to define variables and macros that can be reused throughout the robot description.
- **Modularity**: XACRO files can be split into smaller, reusable components, improving code organization and readability.
- **Conditional Statements**: XACRO supports if-else statements and other programming constructs, allowing for more dynamic robot descriptions.

#### Practical Example: XACRO-based Robotic Arm

Let's revisit the robotic arm example and see how we can use XACRO to make the URDF more modular and parameterized:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robotic_arm">
  <xacro:property name="base_length" value="0.2"/>
  <xacro:property name="base_width" value="0.2"/>
  <xacro:property name="base_height" value="0.1"/>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
  </link>

  <link name="shoulder_link">
    <!-- Link properties -->
  </link>

  <link name="elbow_link">
    <!-- Link properties -->
  </link>

  <joint name="shoulder_joint" type="revolute">
    <!-- Joint properties -->
  </joint>

  <joint name="elbow_joint" type="revolute">
    <!-- Joint properties -->
  </joint>
</robot>
```

In this XACRO-based example, we define three properties: `base_length`, `base_width`, and `base_height`. These properties are then used within the `<geometry>` elements of the `<visual>` and `<collision>` tags, allowing us to easily adjust the dimensions of the base link.

:::note
XACRO files are preprocessed by the `xacro` command-line tool before being used in a ROS 2 simulation or application.
:::

### Integrating URDF/XACRO into ROS 2 Simulations

To use a URDF or XACRO-based robot model in a ROS 2 simulation, you'll need to follow these general steps:

1. **Create the URDF/XACRO file**: Define the robot's links, joints, and other properties as shown in the previous examples.
2. **Load the model into ROS 2**: Use the `robot_state_publisher` node to publish the robot's joint states, and the `gazebo_ros_control` plugin to handle the robot's physics and control.
3. **Visualize the robot**: Use the `rviz_visual_tools` package to display the robot model in RViz, allowing you to inspect and interact with the simulated robot.

Here's a simple example of how to load a URDF-based robot model in a ROS 2 Python script:

```python
import os
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF

class RobotModelLoader(Node):
    def __init__(self):
        super().__init__('robot_model_loader')
        urdf_path = os.path.join(os.path.dirname(__file__), 'robotic_arm.urdf')
        self.robot = URDF.from_xml_file(urdf_path)
        self.get_logger().info(f'Loaded robot model: {self.robot.name}')

def main(args=None):
    rclpy.init(args=args)
    robot_model_loader = RobotModelLoader()
    rclpy.spin(robot_model_loader)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

In this example, we load the URDF file, create a `RobotModelLoader` node, and print the name of the loaded robot model. You can then extend this script to publish the robot's joint states, integrate it with a simulation framework like Gazebo, and visualize the robot in RViz.

## Key Takeaways

- URDF is an XML-based language used to describe the physical and visual properties of robots, enabling seamless integration with simulation environments.
- XACRO provides a way to make URDF descriptions more modular, parameterized, and easier to maintain by introducing variables, macros, and conditional statements.
- Integrating URDF/XACRO-based robot models into ROS 2 simulations involves loading the model, publishing joint states, and visualizing the robot in tools like RViz.
- URDF and XACRO are essential tools for building and simulating complex robotic systems in a ROS 2 environment.

## Glossary

- **URDF (Unified Robot Description Format)**: An XML-based language used to describe the physical and visual properties of a robot, including its links, joints, and other components.
- **XACRO (XML Macros)**: A preprocessor that allows for the creation of modular, parameterized URDF descriptions, making them easier to maintain and extend.
- **Link**: A physical component of a robot, such as a body part or a sensor.
- **Joint**: The connection between two links, defining the type of movement (e.g., revolute, prismatic) and its properties.
- **Visual**: The visual representation of a link, including its geometry, color, and material.
- **Collision**: The collision geometry of a link, used for physics-based simulations.
- **ROS 2 (Robot Operating System 2)**: A widely-used framework for building and deploying robotic applications, providing tools and libraries for robot control, simulation, and more.

## Review Questions

1. Explain the purpose and structure of a URDF file. What are the main elements used to describe a robot model?
2. Describe the advantages of using XACRO to create URDF descriptions. How does XACRO improve the modularity and maintainability of robot models?
3. Outline the general process of integrating a URDF/XACRO-based robot model into a ROS 2 simulation environment. What are the key steps involved?