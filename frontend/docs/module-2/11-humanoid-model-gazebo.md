---
title: Building a Humanoid Model in Gazebo
sidebar_position: 11
description: Learn how to build and simulate a humanoid robot model in the Gazebo simulation environment.
tags: [gazebo, humanoid, model, simulation, robotics, ros2]
---

## Summary

This chapter will guide you through the process of building and simulating a humanoid robot model in the Gazebo simulation environment. You will learn how to create a custom URDF (Unified Robot Description Format) file to define the robot's physical structure, sensors, and joints, and then integrate it into the Gazebo ecosystem. By the end of this chapter, you will have a fully functional humanoid robot model that you can use for further development, testing, and experimentation.

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain the key components of a URDF file and how they are used to define a robot model
- Implement a custom URDF file for a humanoid robot with appropriate links, joints, and sensors
- Analyze the Gazebo simulation environment and its integration with ROS 2 for robotics applications
- Evaluate the performance and behavior of the simulated humanoid robot model in Gazebo
- Create a launch file to load the humanoid robot model into the Gazebo simulation

## Prerequisites

- Familiarity with ROS 2 concepts and workflows
- Basic understanding of URDF and robot modeling
- Experience with Gazebo simulation environment

## Building a Humanoid Model in Gazebo

### Understanding URDF and Robot Modeling

The Unified Robot Description Format (URDF) is an XML-based file format used to describe the physical structure of a robot, including its links, joints, and sensors. URDF files are essential for defining and simulating robots in the Gazebo environment, as they provide a comprehensive description of the robot's properties and capabilities.

:::tip
URDF files are not only used for simulation but also for real-world robot control and planning, as they can be integrated with various ROS 2 packages and libraries.
:::

When creating a URDF file for a humanoid robot, you'll need to consider factors such as the robot's limb lengths, joint types, and sensor placement. This information will be used to accurately represent the robot's physical structure and behavior in the simulation.

### Defining the Humanoid Robot in URDF

Let's start by creating a basic URDF file for a humanoid robot. We'll use the ROS 2 package `xacro` to define the robot's structure, as it allows for more modular and customizable URDF files.

```python
<?xml version="1.0"?>
<robot name="my_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="humanoid_robot.xacro" />

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5" />
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5" />
      </geometry>
    </collision>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_link" />
    <child link="torso_link" />
    <origin xyz="0 0 0.25" rpy="0 0 0" />
  </joint>

  <xacro:humanoid_robot prefix="robot_" />
</robot>

In this example, we've included a separate `humanoid_robot.xacro` file that defines the detailed structure of the humanoid robot, including its limbs, joints, and sensors. The `xacro:humanoid_robot` tag allows us to easily integrate this modular component into the main URDF file.

```

### Integrating the Humanoid Model into Gazebo

Now that we have a URDF file for our humanoid robot, we can integrate it into the Gazebo simulation environment. Gazebo provides a ROS 2 package called `gazebo_ros` that allows you to easily load and simulate your robot model.

```python
# Create a launch file to load the humanoid robot into Gazebo
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = os.path.join('gazebo_ros', 'launch')
    world_file = 'worlds/empty.world'

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='SDF world file'
        ),
        Node(
            package='gazebo_ros',
            executable='gazebo',
            arguments=['-s', 'libgazebo_ros_factory.so', '$(var world)'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity',
            arguments=['-entity', 'my_humanoid', '-file', 'path/to/my_humanoid.urdf'],
            output='screen'
        )
    ])
```

This launch file first loads the Gazebo simulation environment and then spawns the humanoid robot model into the simulation using the `spawn_entity` command from the `gazebo_ros` package. Make sure to replace `'path/to/my_humanoid.urdf'` with the actual file path to your URDF file.

:::tip
You can customize the Gazebo world by specifying a different `world_file` in the launch file. Gazebo provides a variety of pre-built worlds, or you can create your own.
:::

### Simulating and Interacting with the Humanoid Robot

Once you've launched the Gazebo simulation with the humanoid robot, you can start interacting with the model and observing its behavior. You can use ROS 2 tools and libraries to control the robot's joints, read sensor data, and implement various behaviors.

For example, you can create a ROS 2 node that publishes joint commands to the robot's actuators, allowing you to control its movement and posture. You can also subscribe to sensor topics to read data from the robot's cameras, IMUs, and other sensors.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.joint_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_commands)

    def publish_joint_commands(self):
        # Generate joint commands based on your control logic
        joint_commands = Float64MultiArray()
        joint_commands.data = [0.1, -0.2, 0.3, ...]
        self.joint_pub.publish(joint_commands)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example shows a basic ROS 2 node that publishes joint commands to the simulated humanoid robot in Gazebo. You can expand on this code to implement more complex control algorithms and behaviors.

## Key Takeaways

- URDF files are essential for defining the physical structure and properties of a robot, including links, joints, and sensors.
- Gazebo provides a ROS 2 integration that allows you to easily load and simulate your robot models in a realistic 3D environment.
- Integrating your humanoid robot model into Gazebo involves creating a URDF file and using a launch file to spawn the model into the simulation.
- You can use ROS 2 tools and libraries to control the robot's movements, read sensor data, and implement various behaviors in the Gazebo simulation.

## Glossary

1. **URDF (Unified Robot Description Format)**: An XML-based file format used to describe the physical structure of a robot, including its links, joints, and sensors.
2. **Gazebo**: A powerful 3D robotics simulation environment that integrates with ROS 2 for testing and development of robotic systems.
3. **ROS 2 (Robot Operating System 2)**: A collection of software frameworks for robot software development, providing operating system-like functionality on a heterogeneous computer cluster.
4. **Joint**: A connection between two links that allows for relative movement, such as a rotational or prismatic joint.
5. **Link**: A rigid body that represents a part of a robot's physical structure.

## Review Questions

1. Explain the purpose and key components of a URDF file in the context of robot modeling and simulation.
2. Describe the steps involved in integrating a custom humanoid robot model into the Gazebo simulation environment.
3. How can you use ROS 2 tools and libraries to control the movement and behavior of the simulated humanoid robot in Gazebo?