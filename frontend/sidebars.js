/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main textbook sidebar with 4 modules (25 chapters)
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: Foundations of Physical AI & Robotics',
      collapsed: false,
      items: [
        'module-1/introduction-to-physical-ai',
        'module-1/robot-hardware',
        'module-1/mathematics-for-robotics',
        'module-1/introduction-to-ros2',
        'module-1/linear-algebra-calculus',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments & Robotics Software',
      collapsed: true,
      items: [
        'module-2/ros2-in-depth',
        'module-2/building-ros-applications',
        'module-2/urdf-xacro-modeling',
        'module-2/gazebo-comparison',
        'module-2/humanoid-model-gazebo',
        'module-2/unity-robotics',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Advanced Perception, Navigation & Control',
      collapsed: true,
      items: [
        'module-3/computer-vision',
        'module-3/isaac-sim-fundamentals',
        'module-3/isaac-ros-perception',
        'module-3/nav2-navigation',
        'module-3/mapping-localization',
        'module-3/motion-planning-humanoids',
        'module-3/vision-language-action',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Humanoid AI Systems & Capstone Development',
      collapsed: true,
      items: [
        'module-4/integrating-perception-action-control',
        'module-4/ai-agents-robotics',
        'module-4/end-to-end-humanoid-pipeline',
        'module-4/multi-agent-coordination',
        'module-4/project-autonomous-humanoid',
        'module-4/final-capstone',
      ],
    },
  ],
};

module.exports = sidebars;
