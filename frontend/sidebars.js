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
        'module-1/01-introduction-to-physical-ai',
        'module-1/02-history-evolution-robotics',
        'module-1/03-key-concepts-terminology',
        'module-1/04-hardware-components-sensors',
        'module-1/05-actuators-control-systems',
        'module-1/06-ethical-considerations',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments & Robotics Software',
      collapsed: true,
      items: [
        'module-2/07-introduction-simulation',
        'module-2/08-nvidia-isaac-sim',
        'module-2/09-gazebo-simulator',
        'module-2/10-ros2-fundamentals',
        'module-2/11-ros2-advanced-topics',
        'module-2/12-integration-simulation-ros2',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Advanced Perception, Navigation & Control',
      collapsed: true,
      items: [
        'module-3/13-computer-vision-robotics',
        'module-3/14-sensor-fusion-techniques',
        'module-3/15-slam-mapping',
        'module-3/16-path-planning-algorithms',
        'module-3/17-motion-control-systems',
        'module-3/18-machine-learning-perception',
        'module-3/19-real-time-decision-making',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Humanoid AI Systems & Capstone Development',
      collapsed: true,
      items: [
        'module-4/20-humanoid-robotics-overview',
        'module-4/21-bipedal-locomotion',
        'module-4/22-human-robot-interaction',
        'module-4/23-manipulation-grasping',
        'module-4/24-integration-ai-systems',
        'module-4/25-capstone-project-guide',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      collapsed: true,
      items: [
        'resources/glossary',
        'resources/tools-libraries',
        'resources/further-reading',
      ],
    },
  ],
};

module.exports = sidebars;
