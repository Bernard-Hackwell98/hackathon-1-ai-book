// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  ros2HumanoidSidebar: [
    'index',
    {
      type: 'category',
      label: 'ROS 2 for Humanoid Robotics',
      items: [
        'modules/ros2-humanoid/intro',
        {
          type: 'category',
          label: 'Chapter 1: Introduction to ROS 2 for Humanoid Robotics',
          items: [
            'modules/ros2-humanoid/chapter-1-intro/index',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: ROS 2 Communication Primitives',
          items: [
            'modules/ros2-humanoid/chapter-2-communication/index',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Robot Modeling with URDF',
          items: [
            'modules/ros2-humanoid/chapter-3-urdf/index',
          ],
        },
      ],
    },
  ],
  physicalAISidebar: [
    'index',
    {
      type: 'category',
      label: 'Physical AI and Computer Vision',
      items: [
        'modules/physical-ai/intro',
        {
          type: 'category',
          label: 'Chapter 1: Introduction to Physical AI',
          items: [
            'modules/physical-ai/chapter-1-intro/index',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: ROS 2 Vision Primitives',
          items: [
            'modules/physical-ai/chapter-2-vision/index',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Perception-Action Systems',
          items: [
            'modules/physical-ai/chapter-3-perception-action/index',
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;