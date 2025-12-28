import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/hackathon-1-ai-book/',
    component: ComponentCreator('/hackathon-1-ai-book/', '02c'),
    exact: true
  },
  {
    path: '/hackathon-1-ai-book/',
    component: ComponentCreator('/hackathon-1-ai-book/', '66a'),
    routes: [
      {
        path: '/hackathon-1-ai-book/intro',
        component: ComponentCreator('/hackathon-1-ai-book/intro', '1ec'),
        exact: true
      },
      {
        path: '/hackathon-1-ai-book/modules/physical-ai/intro',
        component: ComponentCreator('/hackathon-1-ai-book/modules/physical-ai/intro', '0fb'),
        exact: true,
        sidebar: "physicalAISidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/ros2-humanoid/intro',
        component: ComponentCreator('/hackathon-1-ai-book/modules/ros2-humanoid/intro', '519'),
        exact: true,
        sidebar: "ros2HumanoidSidebar"
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
