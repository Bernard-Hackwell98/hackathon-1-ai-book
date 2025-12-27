import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/hackathon-1-ai-book/__docusaurus/debug/',
    component: ComponentCreator('/hackathon-1-ai-book/__docusaurus/debug/', 'd3e'),
    exact: true
  },
  {
    path: '/hackathon-1-ai-book/__docusaurus/debug/config/',
    component: ComponentCreator('/hackathon-1-ai-book/__docusaurus/debug/config/', '711'),
    exact: true
  },
  {
    path: '/hackathon-1-ai-book/__docusaurus/debug/content/',
    component: ComponentCreator('/hackathon-1-ai-book/__docusaurus/debug/content/', '446'),
    exact: true
  },
  {
    path: '/hackathon-1-ai-book/__docusaurus/debug/globalData/',
    component: ComponentCreator('/hackathon-1-ai-book/__docusaurus/debug/globalData/', 'b59'),
    exact: true
  },
  {
    path: '/hackathon-1-ai-book/__docusaurus/debug/metadata/',
    component: ComponentCreator('/hackathon-1-ai-book/__docusaurus/debug/metadata/', '7ac'),
    exact: true
  },
  {
    path: '/hackathon-1-ai-book/__docusaurus/debug/registry/',
    component: ComponentCreator('/hackathon-1-ai-book/__docusaurus/debug/registry/', '357'),
    exact: true
  },
  {
    path: '/hackathon-1-ai-book/__docusaurus/debug/routes/',
    component: ComponentCreator('/hackathon-1-ai-book/__docusaurus/debug/routes/', 'b58'),
    exact: true
  },
  {
    path: '/hackathon-1-ai-book/',
    component: ComponentCreator('/hackathon-1-ai-book/', 'b04'),
    routes: [
      {
        path: '/hackathon-1-ai-book/',
        component: ComponentCreator('/hackathon-1-ai-book/', 'cb0'),
        exact: true,
        sidebar: "physicalAISidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/physical-ai/assessments/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/physical-ai/assessments/', 'dc1'),
        exact: true
      },
      {
        path: '/hackathon-1-ai-book/modules/physical-ai/chapter-1-intro/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/physical-ai/chapter-1-intro/', '751'),
        exact: true,
        sidebar: "physicalAISidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/physical-ai/chapter-2-vision/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/physical-ai/chapter-2-vision/', '151'),
        exact: true,
        sidebar: "physicalAISidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/physical-ai/chapter-3-perception-action/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/physical-ai/chapter-3-perception-action/', '3e8'),
        exact: true,
        sidebar: "physicalAISidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/physical-ai/entities/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/physical-ai/entities/', '1af'),
        exact: true
      },
      {
        path: '/hackathon-1-ai-book/modules/physical-ai/glossary/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/physical-ai/glossary/', 'db7'),
        exact: true
      },
      {
        path: '/hackathon-1-ai-book/modules/physical-ai/intro/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/physical-ai/intro/', '474'),
        exact: true,
        sidebar: "physicalAISidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/physical-ai/validation-report/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/physical-ai/validation-report/', 'add'),
        exact: true
      },
      {
        path: '/hackathon-1-ai-book/modules/ros2-humanoid/assessments/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/ros2-humanoid/assessments/', '6b9'),
        exact: true
      },
      {
        path: '/hackathon-1-ai-book/modules/ros2-humanoid/chapter-1-intro/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/ros2-humanoid/chapter-1-intro/', '50a'),
        exact: true,
        sidebar: "ros2HumanoidSidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/ros2-humanoid/chapter-2-communication/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/ros2-humanoid/chapter-2-communication/', '4ac'),
        exact: true,
        sidebar: "ros2HumanoidSidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/ros2-humanoid/chapter-3-urdf/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/ros2-humanoid/chapter-3-urdf/', 'de4'),
        exact: true,
        sidebar: "ros2HumanoidSidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/ros2-humanoid/glossary/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/ros2-humanoid/glossary/', 'f2c'),
        exact: true
      },
      {
        path: '/hackathon-1-ai-book/modules/ros2-humanoid/intro/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/ros2-humanoid/intro/', '11e'),
        exact: true,
        sidebar: "ros2HumanoidSidebar"
      },
      {
        path: '/hackathon-1-ai-book/modules/ros2-humanoid/validation-report/',
        component: ComponentCreator('/hackathon-1-ai-book/modules/ros2-humanoid/validation-report/', '4ff'),
        exact: true
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
