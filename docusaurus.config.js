// @ts-check
// `@ts-check` enables ts-checking for the config file to catch any type issues

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI-Spec-Driven Open-Source Book',
  tagline: 'Physical AI and Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://bernard-hackwell98.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/hackathon-1-ai-book/',

  // GitHub pages deployment config.
  organizationName: 'bernard-hackwell98', // Usually your GitHub org/user name.
  projectName: 'hackathon-1-ai-book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/bernard-hackwell98/hackathon-1-ai-book/tree/main/',
          routeBasePath: '/', // Serve docs at the root route
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'AI Book',
        logo: {
          alt: 'AI Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'ros2HumanoidSidebar',
            position: 'left',
            label: 'ROS 2 Module',
          },
          {
            type: 'docSidebar',
            sidebarId: 'physicalAISidebar',
            position: 'left',
            label: 'Physical AI Module',
          },
          {
            href: 'https://bernard-hackwell98.github.io/hackathon-1-ai-book/',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'ROS 2 for Humanoid Robotics',
                to: '/modules/ros2-humanoid/intro',
              },
              {
                label: 'Physical AI and Computer Vision',
                to: '/modules/physical-ai/intro',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://bernard-hackwell98.github.io/hackathon-1-ai-book/',
                
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AI-Spec-Driven Open-Source Book. Built with Docusaurus.`,
      },
    }),
};

module.exports = config;