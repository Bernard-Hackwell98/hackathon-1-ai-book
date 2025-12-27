// @ts-check
// `@type` JSDoc annotations allow IDEs and type-checking tools to autocomplete
// and validate function arguments and return values, based on the jsdoc tags.

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
  organizationName: 'bernardhackwell98', // Usually your GitHub org/user name.
  projectName: 'hackathon-1-ai-book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: true,

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
          sidebarPath: require.resolve('./sidebars.js'),
          // Serve the docs at the site root
          routeBasePath: '/',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/bernard-hackwell98/hackathon-1-ai-book/tree/main/',
        },
        blog: false, // Optional: disable the blog plugin
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
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
            href: 'https://github.com/bernard-hackwell98/hackathon-1-ai-book',
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
                href: 'https://github.com/bernard-hackwell98/hackathon-1-ai-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AI-Spec-Driven Open-Source Book. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer/themes/github'),
        darkTheme: require('prism-react-renderer/themes/dracula'),
        additionalLanguages: ['python', 'bash'],
      },
    }),
};

module.exports = config;