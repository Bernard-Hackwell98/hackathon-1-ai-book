# Quickstart: Fix Docusaurus Root Route 404 (Local + GitHub Pages)

## Overview
This guide provides quick instructions for fixing the Docusaurus root route 404 issue by implementing proper routing configuration.

## Prerequisites
- Node.js 18+ installed
- Docusaurus project set up
- Access to the project configuration files

## Steps to Fix Root Route 404

### 1. Create the Index File
Create a new file at `docs/index.md` with the following frontmatter:

```
---
id: index
title: AI Book
---

# Welcome to the AI-Spec-Driven Open-Source Book

This is the landing page for the AI-Spec-Driven Open-Source Book on Physical AI and Robotics.

## Available Modules

- [ROS 2 for Humanoid Robotics](./modules/ros2-humanoid/intro)
- [Physical AI and Computer Vision](./modules/physical-ai/intro)

Select a module from the navigation menu to get started.
```

### 2. Update Docusaurus Configuration
In `docusaurus.config.js`, update the docs plugin configuration to route to the root:

```javascript
// In the presets section
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
        editUrl:
          'https://github.com/your-username/your-repo/tree/main/',
      },
      blog: false, // Optional: disable the blog plugin
      theme: {
        customCss: require.resolve('./src/css/custom.css'),
      },
    }),
  ],
],
```

### 3. Update Sidebar Configuration
In `sidebars.js`, ensure the index page is included as the first item:

```javascript
// Example sidebars.js structure
module.exports = {
  ros2HumanoidSidebar: [
    'index',  // Add index as the first item
    // ... other sidebar items
  ],
  physicalAISidebar: [
    'index',  // Add index as the first item
    // ... other sidebar items
  ],
};
```

### 4. Verify Configuration Settings
Ensure the following settings in `docusaurus.config.js`:

```javascript
// Make sure these are properly configured for GitHub Pages
url: 'https://your-username.github.io', // Your website URL
baseUrl: '/your-repo-name/', // Base path for GitHub Pages
trailingSlash: true, // Ensure consistent URL handling
```

### 5. Test the Changes
After making changes, verify that:

1. The site builds successfully:
   ```bash
   npm run build
   ```

2. The local development server works:
   ```bash
   npm run start
   ```

3. The root path (http://localhost:3000/) loads the new index page without 404 errors

4. All navigation links resolve to valid pages

## Troubleshooting
- If the site still shows 404 at the root, check that `routeBasePath: '/'` is set in the docs configuration
- If sidebar links don't work, verify that the index page is referenced correctly in sidebars.js
- If GitHub Pages deployment fails, ensure the baseUrl is set correctly for your repository