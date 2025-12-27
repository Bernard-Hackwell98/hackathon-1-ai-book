# Quickstart: Fix Docusaurus Routing

## Prerequisites
- Node.js installed (version 18 or higher)
- npm or yarn package manager
- Git for version control
- Access to the project repository

## Setup
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Install dependencies:
   ```bash
   npm install
   # or
   yarn install
   ```

## Implementation Steps

### Step 1: Create Documentation Homepage
1. Create a new file at `docs/index.md`
2. Add the following frontmatter and content:
   ```markdown
   ---
   id: docs-home
   title: Documentation
   ---

   # Documentation

   Welcome to our documentation. Use the sidebar to navigate through the available topics.
   ```

### Step 2: Update Navigation Configuration
1. Open `docusaurus.config.js`
2. Find the navbar configuration
3. Update the "Docs" link to point to `/docs` instead of `/`
4. Example:
   ```javascript
   navbar: {
     items: [
       {
         type: 'docSidebar',
         sidebarId: 'tutorialSidebar',
         position: 'left',
         label: 'Docs',
         to: '/docs/',  // Changed from '/'
       },
       // ... other items
     ],
   },
   ```

### Step 3: Configure Site Settings
1. In `docusaurus.config.js`, ensure the following settings:
   ```javascript
   module.exports = {
     url: 'https://bernard-hackwell98.github.io',
     baseUrl: '/hackathon-1-ai-book/',
     trailingSlash: true,  // Add this if not present
     // ... other settings
   };
   ```

### Step 4: Validate Sidebar References
1. Open `sidebars.js`
2. Check that all documentation IDs referenced in the sidebar exist as actual documentation files
3. Remove or update any references to non-existent documentation pages

## Testing
1. Start the development server:
   ```bash
   npm run start
   # or
   yarn start
   ```

2. Navigate to the following URLs to verify the fixes:
   - Site homepage: `http://localhost:3000/`
   - Documentation homepage: `http://localhost:3000/docs/`
   - Click the "Docs" link in the navbar

3. Verify that no 404 errors occur when navigating through the documentation

## Deployment
1. Build the site:
   ```bash
   npm run build
   # or
   yarn build
   ```

2. Deploy to GitHub Pages following your project's deployment process

## Troubleshooting
- If the documentation homepage still shows 404, verify that `docs/index.md` exists with proper frontmatter
- If navigation links don't work, check that `docusaurus.config.js` has the correct paths
- If URLs don't have trailing slashes as expected, verify the `trailingSlash: true` setting