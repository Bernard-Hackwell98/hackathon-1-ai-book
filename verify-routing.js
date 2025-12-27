#!/usr/bin/env node

// Script to verify routing configuration works correctly
// Usage: node verify-routing.js

const fs = require('fs');
const path = require('path');

console.log('Verifying Docusaurus routing configuration...\n');

// Check if docs/index.md exists
const indexPath = './docs/index.md';
const indexExists = fs.existsSync(indexPath);

if (indexExists) {
  console.log('✅ docs/index.md exists');
  const indexContent = fs.readFileSync(indexPath, 'utf8');
  if (indexContent.includes('id: index') && indexContent.includes('title: AI Book')) {
    console.log('✅ docs/index.md has correct frontmatter');
  } else {
    console.log('❌ docs/index.md missing correct frontmatter');
  }
} else {
  console.log('❌ docs/index.md does not exist');
}

// Check docusaurus.config.js for routeBasePath configuration
const configPath = './docusaurus.config.js';
if (fs.existsSync(configPath)) {
  console.log('\n✅ docusaurus.config.js exists');
  const configContent = fs.readFileSync(configPath, 'utf8');
  
  if (configContent.includes("routeBasePath: '/'")) {
    console.log('✅ routeBasePath is set to "/"');
  } else {
    console.log('❌ routeBasePath is NOT set to "/"');
  }
  
  if (configContent.includes('trailingSlash: true')) {
    console.log('✅ trailingSlash is set to true');
  } else {
    console.log('⚠️  trailingSlash is NOT set to true (may cause issues)');
  }
} else {
  console.log('❌ docusaurus.config.js does not exist');
}

// Check sidebars.js for index reference
const sidebarPath = './sidebars.js';
if (fs.existsSync(sidebarPath)) {
  console.log('\n✅ sidebars.js exists');
  const sidebarContent = fs.readFileSync(sidebarPath, 'utf8');
  
  if (sidebarContent.includes("'index'") || sidebarContent.includes('"index"')) {
    console.log('✅ sidebars.js contains index reference');
  } else {
    console.log('❌ sidebars.js does NOT contain index reference');
  }
} else {
  console.log('❌ sidebars.js does not exist');
}

console.log('\nRouting verification complete!');