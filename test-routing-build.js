#!/usr/bin/env node

// Test build command to verify routing fixes work correctly
// Usage: node test-build.js

const { exec } = require('child_process');
const fs = require('fs');

console.log('Testing Docusaurus build after routing fixes...\n');

// Run the build command
const buildProcess = exec('npm run build', (error, stdout, stderr) => {
  if (error) {
    console.log(`Build failed with error code: ${error.code}`);
    console.log(`Error: ${error.message}`);
    return;
  }
  
  if (stdout.includes('Compiled successfully')) {
    console.log('✅ Build completed successfully!');
    
    // Check for broken links which would indicate routing issues
    if (stdout.includes('Docusaurus found broken links')) {
      console.log('⚠️  Build completed but found broken links - routing may still have issues');
    } else {
      console.log('✅ No broken links detected - routing appears to be working correctly');
    }
  } else {
    console.log('Build output:');
    console.log(stdout);
  }
});

buildProcess.stdout.on('data', (data) => {
  if (data.includes('404') || data.includes('broken links')) {
    console.log(`Potential routing issue detected: ${data}`);
  }
});