#!/usr/bin/env node

// Test build command to verify fixes work correctly
// Usage: node test-build.js

const { exec } = require('child_process');
const fs = require('fs');

console.log('Testing Docusaurus build after XML tag fixes...\n');

// Run the build command
const buildProcess = exec('npm run build', (error, stdout, stderr) => {
  if (error) {
    console.log(`Build failed with error code: ${error.code}`);
    console.log(`Error: ${error.message}`);
    console.log(`Stderr: ${stderr}`);
    
    // Check if it's a JSX parsing error
    if (stderr.includes('JSX') || stderr.includes('SyntaxError') || stderr.includes('closing tag')) {
      console.log('\n❌ Build failed due to JSX parsing errors - XML tags still need fixing');
      return;
    }
  }
  
  if (stdout.includes('Compiled successfully')) {
    console.log('✅ Build completed successfully!');
    console.log('✅ No JSX parsing errors detected');
  } else {
    console.log('Build output:');
    console.log(stdout);
  }
});

buildProcess.stdout.on('data', (data) => {
  if (data.includes('JSX') || data.includes('SyntaxError') || data.includes('closing tag')) {
    console.log(`Potential JSX error detected: ${data}`);
  }
});

buildProcess.stderr.on('data', (data) => {
  if (data.includes('JSX') || data.includes('SyntaxError') || data.includes('closing tag')) {
    console.log(`JSX error in stderr: ${data}`);
  }
});