#!/usr/bin/env node

// Script to identify raw XML tags in MDX files
// Usage: node find-xml-tags.js [directory]

const fs = require('fs');
const path = require('path');

// Common XML tags to look for (from the quickstart guide)
const xmlTags = [
  'inertial', 'link', 'joint', 'robot', 'visual', 'collision', 
  'geometry', 'origin', 'mass', 'inertia', 'material', 'axis'
];

// Regex pattern to match raw XML tags (not in code blocks)
// This looks for tags that are not properly formatted as inline code or fenced code blocks
const xmlTagPattern = /<(\w+)(\s+[^>]*)?>/g;

function findXmlTagsInFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const lines = content.split('\n');
  const results = [];

  // Check if file is likely to have JSX parsing issues
  // Look for XML tags that are not in code blocks or inline code
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];

    // Find all potential XML tags in the line
    const matches = [...line.matchAll(xmlTagPattern)];

    for (const match of matches) {
      const tagName = match[1];

      // Check if this tag is NOT in a properly formatted context
      // 1. Not in inline code: `<tag>`
      const hasInlineCodeFormat = line.includes('`<' + tagName) && line.includes(tagName + '>`');

      // 2. Not in a fenced code block (```xml)
      const isInCodeBlock = isWithinCodeBlock(content, i, line);

      // 3. Not part of JSX expression (inside curly braces)
      const isInJSXExpression = isWithinJSXExpression(content, i, match.index);

      // If tag is not properly formatted, it could cause JSX parsing errors
      if (xmlTags.includes(tagName) && !hasInlineCodeFormat && !isInCodeBlock && !isInJSXExpression) {
        results.push({
          line: i + 1,
          content: line.trim(),
          tag: match[0],
          tagName: tagName
        });
      }
    }
  }

  return results;
}

// Helper function to check if a line is within a code block
function isWithinCodeBlock(content, lineIndex, line) {
  const lines = content.split('\n');
  let inCodeBlock = false;
  let codeBlockType = '';

  for (let i = 0; i <= lineIndex; i++) {
    const currentLine = lines[i];

    if (currentLine.trim().startsWith('```')) {
      if (inCodeBlock) {
        // Ending a code block
        inCodeBlock = false;
        codeBlockType = '';
      } else {
        // Starting a code block
        inCodeBlock = true;
        codeBlockType = currentLine.trim().substring(3).trim();
      }
    }
  }

  return inCodeBlock;
}

// Helper function to check if a position is within a JSX expression
function isWithinJSXExpression(content, lineIndex, charIndex) {
  const lines = content.split('\n');
  let fullText = '';

  // Build text up to the target position
  for (let i = 0; i < lineIndex; i++) {
    fullText += lines[i] + '\n';
  }
  fullText += lines[lineIndex].substring(0, charIndex);

  // Count braces to see if we're inside JSX expression
  const openBraces = (fullText.match(/{/g) || []).length;
  const closeBraces = (fullText.match(/}/g) || []).length;

  // If more open braces than close braces, we're inside a JSX expression
  return openBraces > closeBraces;
}

function scanDirectory(dirPath) {
  const results = [];
  
  const items = fs.readdirSync(dirPath);
  
  for (const item of items) {
    const fullPath = path.join(dirPath, item);
    const stat = fs.statSync(fullPath);
    
    if (stat.isDirectory()) {
      results.push(...scanDirectory(fullPath));
    } else if (item.endsWith('.md') || item.endsWith('.mdx')) {
      const xmlTags = findXmlTagsInFile(fullPath);
      if (xmlTags.length > 0) {
        results.push({
          file: fullPath,
          tags: xmlTags
        });
      }
    }
  }
  
  return results;
}

// Main execution
const directory = process.argv[2] || './docs/modules';
const results = scanDirectory(directory);

console.log('XML tags found in MDX files:');
console.log('=============================');

if (results.length === 0) {
  console.log('No raw XML tags found that need fixing.');
} else {
  for (const result of results) {
    console.log(`\nFile: ${result.file}`);
    for (const tag of result.tags) {
      console.log(`  Line ${tag.line}: ${tag.content}`);
      console.log(`    Tag: ${tag.tag}`);
    }
  }
}