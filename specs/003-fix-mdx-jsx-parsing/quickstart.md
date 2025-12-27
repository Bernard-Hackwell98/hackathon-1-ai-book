# Quickstart: Fix MDX JSX Parsing Error in Docusaurus

## Overview
This guide provides quick instructions for fixing MDX JSX parsing errors caused by raw XML tags in Docusaurus documentation.

## Prerequisites
- Node.js 18+ installed
- Docusaurus project set up
- Access to the documentation files in `docs/modules/`

## Steps to Fix XML Tags in MDX Files

### 1. Identify Problematic XML Tags
Look for raw XML tags in MDX files that cause JSX parsing errors during build:
- Tags like `<inertial>`, `<link>`, `<joint>` used directly in text
- XML snippets not properly fenced as code blocks

### 2. Apply Appropriate Formatting

#### For Inline XML Tags
Replace raw XML tags within text with inline code formatting:

**Before:**
```
What is the purpose of the <inertial> tag in URDF?
```

**After:**
```
What is the purpose of the `<inertial>` tag in URDF?
```

#### For XML Code Blocks
Ensure XML snippets are properly fenced with the xml language identifier:

**Before:**
```
<robot name="my_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
    </inertial>
  </link>
</robot>
```

**After:**
```
```xml
<robot name="my_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
    </inertial>
  </link>
</robot>
```
```

### 3. Process All Affected Files
Apply the fixes systematically to all .md/.mdx files under `docs/modules/`:
- Navigate to the `docs/modules/` directory
- Process each file, focusing on assessment and quiz sections
- Use search functionality to find all occurrences of common XML tags

### 4. Verify the Fixes
After making changes, verify that:

1. The Docusaurus build completes successfully:
   ```bash
   npm run build
   ```

2. The documentation renders correctly in the UI:
   ```bash
   npm run start
   ```

3. XML examples are properly formatted and readable

## Common XML Tags to Look For
- `<inertial>`, `</inertial>`
- `<link>`, `</link>`
- `<joint>`, `</joint>`
- `<robot>`, `</robot>`
- `<visual>`, `</visual>`
- `<collision>`, `</collision>`
- `<geometry>`, `</geometry>`
- `<origin>`, `</origin>`

## Troubleshooting
- If build errors persist, check for XML tags inside list items or other JSX contexts
- Ensure all XML code blocks have the `xml` language identifier
- Verify that no XML tags are embedded within JSX expressions