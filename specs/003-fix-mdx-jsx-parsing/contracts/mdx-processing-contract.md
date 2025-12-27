# Contract: MDX XML Tag Processing

## Overview
This contract defines the expected behavior for handling XML tags in MDX files to prevent JSX parsing errors in Docusaurus.

## Input Contract
- **Input Type**: MDX files containing XML tags
- **Input Location**: All .md/.mdx files under docs/modules/
- **Input Format**: Markdown/MDX with potential raw XML tags

## Processing Rules

### Rule 1: Inline XML Tags
- **Condition**: XML tags appear within regular text content
- **Action**: Format as inline code using backticks
- **Example**: `<inertial>` becomes `<inertial>`

### Rule 2: XML Code Blocks
- **Condition**: Multi-line XML snippets
- **Action**: Format as fenced code blocks with xml language identifier
- **Example**: 
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

### Rule 3: Assessment and Quiz Content
- **Condition**: XML tags in assessment/quiz sections
- **Action**: Apply same formatting rules as general content
- **Validation**: Ensure no JSX parsing errors occur

## Output Contract
- **Output Type**: Valid MDX files that build without JSX parsing errors
- **Output Location**: Same files as input, with formatting fixes applied
- **Output Format**: MDX with properly formatted XML content

## Validation Criteria
1. Docusaurus build completes successfully without JSX parsing errors
2. XML examples render correctly in the UI with proper syntax highlighting
3. Original meaning and readability of XML examples preserved
4. All content in docs/modules/ processed according to rules

## Error Handling
- If XML tags are found within JSX expressions, they must be reformatted appropriately
- If XML tags are in list items, they should be formatted as inline code
- If malformed XML is encountered, it should be corrected or properly escaped