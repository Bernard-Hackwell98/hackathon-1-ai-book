# Data Model: Fix MDX JSX Parsing Error in Docusaurus

## Overview
This data model describes the entities and structures involved in fixing MDX JSX parsing errors related to XML tags in Docusaurus documentation.

## Key Entities

### MDX Files
- **Description**: Documentation files in the docs/modules directory that may contain raw XML tags
- **Format**: .md or .mdx files
- **Attributes**: 
  - path: file location in docs/modules
  - content: text content that may contain XML tags
  - status: whether the file has been processed for XML tag fixes

### XML Tags
- **Description**: Technical tags used in URDF and other XML formats that need proper escaping in MDX
- **Format**: <tagname> or <tagname attribute="value">
- **Attributes**:
  - name: the tag name (e.g., "inertial", "link", "joint")
  - context: where the tag appears (inline text, code block, list item)
  - format: how the tag should be formatted (inline code, escaped, code block)

### Code Blocks
- **Description**: Fenced sections containing XML examples that need proper syntax highlighting
- **Format**: ```xml ... ```
- **Attributes**:
  - language: the language identifier (xml)
  - content: the XML code content
  - position: location within the MDX file

### Assessment Content
- **Description**: Educational content that may contain XML examples in questions or answers
- **Format**: MDX with potential XML examples
- **Attributes**:
  - type: question, answer, explanation
  - content: the text content
  - xml_tags: list of XML tags that need fixing

## Validation Rules

### From Functional Requirements
- **FR-001**: Raw XML tags like `<inertial>` must be replaced with properly escaped versions or code formatting
- **FR-002**: URDF snippets must be properly fenced with ```xml syntax
- **FR-003**: All .md/.mdx files under docs/modules must be processed
- **FR-004**: XML tags in assessment and quiz sections must not cause build errors
- **FR-005**: Original meaning and readability of XML examples must be preserved
- **FR-006**: Fixes must be applied consistently to both inline XML tags and XML code blocks
- **FR-007**: Converted XML content must render correctly in the UI

## State Transitions

### MDX File Processing
1. **Unprocessed**: File contains raw XML tags that cause JSX parsing errors
2. **Identified**: Raw XML tags have been identified for fixing
3. **Fixed**: Raw XML tags have been replaced with properly formatted alternatives
4. **Verified**: File builds successfully without JSX parsing errors