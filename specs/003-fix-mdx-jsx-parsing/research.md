# Research: Fix MDX JSX Parsing Error in Docusaurus

## Overview
This research document addresses the MDX JSX parsing error in Docusaurus, specifically how to handle raw XML tags (like `<inertial>`) that are being misinterpreted as JSX components.

## Decision: Approach for Handling XML in MDX
**Rationale**: MDX treats angle-bracket tokens as JSX components unless properly escaped or placed in code. The solution involves systematically replacing raw XML tags with properly formatted alternatives.

**Chosen Approach**:
1. For inline XML tags: Use inline code formatting (e.g., `<inertial>` becomes `<inertial>`)
2. For XML code blocks: Use fenced code blocks with xml language identifier (```xml)
3. For escaped text: Use HTML entities when needed (e.g., &lt;inertial&gt;)

## Alternatives Considered
1. **Custom MDX plugin**: Create a custom remark/rehype plugin to handle XML tags automatically
   - Pros: Automated processing, consistent application
   - Cons: More complex implementation, potential maintenance overhead
   - Rejected because: The manual approach is simpler and more transparent

2. **JSX-compatible XML components**: Wrap XML in JSX components
   - Pros: Would work within JSX context
   - Cons: Would change the semantic meaning, make XML examples less clear
   - Rejected because: Would make XML examples less readable and accurate

3. **Raw HTML blocks**: Use HTML escape sequences
   - Pros: Would prevent JSX parsing
   - Cons: Less readable, harder to maintain
   - Rejected because: Code formatting provides better readability

## Best Practices for XML in MDX
Based on Docusaurus and MDX documentation:
- Use inline code formatting for single XML tags within text
- Use fenced code blocks with appropriate language identifier for complete XML snippets
- Avoid raw XML tags in JSX contexts (list items, JSX expressions)
- Maintain semantic meaning while ensuring proper parsing

## Implementation Strategy
1. Scan all .md/.mdx files under docs/modules
2. Identify raw XML tags that cause JSX parsing errors
3. Apply appropriate formatting based on context:
   - Inline mentions: Use `<tagname>` format
   - Code examples: Use ```xml code blocks
4. Test build process to ensure no parsing errors remain
5. Verify that content renders correctly in UI

## Tools for Implementation
- Text editor with find-and-replace capabilities
- Docusaurus build process for verification
- Git for tracking changes systematically