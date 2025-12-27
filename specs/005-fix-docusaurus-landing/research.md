# Research: Docusaurus Landing Page Implementation

## Decision: Use docs/index.md approach for landing page

### Rationale:
Based on the feature specification and Docusaurus best practices, implementing the landing page as `docs/index.md` is the recommended approach. This follows the docs-based landing page pattern (Option A) as specified in the requirements.

### Alternatives considered:
1. **Custom homepage with src/pages/index.tsx**: This would involve creating a custom React page component. While offering more flexibility, it would be more complex and potentially inconsistent with the documentation-based structure of the rest of the site.

2. **docs/index.md approach**: This approach treats the landing page as the first document in the documentation set, which is consistent with Docusaurus conventions. It allows for rich Markdown/MDX content with frontmatter configuration.

3. **Using Docusaurus' built-in homepages**: Docusaurus offers some built-in homepage components, but these might not match the specific textbook branding requirements.

### Decision justification:
The docs/index.md approach was chosen because:
- It aligns with the feature specification requirement (Option A)
- It maintains consistency with the existing documentation structure
- It's simpler to implement and maintain than a custom React component
- It follows Docusaurus best practices for documentation sites
- It allows for rich content with proper frontmatter configuration
- It ensures the landing page integrates seamlessly with the navbar and footer

## Decision: Frontmatter configuration

### Rationale:
The frontmatter for docs/index.md will include the required id and title as specified in the feature requirements.

### Configuration:
```yaml
---
id: index
title: Physical AI & Humanoid Robotics Textbook
---
```

This matches the exact specification requirement and will ensure the page is properly recognized by Docusaurus as the homepage.

## Decision: Hero-style content structure

### Rationale:
The landing page needs to include hero-style content that introduces the textbook, as specified in the requirements.

### Content approach:
- A compelling headline that introduces the textbook
- Brief description of the textbook's purpose and content
- Clear "Start Reading" call-to-action button that links to the first module
- Any additional visual elements that reinforce the textbook's theme

## Decision: Linking to first module

### Rationale:
The landing page must provide a "Start Reading" call-to-action that links to the first module documentation, as specified in the requirements.

### Implementation:
- Identify the first module documentation page in the existing docs structure
- Create a prominent link/button with text "Start Reading"
- Ensure the link properly navigates to the first module