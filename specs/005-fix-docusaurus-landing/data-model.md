# Data Model: Docusaurus Landing Page

## Entities

### Landing Page Content
- **id**: string - Unique identifier for the page (value: "index")
- **title**: string - Title displayed on the page and in the browser tab (value: "Physical AI & Humanoid Robotics Textbook")
- **heroContent**: string - Markdown/MDX content for the hero section
- **ctaText**: string - Text for the call-to-action button (value: "Start Reading")
- **ctaLink**: string - URL/path to the first module documentation

### Navigation Elements
- **navbar**: object - Standard Docusaurus navigation bar (inherited from site config)
- **footer**: object - Standard Docusaurus footer (inherited from site config)

## Relationships
- The Landing Page Content entity uses the Navigation Elements for consistent site layout
- The ctaLink property references the first module documentation page in the docs directory

## Validation Rules
- title must match the specified value "Physical AI & Humanoid Robotics Textbook"
- id must be "index" to function as the homepage
- ctaLink must point to a valid documentation page
- heroContent must be valid Markdown/MDX syntax