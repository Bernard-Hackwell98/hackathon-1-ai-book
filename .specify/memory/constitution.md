<!--
Sync Impact Report:
- Version change: N/A -> 1.0.0
- Modified principles: N/A (new constitution)
- Added sections: Core Principles, Book Standards, AI Authoring Constraints, RAG Chatbot Standards, Deployment & Integration
- Removed sections: N/A
- Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
- Follow-up TODOs: None
-->

# AI-Spec-Driven Open-Source Book Constitution

## Core Principles

### I. Spec-First, AI-Native Development
All features and content must be defined in specifications before implementation. Development workflows integrate AI tools for efficiency while maintaining human oversight and approval at critical decision points.

### II. Verifiable Accuracy via Primary Sources
All content and code examples must be traceable to authoritative sources. Claims must be supported by peer-reviewed research, official documentation, or verifiable data. No secondary sources without verification of original material.

### III. Clarity for Technical Readers
Content must be accessible to the target audience (computer science students, AI engineers, researchers) with clear explanations, well-structured examples, and appropriate technical depth. Writing must meet Flesch-Kincaid grade level 11-13.

### IV. Reproducibility and Rigor
All code examples, experiments, and processes must be reproducible. Clear setup instructions, version specifications, and verification steps are mandatory. Scientific rigor applies to all technical content.

### V. Zero Plagiarism Tolerance
All content must be original or properly attributed. AI-generated content must be clearly identified and reviewed for accuracy. Direct copying from other sources without attribution is strictly prohibited.

## Book Standards

### Format and Structure
- Format: Docusaurus (MD/MDX)
- Clear chapter structure with objectives, content, and summaries
- Code examples must be runnable and secure
- Citations: APA style
- Sources: ≥60% peer-reviewed or official documentation
- Writing level: Flesch-Kincaid 11–13

### Content Quality
- Each chapter must have clear learning objectives
- Content must be factually accurate and up-to-date
- Examples should be practical and relevant to the field
- All external links must be verified and stable

## AI Authoring Constraints

### Accuracy Requirements
- No hallucinated facts or citations
- All claims must be traceable to verified sources
- Explicitly flag uncertain or speculative information
- Follow Spec-Kit Plus specifications strictly

### Review Process
- All AI-generated content requires human verification
- Technical content must be reviewed by domain experts
- Fact-checking against primary sources is mandatory
- Regular audits of AI-generated content for accuracy

## RAG Chatbot Standards

### Technical Architecture
- Backend: FastAPI
- LLM: OpenAI Agents / ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- DB: Neon Serverless Postgres
- Retrieval only from indexed book content

### Response Requirements
- Selected-text mode must answer only from user-highlighted text
- If information not found, respond: "Answer not contained in the provided text."
- Responses must be grounded in book content only
- No generation of content outside the book's scope

## Deployment & Integration

### Infrastructure
- GitHub Pages with explicit trailingSlash configuration
- Embedded chatbot in Docusaurus pages
- Single monorepo with clear service separation
- Automated deployment pipeline

### Quality Assurance
- All services must pass integration tests before deployment
- Performance benchmarks must be met
- Security scanning required for all dependencies
- Accessibility compliance (WCAG 2.1 AA)

## Governance

This constitution supersedes all other practices and guidelines in the project. All development, content creation, and deployment activities must comply with these principles.

Amendments require:
1. Documentation of the proposed change with rationale
2. Approval from project maintainers
3. Migration plan for existing content/code if needed
4. Update to all dependent templates and documentation

All pull requests and code reviews must verify compliance with these principles. Complexity must be justified with clear benefits to the project's core mission.

**Version**: 1.0.0 | **Ratified**: 2025-01-01 | **Last Amended**: 2025-12-26
