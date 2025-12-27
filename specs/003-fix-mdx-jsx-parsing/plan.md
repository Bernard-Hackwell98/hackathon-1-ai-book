# Implementation Plan: Fix MDX JSX Parsing Error in Docusaurus

**Branch**: `003-fix-mdx-jsx-parsing` | **Date**: 2025-01-04 | **Spec**: [link](./spec.md)
**Input**: Feature specification from `/specs/003-fix-mdx-jsx-parsing/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan addresses the MDX JSX parsing error in Docusaurus by systematically fixing raw XML tags (like `<inertial>`) that are being misinterpreted as JSX components. Based on research, the solution involves replacing raw XML tags with properly formatted alternatives: using inline code formatting (e.g., `<inertial>`) for single tags in text, and fenced code blocks with xml language identifier (```xml) for complete XML snippets. The fix will be applied to all .md/.mdx files under docs/modules, with special attention to assessment and quiz sections. The implementation approach maintains backward compatibility and preserves the original meaning and readability of XML examples.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+), MDX 2.x
**Primary Dependencies**: Docusaurus 3.x, remark/remark-gfm, rehype plugins
**Storage**: N/A (documentation files in repository)
**Testing**: Jest for unit tests, manual verification of build process
**Target Platform**: Web (Docusaurus static site generation)
**Project Type**: Single (documentation site)
**Performance Goals**: Build time should not significantly increase after fixes
**Constraints**: Must maintain backward compatibility with existing documentation
**Scale/Scope**: Apply to all .md/.mdx files under docs/modules, especially assessment and quiz sections

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Verification

**I. Spec-First, AI-Native Development** ✅
- Feature fully specified in `/specs/003-fix-mdx-jsx-parsing/spec.md` before implementation
- Following AI-assisted development workflow per Spec-Kit Plus

**II. Verifiable Accuracy via Primary Sources** ✅
- Solution based on Docusaurus and MDX official documentation
- Approach follows established best practices for handling XML in MDX

**III. Clarity for Technical Readers** ✅
- Documentation will maintain readability after XML tag fixes
- Proper code formatting will enhance understanding of XML examples

**IV. Reproducibility and Rigor** ✅
- Changes will be systematic and reproducible across all affected files
- Build process verification will confirm fixes work consistently

**V. Zero Plagiarism Tolerance** ✅
- All changes will be original implementation in existing documentation files
- No copying of content, only fixing formatting issues

### Gate Status: PASSED
All constitution principles are satisfied by this implementation approach.

## Post-Design Constitution Check

*Re-evaluation after Phase 1 design*

### Compliance Verification

**I. Spec-First, AI-Native Development** ✅
- Feature fully specified before implementation and design completed
- Following AI-assisted development workflow per Spec-Kit Plus

**II. Verifiable Accuracy via Primary Sources** ✅
- Solution based on Docusaurus and MDX official documentation
- Approach follows established best practices for handling XML in MDX

**III. Clarity for Technical Readers** ✅
- Documentation will maintain or improve readability after XML tag fixes
- Proper code formatting will enhance understanding of XML examples

**IV. Reproducibility and Rigor** ✅
- Changes will be systematic and reproducible across all affected files
- Build process verification will confirm fixes work consistently

**V. Zero Plagiarism Tolerance** ✅
- All changes will be original implementation in existing documentation files
- No copying of content, only fixing formatting issues

### Gate Status: PASSED
All constitution principles continue to be satisfied after Phase 1 design.

## Project Structure

### Documentation (this feature)

```text
specs/003-fix-mdx-jsx-parsing/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
This is a documentation-only change, so no new source code structure is needed. The changes will be applied to existing documentation files:

```text
docs/
└── modules/             # Where MDX files containing XML tags need fixing
    ├── assessment/      # Assessment and quiz sections that may contain XML
    └── [other modules]  # Other documentation modules with XML examples

# Docusaurus configuration
docusaurus.config.js      # May need updates for MDX processing if needed
package.json             # Dependencies for MDX processing
```

**Structure Decision**: This feature only involves updating existing documentation files to properly format XML tags. No new source code directories are required, just modifications to existing .md/.mdx files in the docs/modules directory.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
