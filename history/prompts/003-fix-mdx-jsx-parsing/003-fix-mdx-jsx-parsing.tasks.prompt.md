---
id: 3
title: fix-mdx-jsx-parsing
stage: tasks
date: 2025-01-04
surface: agent
model: Qwen
feature: fix-mdx-jsx-parsing
branch: 003-fix-mdx-jsx-parsing
user: 
command: sp.tasks
labels: ["mdx", "jsx", "parsing", "docusaurus", "xml", "tasks"]
links:
  spec: ../specs/003-fix-mdx-jsx-parsing/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-fix-mdx-jsx-parsing/tasks.md
tests:
---

## Prompt

Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.

Fix MDX JSX Parsing Error in Docusaurus Problem: Docusaurus MDX build fails because raw XML/HTML tags (e.g. <inertial>) are interpreted as JSX. Root cause: MDX treats angle-bracket tokens as JSX components unless escaped or placed in code. Required fixes: - Replace raw tags like <inertial> with one of: - Inline code: `<inertial>` - Escaped text: &lt;inertial&gt; - Code block (preferred for URDF/XML examples) - Do NOT embed XML tags inside JSX expressions or list items as raw text - Ensure all URDF snippets are fenced with ```xml Example fix: Change: What is the purpose of the <inertial> tag in URDF? To: What is the purpose of the `<inertial>` tag in URDF? Scope: - Apply to all .md/.mdx files under docs/modules - Especially assessment and quiz sections Success criteria: - Build completes without MDX parser errors - No JSX-related syntax errors - Content renders correctly in UI

## Response snapshot

# Implementation Tasks: Fix MDX JSX Parsing Error in Docusaurus

**Feature**: Fix MDX JSX Parsing Error in Docusaurus  
**Branch**: `003-fix-mdx-jsx-parsing`  
**Created**: 2025-01-04  
**Input**: Feature specification and design artifacts from `/specs/003-fix-mdx-jsx-parsing/`

## Implementation Strategy

This implementation will follow an incremental delivery approach, starting with the highest priority user story (Content Authors Can Write Documentation with XML Tags) to ensure an MVP that allows documentation to build successfully. Each user story will be implemented as a complete, independently testable increment.

## Dependencies

User stories are largely independent, but foundational tasks must be completed first. The priority order is:
1. US1 (P1): Content Authors Can Write Documentation with XML Tags
2. US2 (P1): Readers Can View Documentation with XML Examples Correctly
3. US3 (P2): Assessment and Quiz Sections Render Without Errors

## Parallel Execution Examples

Many tasks can be executed in parallel since they operate on different files:
- T006-P through T020-P can be executed simultaneously across different MDX files
- Each user story has multiple parallelizable tasks for different documentation files

---

## Phase 1: Setup

**Goal**: Prepare the environment and tools needed to systematically fix XML tags in MDX files

- [ ] T001 Set up development environment with Node.js 18+ and Docusaurus 3.x
- [ ] T002 Verify current build process fails with JSX parsing errors for XML tags
- [ ] T003 Create backup of docs/modules directory before making changes
- [ ] T004 Document current list of all .md/.mdx files in docs/modules for tracking

---

## Phase 2: Foundational Tasks

**Goal**: Establish tools and processes needed for all user stories

- [ ] T005 Create script or tool to identify raw XML tags in MDX files
- [ ] T006 [P] Identify all MDX files containing raw XML tags in docs/modules
- [ ] T007 [P] Identify all MDX files containing raw XML tags in docs/modules/assessment
- [ ] T008 Create test build command to verify fixes work correctly
- [ ] T009 Document the common XML tags that need fixing (inertial, link, joint, etc.)

---

## Phase 3: [US1] Content Authors Can Write Documentation with XML Tags (Priority: P1)

**Goal**: Enable content authors to write documentation with XML tags without encountering build errors

**Independent Test**: Content authors can write MDX files containing XML tags like `<inertial>` and the Docusaurus build completes successfully without JSX parsing errors.

- [ ] T010 [US1] Fix raw XML tags in inline text to use inline code formatting in basic module files
- [ ] T011 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 1
- [ ] T012 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 2
- [ ] T013 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 3
- [ ] T014 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 4
- [ ] T015 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 5
- [ ] T016 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 6
- [ ] T017 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 7
- [ ] T018 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 8
- [ ] T019 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 9
- [ ] T020 [P] [US1] Update XML code blocks to use proper ```xml fencing in module 10
- [ ] T021 [US1] Run build process to verify no JSX parsing errors remain
- [ ] T022 [US1] Verify acceptance scenario 1: Build completes successfully with XML tags in MDX files
- [ ] T023 [US1] Verify acceptance scenario 2: XML syntax highlighting works in code blocks

---

## Phase 4: [US2] Readers Can View Documentation with XML Examples Correctly (Priority: P1)

**Goal**: Ensure readers can view documentation containing XML examples that render correctly in the browser

**Independent Test**: Readers can view documentation pages containing XML examples and the XML tags are displayed correctly with proper syntax highlighting.

- [ ] T024 [US2] Verify XML examples render correctly in browser after US1 fixes
- [ ] T025 [US2] Check that inline XML code formatting displays properly in UI
- [ ] T026 [US2] Verify XML syntax highlighting works for fenced code blocks
- [ ] T027 [US2] Test that XML examples are readable and maintain original meaning
- [ ] T028 [US2] Verify acceptance scenario 1: XML tags are properly formatted and syntax-highlighted
- [ ] T029 [US2] Verify acceptance scenario 2: XML structure is clearly visible and readable

---

## Phase 5: [US3] Assessment and Quiz Sections Render Without Errors (Priority: P2)

**Goal**: Ensure assessment and quiz sections containing XML tags render correctly without JSX parsing errors

**Independent Test**: Assessment and quiz sections containing XML examples render correctly without build errors or runtime errors.

- [ ] T030 [US3] Identify all assessment/quiz MDX files that contain XML tags
- [ ] T031 [US3] Fix raw XML tags in inline text within assessment files
- [ ] T032 [US3] Update XML code blocks in assessment files to use proper ```xml fencing
- [ ] T033 [US3] Fix raw XML tags in inline text within quiz files
- [ ] T034 [US3] Update XML code blocks in quiz files to use proper ```xml fencing
- [ ] T035 [US3] Run build process to verify no JSX parsing errors in assessment/quiz sections
- [ ] T036 [US3] Verify acceptance scenario 1: Assessment content renders without JSX errors
- [ ] T037 [US3] Verify acceptance scenario 2: Quiz content with XML examples displays properly

---

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Final verification and quality assurance across all documentation

- [ ] T038 Run complete build process to ensure all fixes work together
- [ ] T039 Verify all success criteria are met (SC-001 through SC-004)
- [ ] T040 Perform visual inspection of key documentation pages with XML examples
- [ ] T041 Update any remaining XML tags that were missed in earlier phases
- [ ] T042 Document the process for future content authors to follow
- [ ] T043 Create a checklist for identifying and fixing XML tags in new documentation
- [ ] T044 Verify that no new errors were introduced during the fixing process
- [ ] T045 Update feature status to completed and prepare for review

## Outcome

- ✅ Impact: Created a comprehensive, actionable task list for fixing MDX JSX parsing errors in Docusaurus
- 🧪 Tests: Tasks organized by user story with independent test criteria for each
- 📁 Files: Created tasks.md with 45 specific, actionable tasks organized by priority
- 🔁 Next prompts: Ready for implementation using the generated tasks
- 🧠 Reflection: Tasks follow proper checklist format and are organized by user story for independent implementation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): 
- Next experiment (smallest change to try): 