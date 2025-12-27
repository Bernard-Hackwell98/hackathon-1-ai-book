---
id: 1
title: Create AI Book Constitution
stage: constitution
date: 2025-12-26
model: Qwen Code
feature: ai-spec-driven-book
branch: main
user: Hammad97
command: /sp.constitution
labels: ["constitution", "ai-book", "spec-first", "governance"]
links: 
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Create AI Book Constitution

## Prompt

```
$ARGUMENTS
```

## Response

```
Project: AI-Spec-Driven Open-Source Book with Integrated RAG Chatbot

Objective:
Author a technical textbook using Docusaurus via Claude Code and Spec-Kit Plus, deploy to GitHub Pages, and embed a RAG chatbot that answers questions strictly from the book content or user-selected text.

Audience:
Computer science students, AI engineers, researchers.

Core Principles:
- Spec-first, AI-native development
- Verifiable accuracy via primary sources
- Clarity for technical readers
- Reproducibility and rigor
- Zero plagiarism tolerance

Book Standards:
- Format: Docusaurus (MD/MDX)
- Clear chapter structure, objectives, summaries
- Code examples must be runnable and secure
- Citations: APA style
- Sources: ≥60% peer-reviewed or official docs
- Writing level: Flesch-Kincaid 11–13

AI Authoring Constraints:
- No hallucinated facts or citations
- All claims traceable
- Explicitly flag uncertain information
- Follow Spec-Kit Plus specs strictly

RAG Chatbot Standards:
- Backend: FastAPI
- LLM: OpenAI Agents / ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- DB: Neon Serverless Postgres
- Retrieval only from indexed book content
- Selected-text mode must answer only from user-highlighted text
- If not found, respond: "Answer not contained in the provided text."

Deployment & Integration:
- GitHub Pages with explicit trailingSlash config
- Embedded chatbot in Docusaurus pages
- Single monorepo with clear service separation

Success Criteria:
- Book publicly deployed
- RAG answers fully grounded in content
- All claims verified and cited
- Reproducible build and deployment
```

## Outcome

Successfully created the project constitution for the AI-Spec-Driven Open-Source Book with Integrated RAG Chatbot project. The constitution includes core principles, book standards, AI authoring constraints, RAG chatbot standards, deployment & integration guidelines, and governance procedures.

## Evaluation

The constitution was created with version 1.0.0, incorporating all specified requirements. The document includes specific, testable principles rather than vague language, and establishes clear governance procedures for future amendments. Files requiring manual follow-up have been identified: plan-template.md, spec-template.md, and tasks-template.md need alignment with the new constitution.