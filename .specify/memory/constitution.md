<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- List of modified principles:
    - Accuracy through Primary Source Verification (removed)
    - Academic Clarity (removed)
    - Reproducibility (removed)
    - Scholarly Rigor (removed)
    - Spec-Driven Authoring (new)
    - Technical Accuracy (new)
    - Pedagogical Clarity (new)
    - Chunkable Knowledge (new)
    - Agent-Augmented Writing (new)
- Added sections:
    - Project Mission
    - Deliverables
    - Optional Bonus
- Removed sections:
    - Quality Standards & Constraints (replaced with more granular sections)
- Templates requiring updates:
    - .specify/templates/plan-template.md: ✅ updated
    - .specify/templates/spec-template.md: ✅ updated
    - .specify/templates/tasks-template.md: ✅ updated
- Follow-up TODOs: None
-->
# AI-Native Textbook: Physical AI & Humanoid Robotics Constitution

## Project Mission

- Produce a complete, high-quality, AI-native textbook using Docusaurus.
- Use Spec-Kit Plus and Claude Code to generate, review, structure, and refine all chapters.
- Create a book aligned with the Physical AI & Humanoid Robotics course.
- Ensure the book is deployment-ready on GitHub Pages.

## Core Principles

### Spec-Driven Authoring
All content generation must follow Spec-Kit Plus tasks, passes, reviews, and automated structure. No free-form writing; every deliverable follows a defined spec.

### Technical Accuracy
All robotics, AI, and software claims must reflect industry best practices. Engineering constraints, system limitations, and physical realities must be stated explicitly.

### Pedagogical Clarity
Write for students learning Physical AI: clear explanations, progressive complexity, code snippets, diagrams, and step-by-step workflows.

### Chunkable Knowledge
Each chapter must be structured into small, standalone semantic units optimized for RAG ingestion (Qdrant), embedding pipelines, and agentic retrieval.

### Agent-Augmented Writing
Claude Code Subagents and Agent Skills should be used to modularize tasks, automate reviews, improve clarity, and support bonus scoring in the hackathon.

## Standards

### Writing
- Use clean Markdown suitable for Docusaurus.
- Every chapter starts with: Summary, Learning Objectives, Prerequisites.
- Every chapter ends with: Key Takeaways, Glossary Terms, Review Questions.
- Avoid long paragraphs. Prefer short, high-signal blocks.
- No cross-chapter dependencies; each section must be self-contained.

### Formatting
- Use H2/H3/H4 headings consistently.
- Provide code blocks where needed (ROS 2, Python, Isaac Sim, Unity).
- Use tables for architectural comparisons or hardware specs.
- No inline HTML unless required by Docusaurus.

### Docusaurus
- Output must be placed in /docs with correct sidebar structure.
- All files must be markdown (.md or .mdx) and buildable without errors.
- Frontmatter must be correct and consistent across chapters.

### Verification
- Claude must run multi-pass reviews: structure pass → draft pass → rigor pass → optimization pass.
- All technical claims must be internally consistent with the course spec.

## Constraints

- The book must cover the entire Physical AI & Humanoid Robotics course outline.
- Minimum 20 chapters; maximum 50.
- Chapters must map directly to course modules (ROS → Simulation → Isaac → VLA → Capstone).
- All content must be generated through Spec-Kit Plus tasks, not manual writing.

## Deliverables

### Book
- Complete Docusaurus-ready markdown files for all chapters.
- Sidebar configuration for navigation.
- Frontmatter for each chapter.

### Deployment
- GitHub repository containing the Docusaurus project.
- GitHub Pages build + deployment pipeline (GitHub Actions).
- Live public link to book.

### RAG Integration
- Text structured for Qdrant embeddings.
- Clear chunk boundaries.
- Per-section context summaries to support RAG retrieval.

## Optional Bonus

- Claude Subagents for chapter writing, code generation, and reviews.
- Agent Skills integrated into the writing workflow.
- Personalization hooks (Beginner/Advanced variants).
- Section-level translation hooks (e.g., Urdu).

## Success Criteria

- Docusaurus site builds successfully without errors.
- GitHub Pages deployment is live and public.
- Book structure matches the official course specification.
- Content demonstrates technical and pedagogical rigor.
- Chapters are embedding-friendly and ideal for RAG pipelines.
- Spec-Kit Plus passes show completeness and consistency.
- Bonus: Agent Skills, personalization, or translation implemented.

## Governance

This Constitution supersedes all other practices and guidelines within the project. Amendments to this Constitution require thorough documentation, explicit approval from the project lead, and a clear migration plan if changes impact existing work. All contributions to the project must verify compliance with these principles and standards. Complexity must always be justified with clear rationale.

**Version**: 1.1.0 | **Ratified**: 2025-12-03 | **Last Amended**: 2025-12-04
