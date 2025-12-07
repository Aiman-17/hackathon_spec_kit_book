---
id: 002
title: Update Constitution for AI-Native Textbook
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-docusaurus-docs
user: unknown
command: /sp.constitution
labels: ["project-update", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

```text
project: "AI-Native Textbook: Physical AI & Humanoid Robotics"

mission:
  - "Produce a complete, high-quality, AI-native textbook using Docusaurus."
  - "Use Spec-Kit Plus and Claude Code to generate, review, structure, and refine all chapters."
  - "Create a book aligned with the Physical AI & Humanoid Robotics course."
  - "Ensure the book is deployment-ready on GitHub Pages."

principles:
  - name: "Spec-Driven Authoring"
    description: >
      All content generation must follow Spec-Kit Plus tasks, passes, reviews,
      and automated structure. No free-form writing; every deliverable follows a defined spec.

  - name: "Technical Accuracy"
    description: >
      All robotics, AI, and software claims must reflect industry best practices.
      Engineering constraints, system limitations, and physical realities must be stated explicitly.

  - name: "Pedagogical Clarity"
    description: >
      Write for students learning Physical AI: clear explanations, progressive complexity,
      code snippets, diagrams, and step-by-step workflows.

  - name: "Chunkable Knowledge"
    description: >
      Each chapter must be structured into small, standalone semantic units optimized
      for RAG ingestion (Qdrant), embedding pipelines, and agentic retrieval.

  - name: "Agent-Augmented Writing"
    description: >
      Claude Code Subagents and Agent Skills should be used to modularize tasks,
      automate reviews, improve clarity, and support bonus scoring in the hackathon.

standards:
  writing:
    - "Use clean Markdown suitable for Docusaurus."
    - "Every chapter starts with: Summary, Learning Objectives, Prerequisites."
    - "Every chapter ends with: Key Takeaways, Glossary Terms, Review Questions."
    - "Avoid long paragraphs. Prefer short, high-signal blocks."
    - "No cross-chapter dependencies; each section must be self-contained."

  formatting:
    - "Use H2/H3/H4 headings consistently."
    - "Provide code blocks where needed (ROS 2, Python, Isaac Sim, Unity)."
    - "Use tables for architectural comparisons or hardware specs."
    - "No inline HTML unless required by Docusaurus."

  docusaurus:
    - "Output must be placed in /docs with correct sidebar structure."
    - "All files must be markdown (.md or .mdx) and buildable without errors."
    - "Frontmatter must be correct and consistent across chapters."

  verification:
    - "Claude must run multi-pass reviews: structure pass → draft pass → rigor pass → optimization pass."
    - "All technical claims must be internally consistent with the course spec."

constraints:
  - "The book must cover the entire Physical AI & Humanoid Robotics course outline."
  - "Minimum 20 chapters; maximum 50."
  - "Chapters must map directly to course modules (ROS → Simulation → Isaac → VLA → Capstone)."
  - "All content must be generated through Spec-Kit Plus tasks, not manual writing."

deliverables:
  book:
    - "Complete Docusaurus-ready markdown files for all chapters."
    - "Sidebar configuration for navigation."
    - "Frontmatter for each chapter."

  deployment:
    - "GitHub repository containing the Docusaurus project."
    - "GitHub Pages build + deployment pipeline (GitHub Actions)."
    - "Live public link to book."

  rag_integration:
    - "Text structured for Qdrant embeddings."
    - "Clear chunk boundaries."
    - "Per-section context summaries to support RAG retrieval."

  optional_bonus:
    - "Claude Subagents for chapter writing, code generation, and reviews."
    - "Agent Skills integrated into the writing workflow."
    - "Personalization hooks (Beginner/Advanced variants)."
    - "Section-level translation hooks (e.g., Urdu)."

success_criteria:
  - "Docusaurus site builds successfully without errors."
  - "GitHub Pages deployment is live and public."
  - "Book structure matches the official course specification."
  - "Content demonstrates technical and pedagogical rigor."
  - "Chapters are embedding-friendly and ideal for RAG pipelines."
  - "Spec-Kit Plus passes show completeness and consistency."
  - "Bonus: Agent Skills, personalization, or translation implemented."
  add these changes on constitution file you have been created
```

## Response snapshot

The project constitution for "AI-Native Textbook: Physical AI & Humanoid Robotics" has been successfully updated.

**New Version**: 1.1.0
**Version Rationale**: This is a minor version bump due to significant updates including a new project mission, new principles, updated standards, added constraints, deliverables, and success criteria.

**Manual Follow-up Files**: None. Existing templates (`plan-template.md`, `spec-template.md`, `tasks-template.md`) do not require direct updates, but adherence to the new constitution's principles is expected in future artifacts generated from them.

**Suggested Commit Message**: `docs: update constitution v1.1.0 for AI-Native Textbook project`

## Outcome

- ✅ Impact: Project constitution updated with new details.
- 🧪 Tests: No tests were run for this task.
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: User to proceed with project development based on updated constitution.
- 🧠 Reflection: Successfully updated the constitution based on user input, including a minor version bump due to significant changes.

## Evaluation notes (flywheel)

- Failure modes observed: `create-phr.sh` script not found, falling back to agent-native tool.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure `create-phr.sh` or a similar script is available and functional for shell-based PHR creation.
