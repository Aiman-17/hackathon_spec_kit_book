---
id: 001
title: Create Constitution for AI-Native Software Development
stage: constitution
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-docusaurus-docs
user: unknown
command: /sp.constitution
labels: ["project-setup", "governance"]
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
project: "Research Paper on AI-Native Software Development"

principles:
  - name: "Accuracy through Primary Source Verification"
    description: >
      All claims must be based on verifiable primary sources. Secondary commentary
      must be explicitly labeled as such.

  - name: "Academic Clarity"
    description: >
      Writing must be optimized for an academic audience with a computer science background.
      Explanations must be concise, technically rigorous, and free from ambiguity.

  - name: "Reproducibility"
    description: >
      All assertions, results, and conceptual frameworks must be citeable and reproducible.
      No unverifiable claims are allowed.

  - name: "Scholarly Rigor"
    description: >
      Peer-reviewed sources are preferred; all reasoning must follow scientific methodology
      (definitions → assumptions → method → evidence → conclusion).

standards:
  citations:
    format: "APA 7th Edition"
    requirements:
      - "All factual claims must be accompanied by citations."
      - "Direct quotes must include page numbers when available."
      - "Minimum 50% of all sources must be peer-reviewed journal or conference papers."
      - "Minimum total sources: 15."

  writing_clarity:
    metric: "Flesch-Kincaid Grade Level 10–12"
    rules:
      - "Avoid unnecessarily complex wording."
      - "Sentences should be structurally simple and direct."
      - "Define all technical terms at first use."

  plagiarism:
    tolerance: "0%"
    requirements:
      - "All content must be original."
      - "All borrowed ideas must be cited."
      - "Claude must rewrite any template or reused phrasing."

constraints:
  word_count:
    minimum: 5000
    maximum: 7000

  format:
    output_type: "PDF"
    requirements:
      - "Final output must include embedded APA citations."
      - "Bibliography must be included as references section."

success_criteria:
  - "All factual claims passed source verification."
  - "Zero plagiarism detected by external checking tools."
  - "Paper meets 5,000–7,000 word requirement."
  - "Minimum of 15 sources included; at least 50% peer-reviewed."
  - "APA citation formatting validated."
  - "Writing meets Flesch-Kincaid Grade 10–12 readability score."
  - "Passes independent fact-checking review."
```

## Response snapshot

The project constitution for "Research Paper on AI-Native Software Development" has been successfully created.

**New Version**: 1.0.0
**Version Rationale**: This is the initial version of the project constitution.

**Manual Follow-up Files**: None. Existing templates (`plan-template.md`, `spec-template.md`, `tasks-template.md`) do not require direct updates, but adherence to the new constitution's principles is expected in future artifacts generated from them.

**Suggested Commit Message**: `docs: create constitution v1.0.0 for Research Paper on AI-Native Software Development`

## Outcome

- ✅ Impact: Project constitution created and updated.
- 🧪 Tests: No tests were run for this task.
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: User to proceed with project development.
- 🧠 Reflection: Successfully created the constitution based on user input and template.

## Evaluation notes (flywheel)

- Failure modes observed: `create-phr.sh` script not found, falling back to agent-native tool.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure `create-phr.sh` or a similar script is available and functional for shell-based PHR creation.
