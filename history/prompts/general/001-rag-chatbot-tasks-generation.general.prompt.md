---
id: 001
title: RAG Chatbot Stabilization Tasks Generation
stage: tasks
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: ACS
command: /sp.tasks
labels: ["tasks", "rag", "production", "deployment", "stabilization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - tasks.md
tests:
  - null
---

## Prompt

```text
 My Open router credit Limit is Low dont have credits so use best free model. You are operating as a Production RAG Systems Engineer.
This is a deployment-bound project with a strict deadline.
Do NOT browse the web.
Do NOT explain basics.
Do NOT refactor unless explicitly required.

OBJECTIVE
Stabilize the existing RAG chatbot so that:
1. It answers ALL queries strictly from the textbook content
2. It never hallucinates or rejects valid textbook topics
3. It operates within OpenRouter credit limits
4. It deploys cleanly on Hugging Face + GitHub Pages
5. Frontend UX supports mobile navigation, personalization, and Urdu translation

-------------------------
PHASE 1 — CRITICAL BACKEND FIXES (NO NEW FEATURES)
-------------------------

1. Fix OpenRouter 402 (Payment Required) errors:
   - Switch to free best Model.
   - Fail gracefully if credits are insufficient ("⚠️ The AI service is temporarily unavailable due to usage limits.\n\n"
      "The system retrieved relevant textbook sections, but cannot generate a full response right now.\n\n"
      "Please try again later.", not a 500)
   - Ensure the API NEVER crashes due to credit limits.
   - NEVER crash the API with HTTP 500 for credit exhaustion

2. RAG Answer Precision Rules (MANDATORY):
   - If query matches textbook domains (e.g.,Headings, Example with Code, Any definition in textbook chapters, all information provided in textbook):
     → Do not give irrelevant response if query is from whole textbook , every chapters ,Titles, Example codes and anything in the textbook.
     → Answer ONLY from retrieved chunks
     → Do NOT say "irrelevant" if chunks exist
   - If chunks exist but are weak:
     → Provide a cautious answer with explicit limitation
   - If no chunks exist:
     → Say content is not covered in the textbook
   - Maximum answer length:
     → Definitions: 6–7 lines
     → Explanations: concise, no verbosity

3. Hallucination Control:
   - No external facts
   - No inferred explanations beyond retrieved text
   - Reasoning must stay anchored to retrieved chunks
   - No over-quoting, no long citations

-------------------------
PHASE 2 — RAG PROMPT HARDENING
-------------------------

Refine the system prompt so that the model:
- Treats the textbook as the single source of truth
- Answers questions like:
  "What is a vector?"
  "Explain linear algebra in robotics"
  "How is NVIDIA used in robotics?"
  WITHOUT rejecting them if covered in retrieved chunks
- Uses structured reasoning but concise output
- Never exceeds the requested scope

-------------------------
PHASE 3 — PERSONALIZATION (BACKEND + FRONTEND)
-------------------------

Status:
- Backend personalization logic already exists
- Frontend has NO visible personalization component

Tasks:
1. Identify existing personalization logic/files

2.Ensure preferences Influence:
   - Answer tone
   - Language output
   - Conciseness level
Add frontend UI:

User profile panel (role: student / beginner / advanced)

Language preference (English / Urdu)

Pass personalization flags to backend with each query.

Backend must:

Adjust answer depth based on user level

Keep same grounding rules (no hallucination)

DO NOT add authentication.
DO NOT add user accounts.

-------------------------
PHASE 4 — URDU TRANSLATION (CONTROLLED)
-------------------------

Rules:
- Translation must apply ONLY to final answers
- Retrieval, embeddings, and search remain in English
- Urdu output must:
  - Be clear
  - Be neutral academic Urdu
  - Preserve technical terms where necessary
- If Urdu is selected but content is weak:
  → Respond cautiously, not creatively
  →Add toggle in UI (no auto-translate).


-------------------------
PHASE 5 — FRONTEND UX (MOBILE-FIRST)
-------------------------

1. Mobile Navigation:
   - Replace current menu with a slide-in sidebar
   Hamburger opens left slide-bar

Sidebar lists:

Modules

Chapters

Section


-------------------------
PHASE 6 — DEPLOYMENT & PUSH
-------------------------

1. Ensure frontend build works on GitHub Pages:
   - No `process` references in browser code
   - Environment-safe config
2. Ensure backend works on Hugging Face Spaces:
   - No local dependency assumptions
   - All failures handled gracefully
3. Push finalized frontend changes to GitHub Pages
4. Do NOT introduce new build tools or frameworks

-------------------------
DELIVERABLES
-------------------------

- Minimal, surgical code changes only
- No token-wasting refactors
- No exploratory experimentation
- Deployment-safe final state
- Clear commit messages
- No breaking changes

If something is ambiguous:
- Ask ONE precise clarification question only if unavoidable
- Otherwise, make a conservative engineering decision

Tasks

Build frontend with correct base path for GitHub Pages.

Push build artifacts only.

Verify:

Chat works end-to-end

Backend errors are handled gracefully

No blank UI states

Non-Negotiable Constraints

❌ No web browsing

❌ No experimental refactors

❌ No verbose explanations

✅ Deterministic fixes only

✅ Production-ready code

✅ Minimal token usage
```

## Response snapshot

Generated comprehensive tasks.md with 55 production-ready tasks organized into 7 phases:

**Phase 1: Critical Backend Fixes (5 tasks)**
- Switch to best free OpenRouter model
- Add graceful credit exhaustion handling
- Prevent HTTP 500 errors on credit limits
- Add credit status check endpoint

**Phase 2: RAG Answer Precision (8 tasks)**
- Update system prompt for textbook-only grounding
- Remove false rejection logic
- Implement weak chunk handling
- Set answer length constraints
- Enforce hallucination control

**Phase 3: Personalization UI (11 tasks)**
- Audit existing backend personalization logic
- Create PersonalizationPanel component
- Add role selector and language preference
- Store preferences in localStorage (no auth)
- Integrate preferences with chat queries

**Phase 4: Urdu Translation (8 tasks)**
- Verify translation endpoint
- Apply translation only to final answers
- Preserve technical terms
- Add Urdu toggle to UI

**Phase 5: Mobile-First UX (6 tasks)**
- Create MobileNavSidebar component
- Add hamburger menu
- Implement slide-in animation
- List modules/chapters/sections

**Phase 6: Deployment & Push (13 tasks)**
- Configure GitHub Pages base path
- Remove browser-incompatible code
- Verify Hugging Face Spaces compatibility
- Test end-to-end in production

**Phase 7: Polish (4 tasks)**
- Update documentation
- Document free model selection
- Add troubleshooting guide

All tasks follow strict checklist format with IDs, file paths, and clear descriptions.

## Outcome

- ✅ Impact: Production-ready task list for RAG chatbot stabilization with clear execution path
- 🧪 Tests: No test files (production stabilization focused on deployment)
- 📁 Files: tasks.md created at project root
- 🔁 Next prompts: Execute Phase 1 tasks first (critical backend fixes), then Phase 2 (RAG precision)
- 🧠 Reflection: Task list prioritizes critical fixes (Phases 1-2) before UX enhancements, enabling incremental deployment

## Evaluation notes (flywheel)

- Failure modes observed: None - tasks.md generated successfully without formal spec/plan documents
- Graders run and results (PASS/FAIL): Format validation PASS (all tasks have checkboxes, IDs, file paths)
- Prompt variant (if applicable): Direct task generation from user requirements without /sp.specify prerequisite
- Next experiment (smallest change to try): User should execute Phase 1 tasks (T001-T005) to stabilize OpenRouter integration
