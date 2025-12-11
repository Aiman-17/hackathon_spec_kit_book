---
id: 001
title: Cross-artifact analysis prerequisite check
stage: general
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: ACS
command: /sp.analyze
labels: ["analysis", "prerequisites", "workflow"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
tests:
---

## Prompt

/sp.analyze

## Response snapshot

Attempted to run cross-artifact analysis but discovered missing prerequisites. The feature directory structure has not been initialized - no spec.md, plan.md, or tasks.md exist yet.

**Finding:** User needs to complete the specification workflow first:
1. Run /sp.specify to create feature specification
2. Run /sp.plan to generate architectural plan
3. Run /sp.tasks to create task breakdown
4. Then run /sp.analyze for cross-artifact validation

**Status:** Analysis blocked - prerequisite check failed
**Location:** No feature directory at specs/main
**Recommendation:** Start with /sp.specify to begin the Spec-Driven Development workflow

## Outcome

- ✅ Impact: Identified workflow prerequisite gap; guided user to correct entry point
- 🧪 Tests: N/A - no implementation
- 📁 Files: None created
- 🔁 Next prompts: /sp.specify to start feature specification
- 🧠 Reflection: Successfully prevented analysis on non-existent artifacts; clear guidance provided

## Evaluation notes (flywheel)

- Failure modes observed: User attempted to run analysis before specification phase
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard /sp.analyze invocation
- Next experiment (smallest change to try): User should follow recommended workflow sequence
