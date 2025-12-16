---
id: 004
title: Experimental Retrieval Rejection Record
stage: general
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: Aiman Syeda
command: /sp.phr
labels: ["rejection", "retrieval", "hybrid", "agentic", "precision", "policy"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - none
tests:
  - none
---

## Prompt

# PHR-021: Claude Re-Introduced Experimental Retrieval

## What Happened
Claude suggested hybrid retrievers after they were explicitly rejected earlier.

## Root Cause
No recorded rejection history existed.

## Resolution
Added explicit PHR banning experimental retrievers.

## Do Not Repeat
- Do NOT suggest hybrid or agentic retrievers
- Do NOT optimize recall at expense of precision

## Response snapshot

Acknowledged and recorded the rejection policy. This PHR now serves as permanent historical record that experimental retrieval methods (hybrid retrievers, agentic retrievers) have been explicitly rejected for this project. Future Claude sessions will have access to this decision in the prompt history.

## Outcome

- ✅ Impact: Created permanent record preventing re-introduction of rejected experimental retrieval approaches
- 🧪 Tests: none
- 📁 Files: PHR-004 created as policy enforcement artifact
- 🔁 Next prompts: Future retrieval discussions must check this PHR first; maintain precision-first approach
- 🧠 Reflection: Rejection history is as important as implementation history - recording "what NOT to do" prevents repeated exploration of dead ends

## Evaluation notes (flywheel)

- Failure modes observed: Claude re-suggested previously rejected approach (hybrid retrievers) due to lack of recorded rejection in conversation history
- Graders run and results (PASS/FAIL): N/A - policy record, not code
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): When asked about retrieval improvements, explicitly check history/prompts/ for rejection records before suggesting experimental approaches

## Policy Constraints (DO NOT VIOLATE)

**BANNED APPROACHES:**
1. ❌ Hybrid retrievers (semantic + keyword fusion)
2. ❌ Agentic retrievers (multi-step reasoning chains)
3. ❌ Optimizing recall at expense of precision
4. ❌ Experimental retrieval architectures without explicit user approval

**APPROVED APPROACH:**
- ✅ Simple semantic search with Qdrant
- ✅ Lowered score threshold (0.5) for lenient matching
- ✅ Strict textbook grounding in prompts
- ✅ Precision-first retrieval (better to return fewer relevant results than many irrelevant ones)

**RATIONALE:**
- Hybrid/agentic retrievers add complexity without proven benefit for this use case
- Current simple approach works: textbook content is well-structured, semantic search is sufficient
- Precision matters more than recall for educational content (wrong answers are worse than "I don't know")
- User explicitly rejected these approaches - respect that decision
