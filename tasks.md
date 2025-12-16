---
description: "Production RAG Chatbot Stabilization Tasks"
---

# Tasks: RAG Chatbot Stabilization

**Context**: Stabilize existing RAG chatbot for production deployment on Hugging Face + GitHub Pages

**Constraints**:
- Use best free OpenRouter model (no credits)
- No web browsing
- No experimental refactors
- Minimal, surgical changes only
- Deployment-safe code

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which phase/story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Critical Backend Fixes (Priority: P1) 🎯 CRITICAL

**Goal**: Fix OpenRouter 402 errors and prevent API crashes from credit exhaustion

**Independent Test**: Backend handles credit exhaustion gracefully with 200 status and error message

### OpenRouter Credit Management

- [X] T001 Switch to best free OpenRouter model in backend/src/api/routers/chat.py:186
- [X] T002 Add graceful credit exhaustion handling in backend/src/api/routers/chat.py:generate_rag_response
- [X] T003 Update error response to show user-friendly message instead of HTTP 500 in backend/src/api/routers/chat.py:197-202
- [X] T004 [P] Add credit status check endpoint in backend/src/api/routers/chat.py
- [ ] T005 Test credit exhaustion scenario end-to-end

**Checkpoint**: Backend never crashes with HTTP 500 on credit exhaustion

---

## Phase 2: RAG Answer Precision (Priority: P1) 🎯 CRITICAL

**Goal**: Ensure RAG answers all textbook queries correctly without false rejections or hallucinations

**Independent Test**: Query "What is a vector?" returns textbook-based answer if chunks exist

### System Prompt Refinement

- [X] T006 Update system prompt to treat textbook as single source of truth in backend/src/api/routers/chat.py:146-165
- [X] T007 Add logic to never reject queries if chunks exist in backend/src/api/routers/chat.py:112-122
- [X] T008 Implement weak chunk handling with cautious answers in backend/src/api/routers/chat.py:234-250
- [X] T009 Set maximum answer length constraints (6-7 lines for definitions) in backend/src/api/routers/chat.py:184-192
- [X] T010 Remove over-quoting and long citations from responses in backend/src/api/routers/chat.py:167-181

### Hallucination Control

- [X] T011 Add explicit "no external facts" constraint to prompt in backend/src/api/routers/chat.py:146-165
- [X] T012 Enforce reasoning anchored to retrieved chunks in backend/src/api/routers/chat.py:175-180
- [ ] T013 Test with sample queries: "What is a vector?", "Explain linear algebra in robotics", "How is NVIDIA used in robotics?"

**Checkpoint**: RAG provides accurate textbook-based answers without hallucination or false rejections

---

## Phase 3: Personalization UI (Priority: P2)

**Goal**: Add visible personalization controls to frontend (no authentication required)

**Independent Test**: User can select role and language preference, preferences are sent with queries

### Identify Existing Backend Logic

- [X] T014 Audit backend/src/services/personalization_service.py for personalization logic
- [X] T015 Audit backend/src/api/routers/personalization.py for available endpoints
- [X] T016 Document how preferences should be passed to backend in queries

### Frontend Personalization UI

- [X] T017 Create PersonalizationPanel component in frontend/src/components/PersonalizationPanel/index.tsx
- [X] T018 Add role selector (student/beginner/advanced) to PersonalizationPanel in frontend/src/components/PersonalizationPanel/index.tsx
- [X] T019 Add language preference toggle (English/Urdu) to PersonalizationPanel in frontend/src/components/PersonalizationPanel/index.tsx
- [X] T020 Store preferences in localStorage (no auth required) in frontend/src/components/PersonalizationPanel/index.tsx
- [X] T021 Integrate PersonalizationPanel into main UI in frontend/src/theme/Root.tsx or frontend/src/components/ChatWidget/index.tsx

### Backend Personalization Integration

- [X] T022 Update ChatRequest model to accept personalization params in backend/src/api/routers/chat.py:33-40
- [X] T023 Modify generate_rag_response to adjust tone based on user level in backend/src/api/routers/chat.py:112-202
- [ ] T024 Test personalized responses maintain grounding rules (no hallucination)

**Checkpoint**: Users can set preferences, preferences influence answer depth/tone without adding hallucination

---

## Phase 4: Urdu Translation (Priority: P2)

**Goal**: Add controlled Urdu translation for final answers only (no translation in retrieval)

**Independent Test**: Selecting Urdu translates answers correctly while keeping retrieval in English

### Translation Infrastructure

- [X] T025 Verify translation endpoint exists in backend/src/api/routers/translate.py
- [X] T026 Ensure translation applies ONLY to final answers in backend/src/api/routers/translate.py
- [X] T027 Preserve technical terms in Urdu output in backend/src/api/routers/translate.py
- [X] T028 Handle weak content cautiously in Urdu (no creative hallucination) in backend/src/api/routers/translate.py

### Frontend Translation Toggle

- [X] T029 Add Urdu toggle to PersonalizationPanel in frontend/src/components/PersonalizationPanel/index.tsx
- [X] T030 Send language preference with chat queries in frontend/src/components/ChatWidget/index.tsx
- [X] T031 Display translated answers when Urdu is selected in frontend/src/components/ChatWidget/index.tsx
- [ ] T032 Test: Urdu translation does not affect retrieval quality

**Checkpoint**: Urdu translation works correctly for answers without affecting retrieval or adding hallucinations

---

## Phase 5: Mobile-First UX (Priority: P2)

**Goal**: Improve mobile navigation with slide-in sidebar for chapters/sections

**Independent Test**: Mobile menu opens/closes smoothly, displays modules/chapters/sections

### Mobile Navigation

- [ ] T033 Create MobileNavSidebar component in frontend/src/components/MobileNavSidebar/index.tsx
- [ ] T034 Add hamburger menu button for mobile in frontend/src/components/MobileNavSidebar/index.tsx
- [ ] T035 Implement slide-in animation from left in frontend/src/components/MobileNavSidebar/index.tsx
- [ ] T036 List modules, chapters, and sections in sidebar in frontend/src/components/MobileNavSidebar/index.tsx
- [ ] T037 Integrate MobileNavSidebar into main UI in frontend/src/theme/Root.tsx
- [ ] T038 Test mobile navigation on responsive breakpoints

**Checkpoint**: Mobile users can navigate textbook structure via slide-in sidebar

---

## Phase 6: Deployment & Push (Priority: P1) 🎯 CRITICAL

**Goal**: Deploy to GitHub Pages (frontend) and Hugging Face Spaces (backend) without breaking

**Independent Test**: Chat works end-to-end in production, errors handled gracefully, no blank UI

### Frontend Deployment

- [X] T039 Configure frontend build with correct GitHub Pages base path in frontend/package.json or frontend/vite.config.js
- [X] T040 Remove `process` references from browser code in frontend/src/**
- [X] T041 Ensure environment-safe config in frontend/src/config or frontend/.env
- [X] T042 Build frontend with npm run build in frontend/
- [ ] T043 Push build artifacts to GitHub Pages (Manual: Set GIT_USER env var OR use: cd frontend && GIT_USER=Aiman-17 npm run deploy OR setup GitHub Actions)

### Backend Deployment

- [X] T044 Verify no local dependency assumptions in backend/requirements.txt
- [X] T045 Ensure all failures are handled gracefully (no unhandled exceptions) in backend/src/api/routers/chat.py
- [ ] T046 Test backend on Hugging Face Spaces environment
- [X] T047 Verify CORS configuration includes production URLs in backend/src/config.py:127-135

### End-to-End Validation

- [ ] T048 Test chat works end-to-end in production
- [ ] T049 Verify backend errors are handled gracefully (no 500s for known errors)
- [ ] T050 Verify no blank UI states in production frontend
- [ ] T051 Commit all changes with clear messages

**Checkpoint**: Production deployment is stable, chat works end-to-end, errors are graceful

---

## Phase 7: Polish & Verification (Priority: P3)

**Goal**: Final cleanup and documentation

**Independent Test**: All deployment targets work, documentation is up-to-date

- [ ] T052 [P] Update README with deployment instructions
- [ ] T053 [P] Document free model selection and credit limits
- [ ] T054 [P] Add troubleshooting guide for common errors
- [ ] T055 Final verification of all phases

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Critical Backend Fixes)**: No dependencies - START IMMEDIATELY
- **Phase 2 (RAG Answer Precision)**: Can run parallel with Phase 1 (different concerns)
- **Phase 3 (Personalization UI)**: Depends on Phase 1 completion (backend stability)
- **Phase 4 (Urdu Translation)**: Depends on Phase 3 completion (personalization integration)
- **Phase 5 (Mobile UX)**: Can run parallel with Phase 3/4 (UI-only)
- **Phase 6 (Deployment)**: Depends on Phases 1, 2 completion (CRITICAL fixes)
- **Phase 7 (Polish)**: Depends on Phase 6 completion (deployment verified)

### Critical Path

1. **Phase 1 + Phase 2** (parallel) → Backend is stable and accurate
2. **Phase 6** → Deployment works
3. **Phase 3 + Phase 5** (parallel) → UX improvements
4. **Phase 4** → Translation
5. **Phase 7** → Final polish

### Parallel Opportunities

- **Phase 1 and Phase 2** can run in parallel (different files/concerns)
- **Phase 3 and Phase 5** can run in parallel (different UI components)
- Within phases, tasks marked **[P]** can run in parallel

---

## Implementation Strategy

### MVP First (Phases 1, 2, 6)

1. Complete Phase 1: Fix OpenRouter 402 errors
2. Complete Phase 2: Fix RAG answer precision
3. Complete Phase 6: Deploy to production
4. **STOP and VALIDATE**: Test end-to-end in production

### Incremental Delivery

1. Add Phase 3: Personalization UI
2. Add Phase 4: Urdu translation
3. Add Phase 5: Mobile navigation
4. Complete Phase 7: Final polish

### Minimal Changes

- No experimental refactors
- No new frameworks or build tools
- Surgical fixes only
- Clear commit messages
- No breaking changes

---

## Notes

- Use best free OpenRouter model (e.g., "meta-llama/llama-3.1-8b-instruct:free" or similar)
- Graceful degradation on credit exhaustion (never crash)
- RAG grounding is non-negotiable (no hallucinations)
- Frontend changes must be GitHub Pages compatible
- Backend changes must be Hugging Face Spaces compatible
- All error paths must be handled
- Mobile-first responsive design
- No authentication/user accounts required for personalization
