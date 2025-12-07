# Implementation Tasks: AI-Native Textbook & RAG System

**Feature**: AI-Native Textbook & RAG System for Physical AI & Humanoid Robotics
**Branch**: `001-ai-robotics-textbook`
**Date**: 2025-12-04

## Overview

This document defines all implementation tasks for building the AI-Native Textbook with RAG chatbot. Tasks are organized by user story to enable independent implementation and testing.

**Tech Stack**:
- Frontend: Docusaurus 3.x, React 18, TypeScript, Better-Auth
- Backend: Python 3.11+, FastAPI, OpenAI SDK, Qdrant Client, psycopg2 (Neon Postgres)
- Deployment: GitHub Pages (frontend), Render/Railway/HuggingFace Spaces (backend)

**User Stories** (from spec.md):
- **US1 (P1)**: Textbook Content Generation & Deployment
- **US2 (P1)**: RAG Chatbot Integration
- **US3 (P2)**: Personalization & Urdu Translation
- **US4 (P3)**: Advanced Agent Intelligence (Bonus)

---

## Task Format Legend

```text
- [ ] [TaskID] [P?] [Story?] Description with file path
```

- **TaskID**: Sequential number (T001, T002...)
- **[P]**: Parallelizable (can run concurrently with other [P] tasks)
- **[Story]**: User story label ([US1], [US2], [US3], [US4])
- **File path**: Exact location in project structure

---

## Phase 1: Project Setup & Infrastructure

**Goal**: Initialize project structure, configure tooling, set up deployment pipelines

**Dependencies**: None (starting point)

### Tasks

- [X] T001 Initialize Docusaurus project in frontend/ directory with TypeScript template
- [X] T002 [P] Initialize Python FastAPI project in backend/ directory with src/ structure
- [X] T003 [P] Create .github/workflows/deploy.yml for GitHub Pages deployment
- [X] T004 [P] Create backend/.env.example with required environment variables (QDRANT_API_KEY, QDRANT_URL, OPENAI_API_KEY, NEON_DB_URL, BETTER_AUTH_SECRET, REDIS_URL)
- [X] T005 [P] Create frontend/package.json with dependencies: docusaurus@3.x, react@18, better-auth, axios
- [X] T006 [P] Create backend/requirements.txt with dependencies: fastapi, uvicorn, qdrant-client, openai, psycopg2-binary, pydantic, redis, python-dotenv
- [X] T007 Configure Docusaurus in frontend/docusaurus.config.js with site metadata, GitHub Pages baseUrl, futuristic theme config
- [X] T008 [P] Create frontend/src/theme/custom.css with futuristic robotic UI styles (Navy #0A1929, Cyan #00E5FF, Purple #B388FF)
- [X] T009 [P] Create frontend/sidebars.js with 4 module structure (module-1 through module-4)
- [X] T010 [P] Create backend/src/config.py to load environment variables using Pydantic BaseSettings
- [X] T011 [P] Create backend/src/main.py with FastAPI app initialization, CORS middleware, and health check endpoint
- [X] T012 Create scripts/generate-chapters.py for AI-powered chapter generation
- [X] T013 [P] Create scripts/validate-content.py for 15-criteria content quality checker
- [X] T014 [P] Create scripts/ingest-embeddings.py for Qdrant ingestion pipeline
- [X] T015 Set up GitHub repository and connect to GitHub Pages in repository settings

---

## Phase 2: Foundational Infrastructure (Blocking Prerequisites)

**Goal**: Implement shared services needed by multiple user stories

**Dependencies**: Phase 1 complete

**Independent Test**: Database connections succeed, embedding pipeline processes sample text, API returns health check

### Tasks

- [X] T016 Create Neon Postgres database tables in backend/src/models/schema.sql (users, user_profiles, chat_history)
- [X] T017 [P] Implement backend/src/services/postgres_manager.py with asyncpg connection pool and table creation
- [X] T018 [P] Implement backend/src/services/qdrant_manager.py with collection creation (book_chapters, 1536 dims, COSINE)
- [X] T019 [P] Implement backend/src/services/embedding_pipeline.py with OpenAI text-embedding-3-small integration
- [X] T020 [P] Implement backend/src/services/document_ingestion.py with semantic chunking by H2/H3 headers (500-1000 token overlap)
- [X] T021 [P] Create backend/src/models/user.py with User and UserProfile Pydantic models
- [X] T022 [P] Create backend/src/models/chat.py with ChatMessage Pydantic model
- [X] T023 [P] Create backend/src/models/chapter.py with ChapterChunk Pydantic model
- [X] T024 [P] Implement backend/src/api/middleware.py with CORS, rate limiting (100 req/min), and error handling
- [X] T025 Initialize Qdrant collection via scripts/ingest-embeddings.py --init-collection flag
- [X] T026 Run database migrations via backend/src/services/postgres_manager.py --migrate flag
- [X] T027 Test FastAPI server startup with uvicorn backend.src.main:app --reload and verify health endpoint

---

## Phase 3: User Story 1 - Textbook Content Generation & Deployment (P1)

**Goal**: Generate 25 chapters across 4 modules and deploy to GitHub Pages

**Why P1**: Core deliverable, foundational for all other stories

**Dependencies**: Phase 2 complete (need database and embedding pipeline for RAG ingestion)

**Independent Test**:
- ✅ Docusaurus site builds successfully with zero errors (`npm run build`)
- ✅ GitHub Pages deployment is live at https://[username].github.io/[repo]
- ✅ All 25 chapters accessible with correct frontmatter and formatting
- ✅ Content passes 15-criteria quality checklist (≥23 of 25 chapters)

### Content Generation Tasks

- [ ] T028 [US1] Parse course outline from docs-source/course-outline.docx and extract 4 modules with 25 chapter titles
- [ ] T029 [US1] Generate Module 1 chapters (01-06) in frontend/docs/module-1/ using scripts/generate-chapters.py with chapter template
- [ ] T030 [P] [US1] Generate Module 2 chapters (07-12) in frontend/docs/module-2/ using scripts/generate-chapters.py
- [ ] T031 [P] [US1] Generate Module 3 chapters (13-19) in frontend/docs/module-3/ using scripts/generate-chapters.py
- [ ] T032 [P] [US1] Generate Module 4 chapters (20-25) in frontend/docs/module-4/ using scripts/generate-chapters.py
- [ ] T033 [US1] Validate all 25 chapters with scripts/validate-content.py --all --threshold 90 to ensure ≥23 chapters pass
- [ ] T034 [US1] Generate frontend/docs/intro.md as homepage with hero section and module overview
- [ ] T035 [P] [US1] Add Docusaurus frontmatter to all chapters (title, sidebar_position, tags, description)

### UI & Theme Tasks

- [ ] T036 [P] [US1] Implement frontend/src/components/HeroSection.tsx with futuristic 3D hero for homepage
- [ ] T037 [P] [US1] Implement frontend/src/theme/Footer/index.tsx with course information and social links
- [ ] T038 [P] [US1] Add frontend/static/fonts/ with Orbitron and Fira Code fonts for futuristic theme
- [ ] T039 [P] [US1] Add frontend/static/img/ with robotics-themed icons, logos, and placeholder diagrams

### Deployment Tasks

- [ ] T040 [US1] Configure GitHub Actions workflow in .github/workflows/deploy.yml to trigger on push to main
- [ ] T041 [US1] Test local Docusaurus build with `npm run build` and verify zero errors/warnings
- [ ] T042 [US1] Commit all changes and push to main branch to trigger GitHub Actions deployment
- [ ] T043 [US1] Verify GitHub Pages deployment at https://[username].github.io/[repo] is live
- [ ] T044 [US1] Test mobile responsiveness on 320px, 768px, 1024px viewports

### Embedding Ingestion Tasks

- [ ] T045 [US1] Run scripts/ingest-embeddings.py --ingest-all to chunk and embed all 25 chapters into Qdrant
- [ ] T046 [US1] Verify Qdrant collection contains ≥100 chunks via Qdrant dashboard or API query
- [ ] T047 [US1] Validate chunk size distribution (≥80% of chunks in 500-1500 token range) with ingestion report

---

## Phase 4: User Story 2 - RAG Chatbot Integration (P1)

**Goal**: Embed functional RAG chatbot widget with citations and selected-text mode

**Why P1**: Core AI-native feature, primary project goal

**Dependencies**: US1 complete (need chapters embedded in Qdrant)

**Independent Test**:
- ✅ Chatbot widget renders on every chapter page
- ✅ Test query "What is ROS 2?" returns answer + citation from correct chapter
- ✅ Selected text mode: Select "ROS 2 nodes communicate via topics" → ask "How does this work?" → response references "topics"
- ✅ 8 of 10 test queries return answer + citation (≥80% success rate per SC-008)
- ✅ Conversation history persists in Neon Postgres chat_history table

### Backend RAG Service Tasks

- [ ] T048 [US2] Implement backend/src/services/rag_service.py with query embedding, Qdrant search, and GPT-4 response generation
- [ ] T049 [P] [US2] Implement citation extraction in backend/src/services/rag_service.py to parse chapter_title + section_heading from Qdrant payload
- [ ] T050 [P] [US2] Implement selected-text mode in backend/src/services/rag_service.py to include selected_text as context in GPT-4 prompt
- [ ] T051 [P] [US2] Implement conversation history storage in backend/src/services/rag_service.py to insert ChatMessage records in Neon Postgres
- [ ] T052 [US2] Create backend/src/api/routers/chat.py with POST /api/chat/query endpoint accepting {query, user_id, selected_text?}
- [ ] T053 [P] [US2] Add POST /api/chat/history endpoint in backend/src/api/routers/chat.py to retrieve user chat history

### Frontend Chatbot Widget Tasks

- [ ] T054 [US2] Implement frontend/src/components/ChatbotWidget.tsx with collapsible chat interface (fixed bottom-right)
- [ ] T055 [P] [US2] Implement frontend/src/services/chatbot.ts API client to call backend /api/chat/query endpoint
- [ ] T056 [P] [US2] Add selected-text detection in frontend/src/components/SelectedTextQuery.tsx to capture window.getSelection()
- [ ] T057 [P] [US2] Implement frontend/src/components/CitationPreview.tsx to render citations as expandable inline links
- [ ] T058 [US2] Integrate ChatbotWidget into Docusaurus layout via frontend/src/theme/Root.tsx wrapper component
- [ ] T059 [US2] Style chatbot widget in frontend/src/styles/chatbot.css with futuristic theme (neon cyan borders, dark background)

### RAG Testing & Validation Tasks

- [ ] T060 [US2] Create backend/tests/integration/test_rag_pipeline.py with 10 predefined test queries from SC-008
- [ ] T061 [US2] Run RAG test suite and verify ≥8 of 10 queries return answer + citation (≥80% threshold)
- [ ] T062 [US2] Test selected-text mode manually: select text → ask question → verify response references selected content
- [ ] T063 [US2] Verify conversation history persists by querying chat_history table after test queries

---

## Phase 5: User Story 3 - Personalization & Urdu Translation (P2)

**Goal**: Implement Better-Auth authentication, user profiles, personalization button, and Urdu translation button

**Why P2**: Important UX feature and bonus scoring (lower priority than core textbook + RAG)

**Dependencies**: US1 complete (need chapters to personalize/translate), US2 complete (chatbot may personalize responses)

**Independent Test**:
- ✅ User can signup/signin via Better-Auth (email + password)
- ✅ Onboarding form collects skill_level, technical_background, learning_goals
- ✅ "Personalize this Chapter" button generates different content for beginner vs advanced profiles (≥20% text diff)
- ✅ "Translate to Urdu" button produces valid UTF-8 Urdu output preserving Markdown structure
- ✅ Unauthenticated users see "Sign in to personalize" prompt instead of blocking access

### Better-Auth Integration Tasks

- [ ] T064 [US3] Install Better-Auth in frontend with `npm install better-auth` and configure backend URL
- [ ] T065 [P] [US3] Implement backend/src/api/routers/auth.py with POST /api/auth/signup endpoint (email, password)
- [ ] T066 [P] [US3] Implement POST /api/auth/signin endpoint in backend/src/api/routers/auth.py returning JWT token
- [ ] T067 [P] [US3] Implement POST /api/auth/profile endpoint in backend/src/api/routers/auth.py to save user_profiles (skill_level, technical_background, learning_goals)
- [ ] T068 [US3] Create frontend/src/pages/auth/signup.tsx with Better-Auth signup form
- [ ] T069 [P] [US3] Create frontend/src/pages/auth/signin.tsx with Better-Auth signin form
- [ ] T070 [P] [US3] Create frontend/src/pages/auth/onboarding.tsx to collect user profile fields (skill_level dropdown, technical_background textarea, learning_goals textarea)
- [ ] T071 [US3] Create frontend/src/components/AuthProvider.tsx wrapping Docusaurus app with Better-Auth context
- [ ] T072 [US3] Add frontend/src/services/auth.ts with Better-Auth TypeScript client hooks (useAuth, useSession)

### Personalization Tasks

- [ ] T073 [US3] Implement backend/src/services/personalization.py with personalize_content(chapter_text, user_profile) using GPT-4 with profile-based prompt
- [ ] T074 [P] [US3] Implement Redis caching in backend/src/services/personalization.py with cache key: personalized:{user_id}:{chapter_id}:{profile_hash}, TTL 1 hour
- [ ] T075 [P] [US3] Create backend/src/api/routers/personalize.py with POST /api/personalize/chapter endpoint accepting {chapter_id, user_id}
- [ ] T076 [US3] Implement frontend/src/components/PersonalizeButton.tsx as button component calling /api/personalize/chapter
- [ ] T077 [US3] Add PersonalizeButton to chapter layout in frontend/src/theme/DocItem/Layout/index.tsx (top of page)
- [ ] T078 [US3] Handle unauthenticated state in frontend/src/components/PersonalizeButton.tsx showing "Sign in to personalize" prompt

### Urdu Translation Tasks

- [ ] T079 [US3] Implement backend/src/services/translation.py with translate_to_urdu(markdown_text) using GPT-4o-mini with Markdown-preserving prompt
- [ ] T080 [P] [US3] Add Markdown structure validation in backend/src/services/translation.py to verify header/code block/list counts match original
- [ ] T081 [P] [US3] Create backend/src/api/routers/translate.py with POST /api/translate/urdu endpoint accepting {chapter_text}
- [ ] T082 [US3] Implement frontend/src/components/TranslateButton.tsx with toggle state (English/Urdu) and loading spinner
- [ ] T083 [US3] Add TranslateButton to chapter layout in frontend/src/theme/DocItem/Layout/index.tsx (near PersonalizeButton)
- [ ] T084 [US3] Handle translation state in frontend/src/components/TranslateButton.tsx to replace chapter content DOM with translated Markdown

### Testing Tasks

- [ ] T085 [US3] Test personalization with 2 profiles (beginner vs advanced) and verify ≥20% text diff via scripts/validate-content.py --compare-personalized
- [ ] T086 [US3] Test Urdu translation on 3 sample chapters (short/medium/long) and verify Markdown structure preserved with validation script
- [ ] T087 [US3] Test unauthenticated flow: visit chapter → see "Sign in to personalize" → signup → onboarding → personalize chapter
- [ ] T088 [US3] Verify translation toggle works: click "Translate to Urdu" → see Urdu → click again → see English

---

## Phase 6: User Story 4 - Advanced Agent Intelligence (P3 - Bonus)

**Goal**: Implement Claude Subagents and Agent Skills for content enhancement

**Why P3**: Bonus scoring feature, not critical for core functionality

**Dependencies**: US1 complete (need chapters to review/enhance)

**Independent Test**:
- ✅ At least 2 Claude Subagents are functional and documented with usage examples
- ✅ At least 2 Agent Skills are functional with passing test cases
- ✅ Example: Code Review Agent reviews ROS code snippet and returns ≥3 feedback items
- ✅ Example: rewrite_content skill takes text input and returns rewritten version with ≥10% character diff

### Claude Subagent Tasks

- [ ] T089 [P] [US4] Implement backend/src/agents/draft_writer.py with Claude API integration to generate chapter outline (Summary + ≥3 sections)
- [ ] T090 [P] [US4] Implement backend/src/agents/reviewer.py with Claude API integration to review chapter and return ≥3 improvement suggestions
- [ ] T091 [P] [US4] Implement backend/src/agents/code_generator.py with Claude API integration to generate code examples for chapters
- [ ] T092 [P] [US4] Implement backend/src/agents/ros_troubleshooter.py with Claude API integration to analyze ROS error logs and suggest ≥2 fixes

### Agent Skill Tasks

- [ ] T093 [P] [US4] Implement backend/src/skills/rewrite_content.py with Claude API to rewrite text with ≥10% character diff
- [ ] T094 [P] [US4] Implement backend/src/skills/summarize_for_embedding.py with Claude API to generate ≤200 word summaries
- [ ] T095 [P] [US4] Implement backend/src/skills/personalize_for_user.py with Claude API to adapt content based on user profile
- [ ] T096 [P] [US4] Implement backend/src/skills/translate_to_urdu.py with Claude API for translation (alternative to GPT-4o-mini)

### Documentation & Testing Tasks

- [ ] T097 [US4] Create backend/agents/README.md with usage examples for each subagent (command-line interface or API endpoint)
- [ ] T098 [US4] Create backend/skills/README.md with usage examples and function signatures for each skill
- [ ] T099 [US4] Create backend/tests/unit/test_agents.py with test cases for ≥2 subagents (verify output format and quality)
- [ ] T100 [US4] Create backend/tests/unit/test_skills.py with test cases for ≥2 skills (verify character diff, summary length, etc.)
- [ ] T101 [US4] Run agent test suite with pytest backend/tests/unit/test_agents.py and verify passing tests
- [ ] T102 [US4] Run skill test suite with pytest backend/tests/unit/test_skills.py and verify passing tests

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Final testing, documentation, deployment optimization, and demo preparation

**Dependencies**: All user stories complete

**Independent Test**: All success criteria (SC-001 through SC-013) pass verification

### Testing & Validation Tasks

- [ ] T103 Run full content quality validation with scripts/validate-content.py --all --report and verify ≥23 of 25 chapters pass 15 criteria
- [ ] T104 [P] Run RAG test suite with backend/tests/integration/test_rag_pipeline.py and verify ≥8 of 10 queries pass (SC-008)
- [ ] T105 [P] Test Docusaurus build performance with `npm run build` and verify build time ≤5 minutes (NFR-002)
- [ ] T106 [P] Test page load performance with Lighthouse and verify FCP <1.5 seconds (NFR-005)
- [ ] T107 [P] Test mobile responsiveness on 320px/768px/1024px viewports and verify touch targets ≥44x44px (NFR-004)
- [ ] T108 Test Qdrant storage usage via dashboard and verify ≤1GB (NFR-003)
- [ ] T109 [P] Test Neon Postgres storage via dashboard and verify ≤512MB (NFR-003)

### Documentation Tasks

- [ ] T110 [P] Create frontend/README.md with setup instructions, environment variables, and deployment steps
- [ ] T111 [P] Create backend/README.md with setup instructions, API endpoint documentation, and testing commands
- [ ] T112 [P] Create root README.md with project overview, architecture diagram, and quick start guide
- [ ] T113 [P] Update backend/.env.example with all required environment variables and example values
- [ ] T114 Create DEPLOYMENT.md with step-by-step deployment instructions for GitHub Pages + backend hosting

### Deployment Optimization Tasks

- [ ] T115 Deploy FastAPI backend to Render/Railway/HuggingFace Spaces using backend deployment guide
- [ ] T116 [P] Configure CORS in backend/src/api/middleware.py to allow frontend domain (GitHub Pages URL)
- [ ] T117 [P] Set up Redis cache (Upstash/Railway Free Tier) and add REDIS_URL to backend .env
- [ ] T118 Test backend cold start time and verify ≤10 seconds (NFR-003)
- [ ] T119 [P] Add rate limiting monitoring to backend logs to track 100 req/min threshold (NFR-006)
- [ ] T120 Verify GitHub Pages deployment with custom domain support (optional, not required for MVP)

### Demo & Presentation Tasks

- [ ] T121 Record demo video showing: textbook navigation, RAG chatbot query with citation, personalization, Urdu translation
- [ ] T122 [P] Create demo script in docs/DEMO_SCRIPT.md with timestamped walkthrough of all features
- [ ] T123 [P] Take screenshots of: homepage, chapter page, chatbot interaction, personalization, translation for documentation
- [ ] T124 Prepare hackathon submission with: GitHub repo URL, live demo URL, demo video link, architecture overview

---

## Dependency Graph

```text
Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ↓
    ├─→ Phase 3 (US1: Textbook Generation) ← REQUIRED FOR ALL OTHERS
    │       ↓
    │       ├─→ Phase 4 (US2: RAG Chatbot)
    │       │
    │       ├─→ Phase 5 (US3: Personalization & Translation)
    │       │
    │       └─→ Phase 6 (US4: Agent Intelligence - BONUS)
    │
    └─→ Phase 7 (Polish) ← REQUIRES ALL USER STORIES COMPLETE
```

**Critical Path**: Phase 1 → Phase 2 → Phase 3 (US1) → Phase 4 (US2) → Phase 7

**Parallel Opportunities**:
- Phase 3 modules can be generated in parallel (T029-T032)
- Phase 4 backend RAG service and frontend widget can be developed in parallel (T048-T053 backend, T054-T059 frontend)
- Phase 5 auth integration and personalization can be developed in parallel (T064-T072 auth, T073-T078 personalization)
- Phase 6 all subagents and skills can be developed in parallel (T089-T096)

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

For initial delivery, focus on:
- ✅ **Phase 1**: Setup
- ✅ **Phase 2**: Foundational
- ✅ **Phase 3**: US1 - Textbook Generation & Deployment (MUST HAVE)
- ✅ **Phase 4**: US2 - RAG Chatbot Integration (MUST HAVE)
- ⚠️ **Phase 5**: US3 - Personalization & Translation (SHOULD HAVE for bonus points)
- ❌ **Phase 6**: US4 - Agent Intelligence (NICE TO HAVE, bonus only)

**Recommended Execution Order**:
1. Complete Phase 1-2 (setup infrastructure)
2. Complete Phase 3 (US1) to have working textbook deployed
3. Complete Phase 4 (US2) to have functional RAG chatbot
4. If time permits: Phase 5 (US3) for bonus points
5. If time permits: Phase 6 (US4) for additional bonus points
6. Final: Phase 7 (polish and testing)

### Parallel Execution Examples

**Week 1**: Setup + Foundational + Textbook Generation
```text
Day 1-2: T001-T015 (Setup) → ALL PARALLEL except T001, T002 first
Day 3-4: T016-T027 (Foundational) → T017-T024 PARALLEL after T016
Day 5-7: T028-T047 (US1 Textbook) → T029-T032 PARALLEL, T036-T039 PARALLEL, T045-T047 SEQUENTIAL
```

**Week 2**: RAG Chatbot + Personalization
```text
Day 1-3: T048-T063 (US2 RAG) → Backend (T048-T053) and Frontend (T054-T059) PARALLEL
Day 4-7: T064-T088 (US3 Personalization) → Auth (T064-T072), Personalization (T073-T078), Translation (T079-T084) can overlap
```

**Week 3**: Bonus + Polish
```text
Day 1-4: T089-T102 (US4 Agents) → All agents/skills PARALLEL (T089-T096)
Day 5-7: T103-T124 (Polish) → Testing (T103-T109) PARALLEL, then Documentation (T110-T114), then Deployment (T115-T120)
```

---

## Task Statistics

**Total Tasks**: 124
- Phase 1 (Setup): 15 tasks
- Phase 2 (Foundational): 12 tasks
- Phase 3 (US1 - Textbook): 20 tasks
- Phase 4 (US2 - RAG): 16 tasks
- Phase 5 (US3 - Personalization): 25 tasks
- Phase 6 (US4 - Agents): 14 tasks (BONUS)
- Phase 7 (Polish): 22 tasks

**Parallelizable Tasks**: 67 tasks marked with [P] (54% of total)

**Critical Path Tasks**: 31 tasks (25% of total) - must be executed sequentially

**Estimated Timeline**:
- MVP (US1 + US2): ~10-12 days (79 tasks)
- Full Feature Set (US1-US3): ~15-18 days (104 tasks)
- With Bonus (US1-US4): ~20-22 days (118 tasks + polish)

---

## Success Criteria Validation Checklist

After completing all phases, verify these success criteria from spec.md:

### Build & Deployment (US1)
- [ ] **SC-001**: Run `npm run build` in frontend/, verify zero errors/warnings
- [ ] **SC-002**: Visit https://[username].github.io/[repo], verify HTTP 200 and valid SSL
- [ ] **SC-003**: Verify 4 modules with 25 chapters match course outline structure

### Content Quality (US1)
- [ ] **SC-004**: Run scripts/validate-content.py --all, verify ≥23 of 25 chapters pass 15 criteria
- [ ] **SC-005**: Run scripts/ingest-embeddings.py --validate, verify all 5 RAG metrics pass
- [ ] **SC-006**: Run speckit validation (if available), verify ≥90% completeness score

### RAG Chatbot (US2)
- [ ] **SC-008**: Run backend/tests/integration/test_rag_pipeline.py, verify 8/10 queries pass
- [ ] **SC-009**: Test selected-text mode manually, verify response references selected content
- [ ] **SC-010**: Test signup/signin flow, verify session persists after page refresh

### Personalization & Translation (US3)
- [ ] **SC-011**: Test personalization with beginner vs advanced profiles, verify ≥20% text diff
- [ ] **SC-012**: Translate 3 sample chapters to Urdu, verify Markdown structure preserved (≥95% success)

### Bonus Features (US4)
- [ ] **SC-007**: Verify ≥2 bonus features implemented (Agent Skills, Personalization, Translation)
- [ ] **SC-013**: Verify ≥2 subagents OR ≥2 skills functional with documented usage examples

---

## Notes

- **Environment Setup**: Ensure all API keys (QDRANT_API_KEY, OPENAI_API_KEY, NEON_DB_URL, BETTER_AUTH_SECRET, REDIS_URL) are configured before starting Phase 2
- **Free Tier Limits**: Monitor Qdrant (1GB), Neon (512MB, 10GB bandwidth), GitHub Pages (100GB bandwidth) throughout development
- **Content Generation**: Use GPT-4 or Claude for chapter generation in T029-T032 with chapter template prompt from constitution.md
- **Testing Strategy**: Tests are NOT auto-generated; only SC validation tests are included (T060, T061, T099, T100)
- **Deployment Priority**: Deploy US1 (textbook) ASAP for early feedback, then US2 (RAG), then US3 (personalization)

---

**Generated**: 2025-12-04 | **Status**: Ready for Implementation
