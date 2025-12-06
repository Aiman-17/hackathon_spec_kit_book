# Feature Specification: AI-Native Textbook & RAG System for Physical AI & Humanoid Robotics

**Feature Branch**: `001-ai-robotics-textbook`
**Created**: 2025-12-04
**Status**: Draft
**Input**: User description: "project_title: "AI-Native Textbook & RAG System for Physical AI & Humanoid Robotics"

  description: >
    Build a complete Docusaurus-based textbook aligned with the Physical AI &
    Humanoid Robotics course. Integrate a Retrieval-Augmented Generation chatbot
    using OpenAI Agents/ChatKit, FastAPI, Qdrant Cloud, and Neon Postgres.
    Implement personalization, Urdu translation, and optional Claude Subagents/
    Agent Skills for maximum scoring. Deploy the project to GitHub Pages.

  inputs:
    - course_outline: "/mnt/data/Hackathon I: Physical AI & Humanoid Robotics Textbook.docx"
    - docusaurus_project_folder: "./"
    - qdrant_api_key
    - neon_connection_string
    - better_auth_env_config
    - openai_api_key
    - github_repository_url

  goals:
    - "Produce a complete multi-chapter textbook in Docusaurus format."
    - "Deploy the book to GitHub Pages."
    - "Integrate a fully operational RAG chatbot."
    - "Implement personalization based on user skill/background."
    - "Implement Urdu translation per chapter."
    - "Enable Claude Subagents & Agent Skills for advanced intelligence."

  # ============================================================
  # NEW SECTION — BOOK STRUCTURE (4 MODULES)
  # ============================================================

  book_structure:

    module_1:
      title: "Foundations of Physical AI & Robotics"
      chapters:
        - "Introduction to Physical AI"
        - "Humanoid Robotics Overview"
        - "Robot Hardware: Sensors, Actuators, Control Systems"
        - "Mathematics for Robotics (Kinematics & Dynamics)"
        - "Introduction to Robot Operating System (ROS 2)"
        - "Linear Algebra & Calculus for Robot Motion"

    module_2:
      title: "Simulation Environments & Robotics Software"
      chapters:
        - "ROS 2 in Depth: Nodes, Topics, Services, Actions"
        - "Building Robot Applications with ROS Packages"
        - "URDF & XACRO for Robot Modeling"
        - "Gazebo Classic vs. Gazebo Garden (Ignition)"
        - "Building a Humanoid Model in Gazebo"
        - "Unity for Robotics Visualization & HRI"

    module_3:
      title: "Advanced Perception, Navigation & Control"
      chapters:
        - "Computer Vision for Robotics"
        - "NVIDIA Isaac Sim: Set Up & Fundamentals"
        - "Isaac ROS Perception Pipelines (VSLAM, AprilTags)"
        - "Nav2 Navigation Stack"
        - "Mapping & Localization"
        - "Motion Planning for Humanoids (Bipedal Control)"
        - "Vision-Language-Action Pipelines (VLA)"

    module_4:
      title: "Humanoid AI Systems & Capstone Development"
      chapters:
        - "Integrating Perception, Action & Control"
        - "AI Agents for Autonomous Robotics"
        - "End-to-End Humanoid Pipeline: Sensing → Reasoning → Acting"
        - "Multi-Agent Coordination for Robotics"
        - "Project: Build an Autonomous Humanoid Simulation"
        - "Final Capstone: Full Humanoid Robotics System"

  # ============================================================

  functional_requirements:

    textbook:
      - "Generate all chapters aligned with the course outline."
      - "Use a consistent chapter template: Summary, Objectives, Tutorial, Code, Glossary, Review."
      - "Output chapters into /docs with valid Docusaurus metadata."
      - "Provide futuristic/custom UI template."

    rag_chatbot:
      - "Embed a chatbot widget into the Docusaurus UI."
      - "Backend must be FastAPI."
      - "Use Qdrant Cloud Free Tier for vector storage."
      - "Ingest all textbook chapters as embeddings.""
      - "Support 'Ask based on selected text' mode (anchored RAG)."
      - "Return citations and matching segments."
      - "Store conversation history in Neon Postgres."

    personalization:
      - "Use Better-Auth for signup/signin."
      - "Request user background information at signup: skill level (beginner/intermediate/advanced), technical background (prior experience), and learning goals."
      - "Personalize each chapter based on user profile (generate on-demand, cache transiently)."
      - "Add 'Personalize this Chapter' button at top of each page."
      - "Store personalized content in session/cache; regenerate when profile changes."

    urdu_translation:
      - "Add 'Translate to Urdu' button at chapter start."
      - "Translate full content while preserving Markdown formatting."
      - "Allow toggling back to English."

    bonus_intelligence:
      - "Implement Claude Subagents to handle tasks like:"
      - "  - Code Review Agent for ROS/Isaac"
      - "  - Simulation Diagnostics Agent"
      - "  - Chapter Writer Agent"
      - "  - Glossary Expander Agent"
      - "Create reusable Agent Skills that support writing, reviewing, and generating exercises."

  non_functional_requirements:
    - "All content must be modular and RAG-friendly."
    - "Pages must build without errors on GitHub Pages."
    - "Backend must run on minimal infrastructure (FastAPI, serverless Postgres, Qdrant Free Tier)."
    - "Design must be mobile responsive."
    - "Performance: RAG responses < 2 seconds when possible."

  architecture:

    frontend:
      - "Docusaurus-based book."
      - "Custom React components for chatbot widget."
      - "Personalization and translation controls."
      - "Futuristic robotic UI design."

    backend:
      - "FastAPI application with modules:"
      - "  - document_ingestion"
      - "  - embedding_pipeline (semantic chunking by H2/H3 sections, 500-1000 token overlap)"
      - "  - chat_router (Agents/ChatKit)"
      - "  - postgres_manager (Neon)"
      - "  - qdrant_manager"
      - "  - user_profile_manager"
      - "  - translation_service"

    data:
      - "Qdrant vectors: chapter_chunks(collection=book_chapters, embedding_model=text-embedding-3-small, dimensions=1536)"
      - "Neon Postgres tables:"
      - "  - users (auth)"
      - "  - user_profiles (background info)"
      - "  - chat_history"
      - "Note: Personalized content stored transiently in session/cache, regenerated on-demand"

    agent_layer:
      - "Primary Agent: RAG Answering Agent"
      - "Subagents:"
      - "  - Draft Writer Agent"
      - "  - Reviewer Agent"
      - "  - Code Example Generator Agent"
      - "  - ROS Troubleshooter Agent"
      - "Agent Skills:"
      - "  - rewrite_content"
      - "  - summarize_for_embedding"
      - "  - personalize_for_user"
      - "  - translate_to_urdu"

  deployment:

    github_actions:
      - "Build and deploy Docusaurus to GitHub Pages."
      - "Trigger on every commit to main branch."

    python_backend:
      - "Deploy FastAPI backend to Replit/Vercel/HuggingFace Spaces/Render."
      - "Use environment variables for secrets."

  tasks_to_generate:
    - "Generate full chapter set from course outline."
    - "Generate Docusaurus sidebar."
    - "Generate futuristic UI theme."
    - "Implement chatbot widget."
    - "Implement FastAPI backend scaffold."
    - "Implement embedding pipeline and ingestion."
    - "Implement RAG query endpoint."
    - "Integrate Better-Auth."
    - "Implement personalization algorithm."
    - "Implement Urdu translation pipeline."
    - "Create Agent Skills + Subagents."
    - "Deploy to GitHub Pages."
    - "Create demo video script."

  acceptance_criteria:
    - "Book builds successfully and deploys to GitHub Pages."
    - "Chatbot answers correctly using RAG, with citations."
    - "Chatbot supports answering based on selected text."
    - "Signup/signin works via Better-Auth."
    - "Personalization button works and updates chapter content."
    - "Urdu translation button works flawlessly."
    - "Optional subagents and agent skills implemented for bonus scoring."

  scoring_alignment:
    base_points:
       textbook + RAG chatbot working end-to-end"

    bonus_points:
           -"Claude Subagents + Agent Skills"
           -"Better-Auth signup/signin + personalization input"
        -"Button-based personalization inside chapters"
    -"Urdu translation button inside chapters""

## Clarifications

### Session 2025-12-04

- Q: When ingesting textbook chapters into Qdrant as embeddings, which embedding model should be used? → A: OpenAI text-embedding-3-small
- Q: What chunking strategy should be used when splitting textbook chapters into embeddings for Qdrant? → A: Semantic chunking by section (H2/H3 headers, 500-1000 token overlap)
- Q: How should personalized chapter content be stored and managed? → A: Store personalized content transiently (cache/session) and regenerate on-demand
- Q: When a user tries to access personalized content without being logged in, what should happen? → A: Show read-only default content with "Sign in to personalize" prompt
- Q: What user background information should be collected at signup for personalization? → A: Skill level, technical background, and learning goals

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Textbook Content Generation & Deployment (Priority: P1)

Produce a complete multi-chapter textbook in Docusaurus format, aligned with the course outline, and deploy it to GitHub Pages.

**Why this priority**: Core deliverable, foundational for the entire project.

**Independent Test**: The Docusaurus site builds successfully and is live on GitHub Pages, with all chapters accessible and correctly formatted.

**Acceptance Scenarios**:

1.  **Given** the course outline and content generation tools, **When** the content generation process is run, **Then** all chapters are generated and placed in `/docs` with valid Docusaurus metadata.
2.  **Given** the generated Docusaurus project, **When** the GitHub Actions pipeline is triggered, **Then** the site builds successfully and deploys to GitHub Pages with a live public link.
3.  **Given** a chapter's content, **When** reviewed against the standards, **Then** it starts with Summary, Learning Objectives, Prerequisites, ends with Key Takeaways, Glossary Terms, Review Questions, uses clean Markdown, H2/H3/H4 headings consistently, and includes code blocks/tables as needed.

---

### User Story 2 - RAG Chatbot Integration (Priority: P1)

Integrate a fully operational RAG chatbot into the Docusaurus UI that answers questions based on textbook content, with personalization and Urdu translation.

**Why this priority**: Core AI-native feature and a primary goal.

**Independent Test**: The chatbot widget is embedded, responds to queries accurately with citations, supports "ask based on selected text", stores conversation history, and can personalize/translate responses.

**Acceptance Scenarios**:

1.  **Given** the Docusaurus textbook, **When** a user asks a question via the embedded chatbot widget, **Then** the chatbot returns a correct answer with citations and matching segments, leveraging Qdrant for vector search and Neon Postgres for conversation history.
2.  **Given** selected text in a chapter, **When** the "Ask based on selected text" mode is activated, **Then** the chatbot uses the selected text as context for its response.

---

### User Story 3 - Personalization & Urdu Translation (Priority: P2)

Implement personalization of chapter content based on user profiles and provide a button for Urdu translation of chapters.

**Why this priority**: Important for user experience and key bonus features.

**Independent Test**: User signup/signin works, and the "Personalize this Chapter" and "Translate to Urdu" buttons function as expected, modifying chapter content based on user profile and language selection.

**Acceptance Scenarios**:

1.  **Given** a user navigates to a chapter, **When** they click "Personalize this Chapter", **Then** the chapter content adapts based on their stored user profile information.
2.  **Given** an unauthenticated user navigates to a chapter, **When** they view the page, **Then** they see default content with a "Sign in to personalize" prompt, and can still browse all chapters normally.
3.  **Given** a user navigates to a chapter, **When** they click "Translate to Urdu", **Then** the full chapter content is translated to Urdu while preserving Markdown formatting, and can be toggled back to English.
4.  **Given** a new user, **When** they sign up via Better-Auth, **Then** their background information (skill level, technical background, learning goals) is requested and stored for personalization.

---

### User Story 4 - Advanced Agent Intelligence (Priority: P3 - Bonus)

Enable Claude Subagents and Agent Skills to augment the textbook writing, review, and diagnostic workflows.

**Why this priority**: Bonus scoring, enhances development process with advanced AI.

**Independent Test**: The implemented Claude Subagents and Agent Skills can perform their specified tasks (e.g., code review, simulation diagnostics, chapter writing, glossary expansion, content rewriting, summarization, personalization, translation) to enhance the textbook.

**Acceptance Scenarios**:

1.  **Given** a task for content generation or review, **When** a Claude Subagent is invoked, **Then** it successfully performs its specialized task (e.g., Code Review Agent reviews ROS/Isaac code, Chapter Writer Agent drafts content).
2.  **Given** a need for content manipulation (e.g., rewrite, summarize, personalize, translate), **When** an Agent Skill is activated, **Then** it applies the transformation to the content.

### Edge Cases

- **Unauthenticated personalization access**: Show read-only default content with "Sign in to personalize" prompt; users can still browse all chapters normally.
- How does the system handle very long chapters for RAG ingestion and translation?
- What if a chapter cannot be translated accurately to Urdu (e.g., highly technical terms with no direct equivalent)?
- How are API keys and connection strings securely managed in deployment?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Generate all chapters aligned with the course outline.
- **FR-002**: Use a consistent chapter template: Summary, Objectives, Tutorial, Code, Glossary, Review.
- **FR-003**: Output chapters into /docs with valid Docusaurus metadata.
- **FR-004**: Provide futuristic/custom UI template.
- **FR-005**: Embed a chatbot widget into the Docusaurus UI.
- **FR-006**: Backend must be FastAPI.
- **FR-007**: Use Qdrant Cloud Free Tier for vector storage.
- **FR-008**: Ingest all textbook chapters as embeddings using text-embedding-3-small with semantic chunking by H2/H3 section headers (500-1000 token overlap).
- **FR-009**: Support 'Ask based on selected text' mode (anchored RAG).
- **FR-010**: Return citations and matching segments.
- **FR-011**: Store conversation history in Neon Postgres.
- **FR-012**: Use Better-Auth for signup/signin.
- **FR-013**: Request user background information at signup: skill level (beginner/intermediate/advanced), technical background (prior experience with robotics/AI/programming), and learning goals (career development, academic research, hobby projects, etc.).
- **FR-014**: Personalize each chapter based on user profile.
- **FR-015**: Add 'Personalize this Chapter' button at top of each page.
- **FR-015a**: For unauthenticated users, show default content with "Sign in to personalize" prompt instead of blocking access.
- **FR-016**: Add 'Translate to Urdu' button at chapter start.
- **FR-017**: Translate full content while preserving Markdown formatting.
- **FR-018**: Allow toggling back to English.
- **FR-019**: Implement Claude Subagents to handle tasks like Code Review, Simulation Diagnostics, Chapter Writing, Glossary Expansion.
- **FR-020**: Create reusable Agent Skills that support writing, reviewing, and generating exercises.

### Non-Functional Requirements

- **NFR-001**: Content Modularity & RAG Compatibility
  - Each chapter must be independently embeddable with ≤10,000 tokens per chapter
  - Semantic sections identifiable by H2/H3 headers (no arbitrary splits)
  - Code blocks must be syntax-highlighted and include copy button
  - See "Definitions & Standards > RAG-Friendly Content" for detailed criteria

- **NFR-002**: Build Quality & Reliability
  - Zero build errors when running `npm run build` for Docusaurus site
  - Zero TypeScript/ESLint errors in frontend code
  - Zero Python linting errors in FastAPI backend (run: `flake8` or `ruff`)
  - Build time ≤5 minutes for full site generation
  - GitHub Actions deployment pipeline completes within 10 minutes

- **NFR-003**: Infrastructure Constraints (Free Tier Limits)
  - **Qdrant Cloud Free Tier**: ≤1GB storage, ≤1M vectors, no dedicated cluster
  - **Neon Postgres Free Tier**: ≤512MB storage, ≤10GB bandwidth/month, ≤3 databases
  - **GitHub Pages**: ≤1GB repository size, ≤100GB bandwidth/month, static sites only
  - **Backend Hosting**: Deploy to free tier (Render/Railway/HuggingFace Spaces)
  - Backend cold start: ≤10 seconds acceptable on free tier
  - No paid services required for MVP functionality

- **NFR-004**: Mobile Responsiveness
  - Must support viewport sizes: 320px (mobile), 768px (tablet), 1024px+ (desktop)
  - Touch targets ≥44x44px for all interactive buttons/links (WCAG guideline)
  - Font sizes ≥16px for body text to prevent mobile zoom
  - Navigation menu collapses to hamburger menu on mobile (<768px)
  - Chatbot widget adapts to mobile screen (full-width on small screens)
  - No horizontal scrolling on any viewport size

- **NFR-005**: Performance Targets
  - **RAG Query Response**: p95 latency < 2 seconds (from user query to answer display)
  - **Page Load Time**: First Contentful Paint (FCP) < 1.5 seconds
  - **Chatbot Widget Load**: ≤500ms to render chat interface
  - **Personalization Generation**: ≤5 seconds per chapter
  - **Translation Request**: ≤10 seconds per chapter (Urdu translation)
  - **Embedding Pipeline**: Process all 25 chapters within 10 minutes

- **NFR-006**: Security & Data Privacy
  - All API keys stored in environment variables (never hardcoded)
  - HTTPS required for all external API calls
  - User passwords hashed using Better-Auth default (bcrypt or argon2)
  - No sensitive data logged to console or error messages
  - CORS configured to allow only frontend domain
  - Rate limiting on backend API endpoints (100 requests/minute per IP)

### Key Entities *(include if feature involves data)*

- **User**: Represents an authenticated user, storing basic auth information.
- **UserProfile**: Stores additional background information about the user for personalization including: skill_level (beginner/intermediate/advanced), technical_background (text describing prior experience with robotics/AI/programming), and learning_goals (text describing objectives like career development, academic research, hobby projects).
- **ChapterChunk**: Represents a semantic unit of a textbook chapter (H2/H3 sections), used for vector storage in Qdrant with text-embedding-3-small (1536 dimensions).
- **ChatMessage**: Represents a single message in a conversation, storing chat history in Neon Postgres.
- **PersonalizedContent** (transient): Cached personalized chapter content stored in session/Redis, regenerated on-demand when requested or when user profile changes.

## Out of Scope *(explicit non-goals)*

To maintain focus and ensure timely delivery, the following are explicitly **NOT** included in this feature:

### Content Beyond Textbook Chapters
- ❌ Video tutorials or multimedia content beyond static images/diagrams
- ❌ Interactive coding environments (embedded Jupyter notebooks, CodeSandbox, REPL widgets)
- ❌ Downloadable PDF/EPUB versions of the textbook
- ❌ Quizzes, assessments, graded assignments, or certification system
- ❌ Live coding challenges or automated code testing

### User Engagement Features
- ❌ Offline mode or Progressive Web App (PWA) capabilities
- ❌ Full-text search (beyond RAG-based semantic search)
- ❌ User-generated content (comments, discussions, forums, ratings)
- ❌ Social features (sharing to social media, likes, bookmarks, reading lists)
- ❌ Collaborative features (shared annotations, group study rooms)
- ❌ Email notifications, newsletters, or digest emails
- ❌ Progress tracking dashboard or learning analytics for users

### Administrative & Content Management
- ❌ Admin dashboard for content management (WYSIWYG editor, content approval workflow)
- ❌ Version control or edit history for chapters beyond Git commits
- ❌ Content scheduling or publishing workflow
- ❌ User role management (admin, moderator, contributor roles)

### Integrations & Third-Party Services
- ❌ Analytics tracking beyond basic page views (no Google Analytics, Plausible, Mixpanel)
- ❌ Third-party integrations (Slack, Discord, Microsoft Teams, LMS platforms like Canvas/Moodle)
- ❌ Payment processing, subscription models, or premium content tiers
- ❌ Single Sign-On (SSO) with Google, GitHub, or enterprise providers
- ❌ Export to external formats (SCORM, LTI, xAPI)

### Localization & Accessibility
- ❌ Multi-language support beyond English and Urdu (no Spanish, French, Chinese, etc.)
- ❌ Full WCAG 2.1 AAA accessibility compliance (target: basic accessibility only)
- ❌ Screen reader optimization beyond standard semantic HTML
- ❌ Internationalization (i18n) framework for future languages

### Performance & Infrastructure
- ❌ Real-time collaboration features (live cursors, presence indicators)
- ❌ Advanced caching strategies (CDN distribution, edge computing, service workers)
- ❌ Load testing or auto-scaling infrastructure
- ❌ Custom domain setup (use GitHub Pages default domain: `<username>.github.io/<repo>`)
- ❌ Backup/disaster recovery beyond GitHub's built-in version control
- ❌ Monitoring dashboards (Grafana, Datadog, New Relic)

### AI/ML Features Beyond Core RAG
- ❌ Adaptive learning paths based on user performance
- ❌ AI-generated practice problems or homework assignments
- ❌ Predictive analytics for learning outcomes
- ❌ Conversational AI tutoring beyond RAG-based Q&A
- ❌ Speech-to-text or text-to-speech capabilities

### Hardware/Robotics Integration
- ❌ Direct hardware integration with physical robots
- ❌ Simulation environment hosting (users must run locally)
- ❌ Remote lab access or cloud-based robotics platforms
- ❌ Integration with ROS environments beyond documentation

## Definitions & Standards

This section clarifies terminology used in requirements and success criteria.

### RAG-Friendly Content

Content is "RAG-friendly" when it meets these characteristics:

1. **Semantic Chunking**: Divided by meaningful section boundaries (H2/H3 headers), not arbitrary character limits
2. **Self-Contained Chunks**: Each chunk can be understood without requiring full chapter context
3. **Explicit Terminology**: Key terms are defined inline or linked to glossary (avoid implicit references)
4. **Contextual Code Examples**: Code snippets include context comments explaining purpose
5. **Full Names Over Pronouns**: Use "ROS 2 nodes" instead of "they" for better retrieval matching
6. **Consistent Formatting**: Markdown structure preserved for reliable parsing

**Example of RAG-friendly**:
```markdown
## ROS 2 Nodes

ROS 2 nodes are independent processes that perform computation. Each node communicates
with other nodes via topics, services, or actions. A node is created using the `rclpy.Node`
class in Python.
```

**Example of NOT RAG-friendly**:
```markdown
## Overview

They communicate with each other. You create them using the class.
(Vague pronouns, no context, missing technical details)
```

### Pedagogical Rigor

Content demonstrates "pedagogical rigor" when it includes:

1. **Measurable Learning Objectives**: Uses Bloom's taxonomy verbs (explain, implement, analyze, evaluate, create) instead of vague verbs (understand, learn, know)
2. **Explicit Prerequisites**: States required prior knowledge upfront
3. **Progressive Complexity**: Examples start simple and build to advanced concepts
4. **Misconception Addressing**: Identifies and corrects common student errors
5. **Aligned Assessments**: Review questions directly test stated learning objectives
6. **Terminology Consistency**: Uses canonical terms throughout (no synonym confusion)

**Example of rigorous learning objective**:
- ✅ "Implement a ROS 2 publisher node that sends sensor data at 10 Hz"

**Example of non-rigorous**:
- ❌ "Understand ROS 2 publishers" (not measurable, vague verb)

### Futuristic Robotic UI Design

The "futuristic robotic UI" aesthetic includes these specifications:

**Color Palette**:
- Primary background: Navy (#0A1929)
- Accent colors: Cyan (#00E5FF), Purple (#B388FF), Teal (#00BFA5)
- Text: White (#FFFFFF) on dark backgrounds
- Code blocks: Dark gray (#1E1E1E) with syntax highlighting

**Typography**:
- Body text: Inter (sans-serif), 16px base size
- Headings: Orbitron or Rajdhani (geometric sans-serif)
- Code: Fira Code (monospace with ligatures)

**UI Components**:
- Buttons: Rounded corners (8px), neon glow effects on hover
- Cards: Semi-transparent backgrounds with subtle borders
- Interactive elements: Animated gradients (cyan → purple)
- Icons: Line-style icons with geometric shapes
- Navigation: Side panel with hierarchical tree structure

**Visual Effects**:
- Subtle grid patterns in backgrounds
- Glow effects on active elements (box-shadow with color)
- Smooth transitions (300ms ease-in-out)
- Geometric accent shapes (hexagons, circuits)

**Reference Inspiration**: GitHub dark theme, Visual Studio Code dark+ theme, sci-fi HUD interfaces

### Personalization Logic

Content personalization adapts based on user profile fields:

**By Skill Level**:
- **Beginner**: Add prerequisite refreshers, expand all acronyms on first use, include step-by-step tutorials, provide more code comments
- **Intermediate**: Standard content with practical examples, assume basic prerequisite knowledge
- **Advanced**: Condensed explanations, focus on edge cases and optimization, include research paper references, advanced troubleshooting

**By Technical Background**:
- If background includes "robotics experience": Skip basic robotics concepts, focus on ROS-specific details
- If background includes "Python experience": Assume Python knowledge, skip language basics
- If background includes "no prior experience": Add foundational explainers for all technical concepts

**By Learning Goals**:
- "Career development": Emphasize industry best practices, job-relevant skills, project portfolio tips
- "Academic research": Include research papers, mathematical derivations, theoretical depth
- "Hobby projects": Focus on practical quick-start guides, emphasize fun/creative applications

**Measurable Personalization**:
- Beginner content should be ≥10% longer (more explanations)
- Advanced content should include ≥2 research paper citations per chapter
- Text diff between skill levels should show ≥20% character change

## Success Criteria *(mandatory)*

### Measurable Outcomes

All criteria must be met by project deadline: **[INSERT_HACKATHON_DEADLINE]**

---

#### **Build & Deployment**

- **SC-001**: Docusaurus site builds successfully with zero errors and zero warnings
  - **Verification**: Run `npm run build` in project root, exit code must be 0
  - **Threshold**: No errors, no warnings in build output
  - **Evidence**: Build log screenshot showing "Compiled successfully"

- **SC-002**: GitHub Pages deployment is accessible at public URL with valid SSL
  - **Verification**: Visit `https://<username>.github.io/<repo>`, check HTTP 200 response
  - **Threshold**: Page loads within 3 seconds, SSL certificate valid, no mixed content warnings
  - **Evidence**: Screenshot of live site with browser address bar showing HTTPS lock icon

---

#### **Content Structure & Quality**

- **SC-003**: Book contains exactly 4 modules and 25 chapters with titles matching course outline
  - **Verification**: Automated comparison script validates chapter titles against official course specification
  - **Threshold**: 100% title match (case-insensitive), correct module grouping
  - **Expected Structure**:
    - Module 1: "Foundations of Physical AI & Robotics" (6 chapters)
    - Module 2: "Simulation Environments & Robotics Software" (6 chapters)
    - Module 3: "Advanced Perception, Navigation & Control" (7 chapters)
    - Module 4: "Humanoid AI Systems & Capstone Development" (6 chapters)
  - **Evidence**: Docusaurus sidebar navigation matching course outline

- **SC-004**: All 25 chapters pass Content Quality Checklist (≥90% criteria met per chapter)

  **Chapter Structure Criteria** (7 required):
  - ✅ Includes Summary (≥3 sentences at chapter start)
  - ✅ Includes ≥3 Learning Objectives using Bloom's taxonomy verbs (explain, implement, analyze, evaluate, create)
  - ✅ Lists Prerequisites explicitly (or states "None")
  - ✅ Includes Key Takeaways section (≥3 bullet points)
  - ✅ Includes Glossary with ≥5 technical terms defined
  - ✅ Includes ≥3 Review Questions aligned with learning objectives
  - ✅ Uses consistent heading hierarchy (H1 → H2 → H3, no level skipping)

  **Technical Quality Criteria** (5 required):
  - ✅ Code examples are syntactically valid (pass language-specific linting)
  - ✅ Code blocks specify language (```python, ```bash, not generic ```)
  - ✅ Technical terms defined on first use or linked to glossary
  - ✅ External references include valid URLs or citations
  - ✅ No placeholder text ("TODO", "Lorem ipsum", "[INSERT_HERE]", "TBD")

  **Pedagogical Quality Criteria** (3 required):
  - ✅ Examples progress from simple to complex within chapter
  - ✅ At least 1 common misconception addressed per chapter
  - ✅ Review questions test stated learning objectives (direct alignment)

  **Verification**:
  - Automated: Run markdownlint + custom quality checker script on all chapters
  - Manual: Spot-check 5 randomly selected chapters (1 per module + 1 extra)

  **Threshold**: ≥23 of 25 chapters (92%) pass all 15 criteria

  **Evidence**: Quality report JSON showing pass/fail per chapter per criterion

- **SC-005**: RAG Pipeline Quality Metrics pass all 5 thresholds

  **Metric 1 - Chunking Success**: 100% of chapters process without errors
  - **Test**: Run embedding pipeline on all 25 chapters
  - **Threshold**: Zero errors, zero exceptions in logs

  **Metric 2 - Chunk Size Distribution**: Average chunk size 500-1500 tokens
  - **Test**: Measure token count for all generated chunks
  - **Threshold**: ≥80% of chunks fall within 500-1500 token range

  **Metric 3 - Vector Coverage**: Qdrant collection contains ≥100 chapter chunks
  - **Test**: Query Qdrant API for collection size: `GET /collections/book_chapters`
  - **Threshold**: Collection size ≥100 vectors

  **Metric 4 - Retrieval Precision**: ≥70% relevant results for test query set
  - **Test**: Run 10 predefined test queries, manually review top-3 results for relevance
  - **Threshold**: ≥7 of 10 queries return at least 2 relevant results in top-3
  - **Test Queries**:
    1. "What is ROS 2?" → Expect: "Introduction to Robot Operating System (ROS 2)" chapter
    2. "How does Nav2 work?" → Expect: "Nav2 Navigation Stack" chapter
    3. "Explain URDF" → Expect: "URDF & XACRO for Robot Modeling" chapter
    4. "What are ROS nodes?" → Expect: "ROS 2 in Depth: Nodes, Topics, Services, Actions" chapter
    5. "How do sensors work in robots?" → Expect: "Robot Hardware: Sensors, Actuators, Control Systems" chapter
    6. "What is Isaac Sim?" → Expect: "NVIDIA Isaac Sim: Set Up & Fundamentals" chapter
    7. "Explain bipedal control" → Expect: "Motion Planning for Humanoids (Bipedal Control)" chapter
    8. "What is a vision-language-action pipeline?" → Expect: "Vision-Language-Action Pipelines (VLA)" chapter
    9. "How does SLAM work?" → Expect: "Mapping & Localization" chapter
    10. "What is Gazebo?" → Expect: "Gazebo Classic vs. Gazebo Garden (Ignition)" chapter

  **Metric 5 - Chunk Self-Containment**: ≥80% of sampled chunks understandable independently
  - **Test**: Randomly sample 10 chunks, manual review for context sufficiency
  - **Threshold**: ≥8 of 10 chunks include necessary context (key terms defined, topic clear)

  **Evidence**: RAG metrics report with pass/fail per metric + query result screenshots

- **SC-006**: Spec-Kit Plus validation returns ≥90% completeness score
  - **Verification**: Run `npx speckit validate --threshold 90` (or equivalent command)
  - **Threshold**: Completeness score ≥90%, zero critical errors, ≤3 warnings
  - **Measured by**:
    - All mandatory sections present (User Scenarios, Requirements, Success Criteria, etc.)
    - All functional requirements numbered sequentially (FR-001 to FR-020)
    - All success criteria include verification method
    - Zero contradictions between requirements detected
  - **Evidence**: Spec-Kit validation report showing score ≥90%

---

#### **RAG Chatbot Functionality**

- **SC-008**: Chatbot returns relevant answers with ≥1 citation for 8/10 test queries

  **Test Set** (10 domain-specific queries):
  1. "What is ROS 2?" → Must cite "Introduction to Robot Operating System (ROS 2)"
  2. "How do URDF and XACRO differ?" → Must cite "URDF & XACRO for Robot Modeling"
  3. "Explain the Nav2 navigation stack" → Must cite "Nav2 Navigation Stack"
  4. "What sensors are used in humanoid robots?" → Must cite "Robot Hardware: Sensors, Actuators, Control Systems"
  5. "How does Isaac Sim work?" → Must cite "NVIDIA Isaac Sim: Set Up & Fundamentals"
  6. "What is bipedal control?" → Must cite "Motion Planning for Humanoids (Bipedal Control)"
  7. "Explain vision-language-action pipelines" → Must cite "Vision-Language-Action Pipelines (VLA)"
  8. "How do ROS nodes communicate?" → Must cite "ROS 2 in Depth: Nodes, Topics, Services, Actions"
  9. "What is VSLAM?" → Must cite "Isaac ROS Perception Pipelines (VSLAM, AprilTags)"
  10. "How do you build a humanoid model?" → Must cite "Building a Humanoid Model in Gazebo"

  **Acceptance Criteria**:
  - ≥80% of queries (8/10) return answer + at least 1 citation
  - Citations include chapter title and section heading
  - Irrelevant queries (out-of-domain) return "I don't have information about that" instead of hallucinated answers
  - Response time p95 < 2 seconds

  **Verification**: Manual testing with screenshots of chatbot responses

  **Evidence**: Test results table showing query → answer → citation → pass/fail

- **SC-009**: Chatbot supports answering based on selected text with context preservation
  - **Test Procedure**:
    1. Open chapter "ROS 2 in Depth: Nodes, Topics, Services, Actions"
    2. Select text: "ROS 2 nodes communicate via topics using a publish-subscribe pattern"
    3. Click "Ask based on selected text" button
    4. Enter query: "How does this communication pattern work?"
    5. Verify response references "topics" and "publish-subscribe pattern" from selected text
  - **Threshold**: Response must explicitly reference selected text content (keyword match ≥50%)
  - **Evidence**: Video recording or screenshot sequence showing selection → query → response

---

#### **Authentication & Personalization**

- **SC-010**: User authentication flow completes successfully with session persistence

  **Test Flow** (5 steps):
  1. Navigate to signup page
  2. Enter email + password (test credentials)
  3. Submit form, receive success confirmation
  4. Sign in with same credentials
  5. Refresh page, verify session persists (user remains logged in)

  **Threshold**: All 5 steps complete without errors, session persists ≥10 minutes

  **Verification**: Manual testing with browser DevTools network tab

  **Evidence**: Screenshot sequence showing successful signup → signin → session persistence

- **SC-011**: Personalization button generates measurably different content for different profiles

  **Test Procedure**:
  1. Create 2 test user profiles:
     - Profile A: Skill level = "Beginner", Background = "No prior robotics experience"
     - Profile B: Skill level = "Advanced", Background = "5 years ROS development"
  2. Navigate to same chapter "Introduction to Physical AI" with each profile
  3. Click "Personalize this Chapter" button
  4. Capture personalized output for both profiles
  5. Run text diff comparison

  **Threshold**:
  - Personalization completes within 5 seconds
  - Text diff shows ≥20% character change between Profile A and Profile B outputs
  - Beginner profile includes more explanatory content (verified by word count increase ≥10%)

  **Verification**: Automated diff tool + manual review

  **Evidence**: Side-by-side comparison screenshots + diff report

---

#### **Translation**

- **SC-012**: Urdu translation produces valid output for ≥95% of chapters

  **Test Procedure**:
  1. Select 3 sample chapters (short ≤2000 words, medium 2000-4000 words, long ≥4000 words)
  2. Click "Translate to Urdu" button for each
  3. Verify output quality using automated checks

  **Acceptance Criteria** (all must pass):
  - ✅ Valid UTF-8 Urdu text (no encoding errors, no mojibake)
  - ✅ Preserved Markdown structure:
    - H2/H3 headers remain formatted (`## `, `### `)
    - Code blocks preserve language tags and content (```python)
    - Bullet lists remain formatted (- , 1. )
    - Links remain clickable
  - ✅ Translation completes within 10 seconds per chapter
  - ✅ "Toggle back to English" button restores original content without data loss

  **Threshold**: ≥95% success rate (23 of 25 chapters if testing all, or 3 of 3 sample chapters)

  **Verification**:
  - Automated: Run Markdown structure validator on translated output
  - Manual: Visual inspection of rendered Urdu content

  **Evidence**: Screenshots of 3 translated chapters + structure validation report

---

#### **Bonus Features**

- **SC-007**: At least 2 of the following bonus features are fully functional

  **Option A - Agent Skills**: ≥2 skills implemented with documented API
  - `personalize_for_user(content, user_profile)` → returns personalized version
  - `translate_to_urdu(content)` → returns Urdu UTF-8 text
  - `rewrite_content(text, style)` → returns rewritten version (≥10% text diff)
  - `summarize_for_embedding(chapter)` → returns ≤200 word summary

  **Option B - Personalization**: Button triggers measurable content adaptation
  - Test: Click button, observe ≥20% content change (verified by diff)
  - Profiles produce different outputs (beginner vs advanced)

  **Option C - Urdu Translation**: Button produces valid UTF-8 output preserving structure
  - Test: Translate chapter, verify Markdown structure intact
  - Verify UTF-8 encoding, no encoding errors

  **Threshold**: At least 2 options pass their respective tests

  **Evidence**: Feature demo video or screenshot sequence for each implemented option

- **SC-013**: At least 2 Claude Subagents OR 2 Agent Skills are functional and documented

  **Subagent Options** (choose ≥2):
  - **Code Review Agent**: Reviews ROS/Isaac code snippet, returns ≥3 actionable feedback items
  - **Chapter Writer Agent**: Generates chapter outline with Summary + ≥3 sections
  - **Simulation Diagnostics Agent**: Analyzes error log snippet, suggests ≥2 fixes
  - **Glossary Expander Agent**: Takes chapter, adds ≥5 glossary term definitions

  **Agent Skill Options** (choose ≥2):
  - **rewrite_content**: Takes text input (≥100 words), returns rewritten version (≥10% character diff)
  - **summarize_for_embedding**: Takes chapter (any length), returns ≤200 word summary
  - **personalize_for_user**: Takes content + user profile object, returns personalized version (measurable diff)
  - **translate_to_urdu**: Takes English text, returns valid UTF-8 Urdu output

  **Acceptance Criteria** (per agent/skill):
  - ✅ Documented usage example in README or docs
  - ✅ At least 1 passing test case demonstrating functionality
  - ✅ Error handling for invalid inputs (doesn't crash)

  **Threshold**: ≥2 subagents OR ≥2 skills meet all 3 acceptance criteria

  **Evidence**: Documentation page + test execution screenshots showing passing tests

---

### Summary Dashboard

**Required for Completion**: SC-001, SC-002, SC-003, SC-004, SC-005, SC-006, SC-008, SC-009, SC-010, SC-011, SC-012

**Bonus Points**: SC-007, SC-013

**Total**: 11 required + 2 bonus = 13 success criteria

## Assumptions

- The `/mnt/data/Hackathon I: Physical AI & Humanoid Robotics Textbook.docx` will be converted to a suitable digital format (e.g., Markdown) for chapter generation.
- Necessary API keys (`qdrant_api_key`, `openai_api_key`, `neon_connection_string`, `better_auth_env_config`, `github_repository_url`) will be provided and managed securely.
- The Docusaurus project structure will be initiated in the specified `docusaurus_project_folder` (`./`).
