# Physical AI & Humanoid Robotics Textbook

An AI-native, interactive textbook for learning Physical AI and Humanoid Robotics. Built with Docusaurus, featuring an integrated RAG chatbot, personalization, and multilingual support.

## Live Demo

Visit the deployed textbook: [https://Aiman-17.github.io/hackathon_spec_kit_book/](https://Aiman-17.github.io/hackathon_spec_kit_book/)

## Features

- **24 Comprehensive Chapters** across 4 modules covering Physical AI fundamentals to advanced humanoid systems
- **AI-Powered RAG Chatbot** - Ask questions about any concept with context-aware responses
- **Interactive Learning** - Hands-on code examples, exercises, and visualizations
- **Modern UI** - Beautiful, responsive design with dark mode support
- **Clean URLs** - SEO-friendly slugs without number prefixes
- **Personalization** (Coming Soon) - Content adapted to your skill level
- **Urdu Translation** (Coming Soon) - Accessible to Urdu-speaking learners

## Course Structure

### Module 1: Foundations of Physical AI & Robotics
- Introduction to Physical AI
- Robot Hardware: Sensors, Actuators, Control Systems
- Mathematics for Robotics (Kinematics & Dynamics)
- Introduction to Robot Operating System (ROS 2)
- Linear Algebra & Calculus for Robot Motion

### Module 2: Simulation Environments & Robotics Software
- ROS 2 in Depth: Nodes, Topics, Services, Actions
- Building Robot Applications with ROS Packages
- URDF & XACRO for Robot Modeling
- Gazebo Classic vs. Gazebo Garden
- Building a Humanoid Model in Gazebo
- Unity for Robotics Visualization & HRI

### Module 3: Advanced Perception, Navigation & Control
- Computer Vision for Robotics
- NVIDIA Isaac Sim: Setup & Fundamentals
- Isaac ROS Perception Pipelines
- Nav2 Navigation Stack
- Mapping & Localization
- Motion Planning for Humanoids
- Vision-Language-Action Pipelines

### Module 4: Humanoid AI Systems & Capstone
- Integrating Perception, Action & Control
- AI Agents for Autonomous Robotics
- End-to-End Humanoid Pipeline
- Multi-Agent Coordination
- Project: Build an Autonomous Humanoid Simulation
- Final Capstone: Full Humanoid Robotics System

## Prerequisites

### Required Software
- **Node.js** v18.0 or higher
- **npm** v9.0 or higher (comes with Node.js)
- **Python** 3.9 or higher (for backend services)
- **Git** for version control

### Optional (for backend/chatbot features)
- **PostgreSQL** 14 or higher
- **Qdrant** vector database (can run in Docker)
- **OpenAI API Key** (for embeddings and chat)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/Aiman-17/hackathon_spec_kit_book.git
cd hackathon_spec_kit_book
```

### 2. Install Frontend Dependencies

```bash
cd frontend
npm install
```

### 3. Install Backend Dependencies (Optional)

```bash
cd ../backend
pip install -r requirements.txt
```

### 4. Configure Environment Variables (Optional)

If you want to run the RAG chatbot locally:

```bash
cd backend
cp .env.example .env
# Edit .env and add your API keys
```

Required environment variables:
```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_HOST=localhost
QDRANT_PORT=6333
DATABASE_URL=postgresql://user:password@localhost:5432/textbook_db
```

## Build Instructions

### Development Mode

Run the development server with hot reload:

```bash
cd frontend
npm start
```

The site will be available at `http://localhost:3000`

### Production Build

Build the static site for production:

```bash
cd frontend
npm run build
```

The production-ready files will be generated in the `frontend/build` directory.

### Test Production Build Locally

Serve the production build locally to test:

```bash
cd frontend
npm run serve
```

### Build with Clean Cache

If you encounter build issues, clean the cache first:

```bash
cd frontend
rm -rf build .docusaurus node_modules/.cache
npm run build
```

## Deployment

### Deploy to GitHub Pages

The project is configured to deploy to GitHub Pages automatically:

```bash
cd frontend
GIT_USER=<your-github-username> npm run deploy
```

This will:
1. Build the production site
2. Push to the `gh-pages` branch
3. GitHub Pages will automatically deploy from that branch

### Manual Deployment

You can also deploy to any static hosting service:

1. Build the site: `npm run build`
2. Upload the `frontend/build` directory to your hosting provider

Supported platforms:
- GitHub Pages (configured)
- Vercel
- Netlify
- AWS S3 + CloudFront
- Azure Static Web Apps

## Project Structure

```
hackathon_spec_kit_book/
├── frontend/                 # Docusaurus frontend
│   ├── docs/                # Markdown content
│   │   ├── intro.md        # Landing page
│   │   ├── module-1/       # Module 1 chapters
│   │   ├── module-2/       # Module 2 chapters
│   │   ├── module-3/       # Module 3 chapters
│   │   └── module-4/       # Module 4 chapters
│   ├── src/
│   │   ├── components/     # React components
│   │   └── theme/          # Custom theme files
│   ├── docusaurus.config.js # Docusaurus configuration
│   ├── sidebars.js         # Sidebar navigation
│   └── package.json        # Frontend dependencies
├── backend/                 # FastAPI backend (optional)
│   ├── src/
│   │   ├── api/           # API routes
│   │   ├── services/      # Business logic
│   │   └── models/        # Data models
│   ├── scripts/           # Utility scripts
│   └── requirements.txt   # Python dependencies
├── scripts/                # Build and utility scripts
│   ├── generate-chapters.py
│   └── validate-content.py
├── add_slugs.py           # Script to add custom URL slugs
└── README.md              # This file
```

## Development Workflow

### Adding New Chapters

1. Create a new markdown file in the appropriate module directory:
   ```bash
   # Example: Adding a new chapter to Module 1
   cd frontend/docs/module-1
   touch 07-new-chapter.md
   ```

2. Add frontmatter to the file:
   ```markdown
   ---
   title: Your Chapter Title
   slug: your-chapter-slug
   sidebar_position: 7
   description: "Brief description"
   tags: [tag1, tag2]
   ---

   # Your Chapter Title

   Your content here...
   ```

3. Update `frontend/sidebars.js` to include the new chapter:
   ```javascript
   {
     type: 'category',
     label: 'Module 1: Foundations',
     items: [
       'module-1/introduction-to-physical-ai',
       // ... other chapters
       'module-1/your-chapter-slug',  // Add this line
     ],
   }
   ```

### Running Backend Services (Optional)

1. Start Qdrant (using Docker):
   ```bash
   docker run -p 6333:6333 qdrant/qdrant
   ```

2. Start PostgreSQL (or use existing instance)

3. Run the FastAPI backend:
   ```bash
   cd backend
   python -m src.main
   ```

4. Ingest chapters into the vector database:
   ```bash
   cd backend
   python scripts/ingest_all_chapters.py
   ```

## Customization

### Changing Theme Colors

Edit `frontend/src/theme/custom.css`:

```css
:root {
  --ifm-color-primary: #7C3AED;  /* Change primary color */
  --ifm-navbar-background-color: #1a1a1a;
  /* Add more custom styles */
}
```

### Modifying Navigation

Edit `frontend/docusaurus.config.js`:

```javascript
navbar: {
  title: 'Your Custom Title',
  items: [
    // Add custom navbar items
  ],
}
```

### Adding Custom Components

Create React components in `frontend/src/components/` and import them in your markdown files.

## Troubleshooting

### Build Errors

**Problem**: Module not found errors
```bash
# Solution: Clean install
rm -rf node_modules package-lock.json
npm install
```

**Problem**: Broken links warning
```bash
# Solution: Check that all internal links use correct slugs
# Run build to see which links are broken
npm run build
```

### Deployment Issues

**Problem**: gh-pages branch doesn't exist
```bash
# Solution: Create the branch first
git checkout --orphan gh-pages
git rm -rf .
git commit --allow-empty -m "Initial gh-pages commit"
git push origin gh-pages
git checkout main
```

**Problem**: 404 on GitHub Pages
- Check that `baseUrl` in `docusaurus.config.js` matches your repository name
- Ensure GitHub Pages is enabled in repository settings

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Make your changes
4. Test locally: `npm run build`
5. Commit your changes: `git commit -m "Add your feature"`
6. Push to your fork: `git push origin feature/your-feature`
7. Create a Pull Request

## Technology Stack

- **Frontend**: Docusaurus 3.x, React 18
- **Backend**: FastAPI, Python 3.9+
- **Database**: PostgreSQL 14+
- **Vector DB**: Qdrant
- **AI**: OpenAI GPT-4, OpenAI Embeddings
- **Deployment**: GitHub Pages
- **Build Tool**: npm, Webpack

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Built with Docusaurus by Meta
- AI-powered content generation using OpenAI GPT-4
- Vector search powered by Qdrant
- Inspired by modern educational platforms and AI-native learning

## Contact

For questions or support:
- GitHub Issues: [Create an issue](https://github.com/Aiman-17/hackathon_spec_kit_book/issues)
- Repository: [github.com/Aiman-17/hackathon_spec_kit_book](https://github.com/Aiman-17/hackathon_spec_kit_book)

---

Built with love for the robotics and AI community
