# Physical AI & Humanoid Robotics - Deployment Guide

## 📦 What Has Been Implemented

### ✅ Configuration Complete

1. **Docusaurus 3.x with TypeScript** - Fully configured
2. **Course Branding** - Title, tagline, and navigation
3. **GitHub Pages Deployment** - Ready to deploy (needs username configuration)
4. **Syntax Highlighting** - Python, Bash, YAML, XML, C++ enabled
5. **Responsive Theme** - Dark/light mode with custom branding

### ✅ Content Created

#### Homepage
- **`docs/intro.md`** - Complete course overview (1800+ words)
  - 13-week timeline table
  - Learning outcomes
  - Prerequisites and setup instructions
  - Course philosophy and study tips

#### Module 1: Physical AI & ROS 2 Fundamentals (Partial)
- **`docs/module-01/intro.md`** - Module overview (1200+ words)
  - Learning outcomes
  - Weekly breakdown
  - Prerequisites
  - Required software installation

- **`docs/module-01/week-01-02-physical-ai-intro.md`** - Complete (2400+ words)
  - Physical AI vs Traditional AI
  - Embodied intelligence concepts
  - Comprehensive sensor systems guide (cameras, LIDAR, IMU, encoders)
  - Sensor fusion principles
  - Real-world applications
  - Hands-on exercises
  - Assessment questions

#### Directory Structure
- ✅ `docs/module-01/` - Physical AI & ROS 2
- ✅ `docs/module-02/` - Simulation
- ✅ `docs/module-03/` - NVIDIA Isaac
- ✅ `docs/module-04/` - Humanoid Development

---

## 🚀 Quick Start Guide

### 1. Local Development

```bash
# Navigate to project directory
cd C:\Users\FC\Documents\HACKATHON\ai-book

# Install dependencies (if not already done)
npm install

# Start development server
npm start
```

**Access**: Open browser to `http://localhost:3000/ai-book/`

The development server features:
- ✅ Hot reload - Changes appear instantly
- ✅ Error reporting - Build errors shown in browser
- ✅ Fast refresh - TypeScript compilation is fast

### 2. Building for Production

```bash
# Create optimized production build
npm run build
```

**Output**: Static files in `build/` directory

**Test Production Build Locally**:
```bash
npm run serve
```

Access at `http://localhost:3000/ai-book/`

---

## 🌐 GitHub Pages Deployment

### Step 1: Configure GitHub Settings

**Edit `docusaurus.config.ts`**:

Find and replace these lines (lines 18, 25, 108):

```typescript
// Line 18
url: 'https://YOUR_GITHUB_USERNAME.github.io',

// Line 25
organizationName: 'YOUR_GITHUB_USERNAME',

// Line 108
href: 'https://github.com/YOUR_GITHUB_USERNAME/ai-book',
```

Replace `YOUR_GITHUB_USERNAME` with your actual GitHub username.

**Example** (if your username is `johndoe`):
```typescript
url: 'https://johndoe.github.io',
organizationName: 'johndoe',
href: 'https://github.com/johndoe/ai-book',
```

### Step 2: Create GitHub Repository

1. Go to [github.com](https://github.com) and create a new repository:
   - Repository name: `ai-book`
   - Description: "Physical AI & Humanoid Robotics Textbook"
   - Public repository (required for free GitHub Pages)
   - DO NOT initialize with README, .gitignore, or license (already have these)

2. Push your code:

```bash
# Check git status
git status

# Add all files
git add .

# Commit
git commit -m "feat: Physical AI & Humanoid Robotics textbook - MVP"

# Add remote (replace YOUR_USERNAME)
git remote add origin https://github.com/YOUR_USERNAME/ai-book.git

# Push to GitHub
git branch -M main
git push -u origin main
```

### Step 3: Deploy to GitHub Pages

**Option A: Using npm deploy command (Recommended)**

```bash
# Set your GitHub username
SET GIT_USER=YOUR_GITHUB_USERNAME

# Deploy
npm run deploy
```

This will:
- Build the production site
- Push to `gh-pages` branch
- Automatically configure GitHub Pages

**Option B: Manual deployment**

```bash
# Build site
npm run build

# Install gh-pages if not already
npm install gh-pages --save-dev

# Deploy
npx gh-pages -d build -b gh-pages
```

### Step 4: Enable GitHub Pages

1. Go to your repository on GitHub
2. Click **Settings** → **Pages** (left sidebar)
3. Under **Build and deployment**:
   - Source: **Deploy from a branch**
   - Branch: **gh-pages** / **root**
   - Click **Save**

4. Wait 2-5 minutes for deployment

5. Access your site: `https://YOUR_USERNAME.github.io/ai-book/`

---

## 📁 Project Structure

```
ai-book/
├── docs/                          # Content files
│   ├── intro.md                   # Homepage ✅
│   ├── module-01/                 # Module 1 content
│   │   ├── intro.md              # Module intro ✅
│   │   ├── week-01-02-physical-ai-intro.md ✅
│   │   ├── week-03-05-ros2-fundamentals.md  ⏳ TODO
│   │   └── ros2-hands-on.md      ⏳ TODO
│   ├── module-02/                 # Module 2 (simulation) ⏳
│   ├── module-03/                 # Module 3 (Isaac) ⏳
│   ├── module-04/                 # Module 4 (humanoid) ⏳
│   ├── hardware.md                ⏳ TODO
│   ├── resources.md               ⏳ TODO
│   └── glossary.md                ⏳ TODO
├── src/                           # React components
│   ├── components/                # Custom React components
│   ├── css/                       # Styling
│   │   └── custom.css             # Custom styles
│   └── pages/                     # Custom pages (if needed)
├── static/                        # Static assets
│   └── img/                       # Images, logos
├── blog/                          # Blog posts (optional)
├── docusaurus.config.ts           # Main configuration ✅
├── sidebars.ts                    # Sidebar configuration ✅
├── package.json                   # Dependencies
├── tsconfig.json                  # TypeScript config
├── IMPLEMENTATION_SUMMARY.md      # Implementation guide ✅
└── DEPLOYMENT_GUIDE.md            # This file ✅
```

---

## 📝 Content Roadmap

### Immediate Priority (MVP Completion)

**Module 1 Completion** - Foundation for all other modules

1. **`docs/module-01/week-03-05-ros2-fundamentals.md`**
   - Estimated: 1500-2000 words
   - Topics: ROS 2 architecture, DDS, nodes, topics, services, actions, colcon
   - Include: 5-8 Python code examples

2. **`docs/module-01/ros2-hands-on.md`**
   - Estimated: 1200-1500 words
   - 5 complete exercises with full Python code
   - Clear success criteria for each

**Supporting Pages** - Required by all modules

3. **`docs/hardware.md`**
   - Hardware requirements table (3 tiers)
   - Version compatibility matrix
   - GPU requirements for Isaac Sim

4. **`docs/resources.md`**
   - External links (ROS 2, Gazebo, Unity, Isaac docs)
   - Community resources
   - Academic papers

5. **`docs/glossary.md`**
   - 40-50 technical terms with definitions
   - Alphabetically organized

### Medium Priority

**Module 2: Simulation (3 files)**
- `intro.md` - Module overview
- `gazebo-simulation.md` - Gazebo setup and URDF/SDF
- `unity-integration.md` - Unity Robotics Hub

**Module 3: NVIDIA Isaac (4 files)**
- `intro.md` - Module overview
- `isaac-platform.md` - Isaac SDK/Sim basics
- `ai-perception.md` - Computer vision and perception
- `reinforcement-learning.md` - RL and sim-to-real

### Lower Priority

**Module 4: Humanoid Development (5 files)**
- `intro.md` - Module overview
- `humanoid-basics.md` - Kinematics and dynamics
- `conversational-robotics.md` - GPT integration
- `vla-systems.md` - Vision-Language-Action
- `capstone-project.md` - Final project requirements

---

## 🛠️ Customization

### Changing Colors and Styles

**Edit `src/css/custom.css`**:

```css
:root {
  --ifm-color-primary: #2e8555;  /* Primary brand color */
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
  --ifm-color-primary-darkest: #205d3b;
  --ifm-color-primary-light: #33925d;
  --ifm-color-primary-lighter: #359962;
  --ifm-color-primary-lightest: #3cad6e;
  --ifm-code-font-size: 95%;
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.1);
}
```

Adjust these values for custom branding.

### Adding a Logo

1. Place logo image in `static/img/logo.svg` (or `.png`)
2. Logo is already configured in `docusaurus.config.ts` (line 80-83)

### Adding Favicon

Replace `static/img/favicon.ico` with your custom icon.

---

## 🧪 Testing Checklist

Before deploying, verify:

- [ ] `npm start` runs without errors
- [ ] Homepage loads with course overview
- [ ] Module 1 intro page loads
- [ ] Week 1-2 content displays correctly
- [ ] Sidebar navigation works
- [ ] Dark/light theme toggle works
- [ ] Code blocks have syntax highlighting
- [ ] Admonitions (tips, warnings) render correctly
- [ ] Tables display properly
- [ ] Links to hardware, resources, glossary work (or will after those pages are created)
- [ ] Mobile view is responsive

**Run Tests**:
```bash
npm run build  # Should complete without errors
npm run serve  # Test production build locally
```

---

## 🐛 Troubleshooting

### Build Errors

**Problem**: `Module not found` errors

**Solution**:
```bash
rm -rf node_modules package-lock.json
npm install
```

**Problem**: TypeScript errors

**Solution**: Check `tsconfig.json` and ensure all imported modules exist

### Broken Links

**Problem**: Links to non-existent pages show 404

**Solution**: Either create the page or comment out the link temporarily

**Temporary Fix in `docusaurus.config.ts`**:
```typescript
onBrokenLinks: 'warn',  // Change from 'throw' to 'warn'
```

### Deployment Fails

**Problem**: `gh-pages` deployment fails

**Solution**:
```bash
# Ensure git user is set
git config user.name "Your Name"
git config user.email "your@email.com"

# Try deployment again
GIT_USER=YOUR_USERNAME npm run deploy
```

### Port Already in Use

**Problem**: `npm start` fails because port 3000 is occupied

**Solution**:
```bash
# Kill process on port 3000 (Windows)
netstat -ano | findstr :3000
taskkill /PID <PID> /F

# Or use different port
npm start -- --port 3001
```

---

## 📊 Current Implementation Status

| Component | Status | Completion % |
|-----------|--------|--------------|
| **Configuration** | ✅ Complete | 100% |
| **Homepage** | ✅ Complete | 100% |
| **Module 1 Intro** | ✅ Complete | 100% |
| **Module 1 Week 1-2** | ✅ Complete | 100% |
| **Module 1 Week 3-5** | ⏳ TODO | 0% |
| **Module 1 Hands-on** | ⏳ TODO | 0% |
| **Module 2** | ⏳ TODO | 0% |
| **Module 3** | ⏳ TODO | 0% |
| **Module 4** | ⏳ TODO | 0% |
| **Hardware Page** | ⏳ TODO | 0% |
| **Resources Page** | ⏳ TODO | 0% |
| **Glossary** | ⏳ TODO | 0% |
| **Overall** | 🚧 In Progress | ~25% |

---

## 📞 Support Resources

- **Docusaurus Docs**: https://docusaurus.io/docs
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **NVIDIA Isaac Docs**: https://docs.omniverse.nvidia.com/isaacsim/
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub

---

## 🎯 Next Steps

1. **Test the current implementation**:
   ```bash
   npm start
   ```
   Verify homepage and Module 1 content display correctly

2. **Complete Module 1** (highest priority):
   - Write ROS 2 fundamentals chapter
   - Create hands-on exercises

3. **Add supporting pages**:
   - Hardware requirements
   - Resources
   - Glossary

4. **Deploy MVP to GitHub Pages**:
   - Configure GitHub username
   - Push to repository
   - Enable GitHub Pages
   - Share link for feedback

5. **Continue with Modules 2-4** based on priority

---

**Last Updated**: 2025-12-07
**Version**: 1.0.0-MVP
**Status**: Ready for local testing and MVP deployment
