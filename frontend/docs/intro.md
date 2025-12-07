---
id: intro
title: Welcome to Physical AI & Humanoid Robotics
sidebar_position: 0
---

import styles from './intro.module.css';

<div className="hero-banner">
  <div className="hero-content">
    <div className="hero-text">
      <div className="hero-logo">
        <svg width="60" height="60" viewBox="0 0 60 60" fill="none" xmlns="http://www.w3.org/2000/svg">
          <circle cx="30" cy="30" r="28" fill="#7C3AED" opacity="0.1"/>
          <path d="M30 10C18.954 10 10 18.954 10 30C10 41.046 18.954 50 30 50C41.046 50 50 41.046 50 30C50 18.954 41.046 10 30 10ZM30 45C21.716 45 15 38.284 15 30C15 21.716 21.716 15 30 15C38.284 15 45 21.716 45 30C45 38.284 38.284 45 30 45Z" fill="#7C3AED"/>
          <circle cx="25" cy="25" r="3" fill="#7C3AED"/>
          <circle cx="35" cy="25" r="3" fill="#7C3AED"/>
          <path d="M30 32C27 32 25 34 25 36H35C35 34 33 32 30 32Z" fill="#7C3AED"/>
          <rect x="28" y="30" width="4" height="8" fill="#7C3AED" opacity="0.5"/>
        </svg>
      </div>
      <h1 className="hero-title">Physical AI & Humanoid Robotics</h1>
      <p className="hero-subtitle">Build Intelligent Robotic Systems from the Ground Up</p>
      <p className="hero-description">
        Welcome to the comprehensive, AI-native textbook on Physical AI and Humanoid Robotics.
        Master the art of building intelligent, embodied systems with interactive AI assistance.
      </p>
      <div className="hero-buttons">
        <a href="/docs/module-1/01-introduction-to-physical-ai" className="btn btn-primary">
          Start Learning
          <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
            <path d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"/>
          </svg>
        </a>
        <a href="#course-structure" className="btn btn-secondary">
          Explore Modules
        </a>
      </div>
      <div className="hero-stats">
        <div className="stat-item">
          <div className="stat-number">4</div>
          <div className="stat-label">Modules</div>
        </div>
        <div className="stat-item">
          <div className="stat-number">25</div>
          <div className="stat-label">Chapters</div>
        </div>
        <div className="stat-item">
          <div className="stat-number">AI</div>
          <div className="stat-label">Powered</div>
        </div>
      </div>
    </div>
    <div className="hero-image">
      <div className="robot-illustration">
        <svg viewBox="0 0 400 400" fill="none" xmlns="http://www.w3.org/2000/svg">
          {/* Robot Body */}
          <rect x="150" y="180" width="100" height="120" rx="8" fill="url(#grad1)" stroke="#7C3AED" strokeWidth="3"/>
          {/* Robot Head */}
          <rect x="160" y="120" width="80" height="70" rx="12" fill="url(#grad2)" stroke="#7C3AED" strokeWidth="3"/>
          {/* Eyes */}
          <circle cx="180" cy="145" r="8" fill="#60A5FA"/>
          <circle cx="220" cy="145" r="8" fill="#60A5FA"/>
          {/* Antenna */}
          <line x1="200" y1="120" x2="200" y2="100" stroke="#7C3AED" strokeWidth="3" strokeLinecap="round"/>
          <circle cx="200" cy="95" r="6" fill="#F59E0B"/>
          {/* Arms */}
          <rect x="120" y="200" width="20" height="60" rx="4" fill="url(#grad1)" stroke="#7C3AED" strokeWidth="2"/>
          <rect x="260" y="200" width="20" height="60" rx="4" fill="url(#grad1)" stroke="#7C3AED" strokeWidth="2"/>
          {/* Legs */}
          <rect x="165" y="300" width="25" height="60" rx="4" fill="url(#grad1)" stroke="#7C3AED" strokeWidth="2"/>
          <rect x="210" y="300" width="25" height="60" rx="4" fill="url(#grad1)" stroke="#7C3AED" strokeWidth="2"/>
          {/* Gradients */}
          <defs>
            <linearGradient id="grad1" x1="0%" y1="0%" x2="0%" y2="100%">
              <stop offset="0%" style={{stopColor:'#A78BFA', stopOpacity:0.8}} />
              <stop offset="100%" style={{stopColor:'#7C3AED', stopOpacity:0.9}} />
            </linearGradient>
            <linearGradient id="grad2" x1="0%" y1="0%" x2="100%" y2="100%">
              <stop offset="0%" style={{stopColor:'#C4B5FD', stopOpacity:0.9}} />
              <stop offset="100%" style={{stopColor:'#8B5CF6', stopOpacity:0.9}} />
            </linearGradient>
          </defs>
          {/* Floating particles */}
          <circle cx="80" cy="150" r="4" fill="#60A5FA" opacity="0.6">
            <animateTransform attributeName="transform" type="translate" values="0,0; 0,-10; 0,0" dur="3s" repeatCount="indefinite"/>
          </circle>
          <circle cx="320" cy="200" r="3" fill="#F59E0B" opacity="0.6">
            <animateTransform attributeName="transform" type="translate" values="0,0; 0,10; 0,0" dur="4s" repeatCount="indefinite"/>
          </circle>
          <circle cx="100" cy="280" r="5" fill="#7C3AED" opacity="0.4">
            <animateTransform attributeName="transform" type="translate" values="0,0; 0,-15; 0,0" dur="5s" repeatCount="indefinite"/>
          </circle>
        </svg>
      </div>
    </div>
  </div>
</div>

:::tip AI-Powered Learning
This textbook features an integrated RAG chatbot that can answer your questions based on the course content. Look for the purple chat widget in the bottom-right corner!
:::

## What You'll Learn

This textbook provides a complete journey through the foundations, techniques, and advanced applications of Physical AI and humanoid robotics. You'll gain hands-on experience with:

- **Robot Operating System (ROS 2)** - Industry-standard framework for robotics development
- **Simulation Environments** - NVIDIA Isaac Sim, Gazebo, and Unity for testing and validation
- **Perception & Navigation** - Computer vision, SLAM, sensor fusion, and autonomous navigation
- **Advanced Control** - Motion planning, bipedal locomotion, and manipulation strategies
- **AI Integration** - Vision-Language-Action pipelines, autonomous agents, and multi-agent systems

<div id="course-structure"></div>

## Course Structure

This course is organized into **4 comprehensive modules** with **25 hands-on chapters**:

### 📘 Module 1: Foundations of Physical AI & Robotics

Build your foundation in Physical AI, robotics hardware, mathematics, and ROS 2 fundamentals.

**Chapters:**
- Introduction to Physical AI
- Humanoid Robotics Overview
- Robot Hardware: Sensors, Actuators, Control Systems
- Mathematics for Robotics (Kinematics & Dynamics)
- Introduction to Robot Operating System (ROS 2)
- Linear Algebra & Calculus for Robot Motion

### 🔧 Module 2: Simulation Environments & Robotics Software

Master simulation tools and advanced ROS 2 development for building robust robotic applications.

**Chapters:**
- ROS 2 in Depth: Nodes, Topics, Services, Actions
- Building Robot Applications with ROS Packages
- URDF & XACRO for Robot Modeling
- Gazebo Classic vs. Gazebo Garden (Ignition)
- Building a Humanoid Model in Gazebo
- Unity for Robotics Visualization & HRI

### 🎯 Module 3: Advanced Perception, Navigation & Control

Dive deep into computer vision, navigation stacks, motion planning, and cutting-edge perception pipelines.

**Chapters:**
- Computer Vision for Robotics
- NVIDIA Isaac Sim: Set Up & Fundamentals
- Isaac ROS Perception Pipelines (VSLAM, AprilTags)
- Nav2 Navigation Stack
- Mapping & Localization
- Motion Planning for Humanoids (Bipedal Control)
- Vision-Language-Action Pipelines (VLA)

### 🤖 Module 4: Humanoid AI Systems & Capstone Development

Integrate perception, action, and control into complete autonomous systems and build your capstone project.

**Chapters:**
- Integrating Perception, Action & Control
- AI Agents for Autonomous Robotics
- End-to-End Humanoid Pipeline: Sensing → Reasoning → Acting
- Multi-Agent Coordination for Robotics
- Project: Build an Autonomous Humanoid Simulation
- Final Capstone: Full Humanoid Robotics System

## Prerequisites

This course is designed for advanced undergraduate students or professionals with:

- **Programming**: Proficiency in Python (required), familiarity with C++ (helpful)
- **Mathematics**: Linear algebra, calculus, basic probability and statistics
- **Computer Science**: Data structures, algorithms, object-oriented programming
- **Robotics**: Introductory robotics concepts (helpful but not required)

## Learning Approach

Each chapter follows a structured pedagogy:

1. **Summary** - Overview of key concepts
2. **Learning Objectives** - Clear, measurable outcomes
3. **Prerequisites** - Required background knowledge
4. **Core Content** - Detailed explanations with diagrams and examples
5. **Practical Examples** - ROS 2 code snippets and hands-on exercises
6. **Key Takeaways** - Summary of main points
7. **Glossary** - Technical terms and definitions
8. **Review Questions** - Test your understanding

## How to Use This Textbook

### For Self-Learners
- Progress through modules sequentially
- Complete all code examples and exercises
- Use the AI chatbot to clarify concepts
- Build the capstone project to apply your knowledge

### For Instructors
- Use as primary textbook for robotics courses
- Assign chapters as weekly readings
- Leverage practical exercises for lab sessions
- Adapt capstone projects to your curriculum

### Interactive Features

**🤖 RAG Chatbot**
- Ask questions about any concept
- Get explanations with citations
- Select text and ask context-aware questions

**💬 Personalization (Coming Soon)**
- Content adapted to your skill level
- Learning paths tailored to your goals

**🌐 Urdu Translation (Coming Soon)**
- Full chapter translations
- Accessible to Urdu-speaking learners

---

## About This Project

This textbook is built with:
- **Docusaurus** - Modern documentation framework
- **OpenAI GPT-4** - AI-powered content generation
- **Qdrant** - Vector database for RAG chatbot
- **FastAPI** - Backend services
- **ROS 2** - Robotics framework

*Built with ❤️ for the robotics and AI community*
