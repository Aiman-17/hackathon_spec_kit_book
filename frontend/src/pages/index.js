import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1 className={styles.heroTitle}>
              Welcome to Physical AI & Humanoid Robotics
            </h1>
            <p className={styles.heroSubtitle}>
              Build Intelligent Robotic Systems from the Ground Up
            </p>
            <p className={styles.heroDescription}>
              Master the art of building intelligent, embodied systems with this comprehensive,
              AI-native textbook on Physical AI and Humanoid Robotics. Get interactive AI assistance
              as you learn.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Start Reading
              </Link>
            </div>
            <div className={styles.heroStats}>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>4</div>
                <div className={styles.statLabel}>Modules</div>
              </div>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>24</div>
                <div className={styles.statLabel}>Chapters</div>
              </div>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>AI</div>
                <div className={styles.statLabel}>Powered</div>
              </div>
            </div>
          </div>
          <div className={styles.heroImage}>
            <div className={styles.robotIllustration}>
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
    </header>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featureGrid}>
          <div className={styles.feature}>
            <h3>📘 Comprehensive Curriculum</h3>
            <p>
              4 modules covering everything from ROS 2 fundamentals to advanced AI integration,
              with 24 detailed chapters and hands-on exercises.
            </p>
          </div>
          <div className={styles.feature}>
            <h3>🤖 AI-Powered Learning</h3>
            <p>
              Integrated RAG chatbot provides instant answers from the textbook content,
              with source citations and context-aware responses.
            </p>
          </div>
          <div className={styles.feature}>
            <h3>🎯 Hands-On Projects</h3>
            <p>
              Build real robotic systems with NVIDIA Isaac Sim, Gazebo, Unity, and ROS 2.
              Complete capstone projects to demonstrate mastery.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="AI-Native Textbook for Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
