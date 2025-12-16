import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

const MobileNavSidebar = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [expandedModules, setExpandedModules] = useState({});
  const { siteConfig } = useDocusaurusContext();

  // Close sidebar when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (isOpen && !event.target.closest(`.${styles.sidebar}`) && !event.target.closest(`.${styles.hamburger}`)) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [isOpen]);

  // Prevent body scroll when sidebar is open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }
    return () => {
      document.body.style.overflow = '';
    };
  }, [isOpen]);

  const toggleModule = (moduleIndex) => {
    setExpandedModules(prev => ({
      ...prev,
      [moduleIndex]: !prev[moduleIndex]
    }));
  };

  const closeSidebar = () => {
    setIsOpen(false);
  };

  // Sidebar navigation structure (mirrors sidebars.js)
  const navigationStructure = [
    {
      label: 'Introduction',
      path: '/docs/intro',
    },
    {
      label: 'Module 1: Foundations of Physical AI & Robotics',
      items: [
        { label: 'Introduction to Physical AI', path: '/docs/module-1/introduction-to-physical-ai' },
        { label: 'Robot Hardware Fundamentals', path: '/docs/module-1/robot-hardware' },
        { label: 'Mathematics for Robotics', path: '/docs/module-1/mathematics-for-robotics' },
        { label: 'Introduction to ROS 2', path: '/docs/module-1/introduction-to-ros2' },
        { label: 'Linear Algebra & Calculus', path: '/docs/module-1/linear-algebra-calculus' },
      ],
    },
    {
      label: 'Module 2: Simulation Environments & Robotics Software',
      items: [
        { label: 'ROS 2 In-Depth', path: '/docs/module-2/ros2-in-depth' },
        { label: 'Building ROS Applications', path: '/docs/module-2/building-ros-applications' },
        { label: 'URDF & Xacro Modeling', path: '/docs/module-2/urdf-xacro-modeling' },
        { label: 'Gazebo Comparison', path: '/docs/module-2/gazebo-comparison' },
        { label: 'Humanoid Model in Gazebo', path: '/docs/module-2/humanoid-model-gazebo' },
        { label: 'Unity Robotics', path: '/docs/module-2/unity-robotics' },
      ],
    },
    {
      label: 'Module 3: Advanced Perception, Navigation & Control',
      items: [
        { label: 'Computer Vision', path: '/docs/module-3/computer-vision' },
        { label: 'Isaac Sim Fundamentals', path: '/docs/module-3/isaac-sim-fundamentals' },
        { label: 'Isaac ROS Perception', path: '/docs/module-3/isaac-ros-perception' },
        { label: 'Nav2 Navigation', path: '/docs/module-3/nav2-navigation' },
        { label: 'Mapping & Localization', path: '/docs/module-3/mapping-localization' },
        { label: 'Motion Planning for Humanoids', path: '/docs/module-3/motion-planning-humanoids' },
        { label: 'Vision-Language-Action Models', path: '/docs/module-3/vision-language-action' },
      ],
    },
    {
      label: 'Module 4: Humanoid AI Systems & Capstone Development',
      items: [
        { label: 'Integrating Perception, Action & Control', path: '/docs/module-4/integrating-perception-action-control' },
        { label: 'AI Agents in Robotics', path: '/docs/module-4/ai-agents-robotics' },
        { label: 'End-to-End Humanoid Pipeline', path: '/docs/module-4/end-to-end-humanoid-pipeline' },
        { label: 'Multi-Agent Coordination', path: '/docs/module-4/multi-agent-coordination' },
        { label: 'Project: Autonomous Humanoid', path: '/docs/module-4/project-autonomous-humanoid' },
        { label: 'Final Capstone', path: '/docs/module-4/final-capstone' },
      ],
    },
  ];

  return (
    <>
      {/* Hamburger Menu Button */}
      <button
        className={styles.hamburger}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle navigation menu"
      >
        <div className={`${styles.hamburgerLine} ${isOpen ? styles.open : ''}`}></div>
        <div className={`${styles.hamburgerLine} ${isOpen ? styles.open : ''}`}></div>
        <div className={`${styles.hamburgerLine} ${isOpen ? styles.open : ''}`}></div>
      </button>

      {/* Backdrop */}
      {isOpen && <div className={styles.backdrop} onClick={closeSidebar}></div>}

      {/* Sidebar */}
      <nav className={`${styles.sidebar} ${isOpen ? styles.open : ''}`}>
        <div className={styles.sidebarHeader}>
          <h2 className={styles.sidebarTitle}>📚 Course Navigation</h2>
          <button
            className={styles.closeButton}
            onClick={closeSidebar}
            aria-label="Close navigation menu"
          >
            ✕
          </button>
        </div>

        <div className={styles.sidebarContent}>
          {navigationStructure.map((item, index) => (
            <div key={index} className={styles.navSection}>
              {item.items ? (
                // Module with chapters
                <>
                  <button
                    className={styles.moduleHeader}
                    onClick={() => toggleModule(index)}
                  >
                    <span className={styles.moduleLabel}>{item.label}</span>
                    <span className={`${styles.expandIcon} ${expandedModules[index] ? styles.expanded : ''}`}>
                      ▶
                    </span>
                  </button>
                  {expandedModules[index] && (
                    <ul className={styles.chapterList}>
                      {item.items.map((chapter, chapterIndex) => (
                        <li key={chapterIndex} className={styles.chapterItem}>
                          <Link
                            to={`${siteConfig.baseUrl}${chapter.path.replace('/docs/', 'docs/')}`}
                            className={styles.chapterLink}
                            onClick={closeSidebar}
                          >
                            {chapter.label}
                          </Link>
                        </li>
                      ))}
                    </ul>
                  )}
                </>
              ) : (
                // Single page (Introduction)
                <Link
                  to={`${siteConfig.baseUrl}${item.path.replace('/docs/', 'docs/')}`}
                  className={styles.singlePageLink}
                  onClick={closeSidebar}
                >
                  {item.label}
                </Link>
              )}
            </div>
          ))}
        </div>

        <div className={styles.sidebarFooter}>
          <p className={styles.footerText}>Physical AI & Humanoid Robotics</p>
          <p className={styles.footerSubtext}>Interactive Textbook</p>
        </div>
      </nav>
    </>
  );
};

export default MobileNavSidebar;
