import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

/**
 * PersonalizationPanel Component
 *
 * Allows users to set preferences without authentication:
 * - User Level: student / beginner / advanced
 * - Language: English (en) / Urdu (ur)
 *
 * Preferences are stored in localStorage and passed to backend with chat queries.
 */
const PersonalizationPanel = ({ isOpen, onClose }) => {
  const [userLevel, setUserLevel] = useState('student');
  const [language, setLanguage] = useState('en');
  const [isSaved, setIsSaved] = useState(false);

  // Load preferences from localStorage on mount
  useEffect(() => {
    try {
      const savedLevel = localStorage.getItem('userLevel');
      const savedLanguage = localStorage.getItem('language');

      if (savedLevel) setUserLevel(savedLevel);
      if (savedLanguage) setLanguage(savedLanguage);
    } catch (error) {
      console.error('Failed to load preferences:', error);
    }
  }, []);

  // Save preferences to localStorage
  const handleSave = () => {
    try {
      localStorage.setItem('userLevel', userLevel);
      localStorage.setItem('language', language);

      setIsSaved(true);
      setTimeout(() => setIsSaved(false), 2000);

      // Dispatch custom event so ChatWidget can update
      window.dispatchEvent(new CustomEvent('preferencesChanged', {
        detail: { userLevel, language }
      }));
    } catch (error) {
      console.error('Failed to save preferences:', error);
      alert('Failed to save preferences. Please check your browser settings.');
    }
  };

  const handleReset = () => {
    setUserLevel('student');
    setLanguage('en');
    localStorage.removeItem('userLevel');
    localStorage.removeItem('language');

    window.dispatchEvent(new CustomEvent('preferencesChanged', {
      detail: { userLevel: 'student', language: 'en' }
    }));
  };

  if (!isOpen) return null;

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div className={styles.panel} onClick={(e) => e.stopPropagation()}>
        <div className={styles.header}>
          <h3 className={styles.title}>⚙️ Personalization Settings</h3>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close settings">
            ✕
          </button>
        </div>

        <div className={styles.content}>
          {/* User Level Selection */}
          <div className={styles.section}>
            <label className={styles.label}>
              <span className={styles.labelIcon}>👤</span>
              Your Learning Level
            </label>
            <p className={styles.description}>
              Choose your experience level. This adjusts answer depth and technical detail.
            </p>
            <div className={styles.radioGroup}>
              <label className={styles.radioLabel}>
                <input
                  type="radio"
                  name="userLevel"
                  value="student"
                  checked={userLevel === 'student'}
                  onChange={(e) => setUserLevel(e.target.value)}
                  className={styles.radioInput}
                />
                <span className={styles.radioText}>
                  <strong>Student</strong> - New to robotics, clear explanations
                </span>
              </label>

              <label className={styles.radioLabel}>
                <input
                  type="radio"
                  name="userLevel"
                  value="beginner"
                  checked={userLevel === 'beginner'}
                  onChange={(e) => setUserLevel(e.target.value)}
                  className={styles.radioInput}
                />
                <span className={styles.radioText}>
                  <strong>Beginner</strong> - Some technical knowledge, balanced detail
                </span>
              </label>

              <label className={styles.radioLabel}>
                <input
                  type="radio"
                  name="userLevel"
                  value="advanced"
                  checked={userLevel === 'advanced'}
                  onChange={(e) => setUserLevel(e.target.value)}
                  className={styles.radioInput}
                />
                <span className={styles.radioText}>
                  <strong>Advanced</strong> - Technical background, concise answers
                </span>
              </label>
            </div>
          </div>

          {/* Language Selection */}
          <div className={styles.section}>
            <label className={styles.label}>
              <span className={styles.labelIcon}>🌐</span>
              Response Language
            </label>
            <p className={styles.description}>
              Choose the language for AI tutor responses. Textbook content remains in English.
            </p>
            <div className={styles.toggleGroup}>
              <button
                className={`${styles.toggleButton} ${language === 'en' ? styles.toggleActive : ''}`}
                onClick={() => setLanguage('en')}
                aria-pressed={language === 'en'}>
                🇺🇸 English
              </button>
              <button
                className={`${styles.toggleButton} ${language === 'ur' ? styles.toggleActive : ''}`}
                onClick={() => setLanguage('ur')}
                aria-pressed={language === 'ur'}>
                🇵🇰 اردو (Urdu)
              </button>
            </div>
            {language === 'ur' && (
              <p className={styles.note}>
                ℹ️ Translation applies to AI responses only. Retrieval and textbook content stay in English.
              </p>
            )}
          </div>

          {/* Action Buttons */}
          <div className={styles.actions}>
            <button
              className={styles.resetButton}
              onClick={handleReset}
              aria-label="Reset to defaults">
              Reset to Defaults
            </button>
            <button
              className={styles.saveButton}
              onClick={handleSave}
              aria-label="Save preferences">
              {isSaved ? '✓ Saved!' : 'Save Preferences'}
            </button>
          </div>
        </div>

        <div className={styles.footer}>
          <small>
            ℹ️ No account required. Preferences are stored locally in your browser.
          </small>
        </div>
      </div>
    </div>
  );
};

export default PersonalizationPanel;
