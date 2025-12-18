import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import PersonalizationPanel from '../PersonalizationPanel';

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: 'Hi! I\'m your AI tutor for Physical AI & Humanoid Robotics. Ask me anything about the textbook content!',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [userLevel, setUserLevel] = useState('student');
  const [language, setLanguage] = useState('en');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Backend API URL - environment-safe detection (no process references)
  const getBackendUrl = () => {
    // Check if running in browser
    if (typeof window === 'undefined') {
      return 'http://localhost:8000';
    }

    // Production: GitHub Pages -> Hugging Face Space
    if (window.location.hostname.includes('github.io')) {
      return 'https://mksjai-ai-robotics-rag-backend.hf.space';
    }

    // Development: localhost
    return 'http://localhost:8000';
  };

  const API_URL = getBackendUrl();

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Load user preferences from localStorage on mount
  useEffect(() => {
    try {
      const savedLevel = localStorage.getItem('userLevel') || 'student';
      const savedLanguage = localStorage.getItem('language') || 'en';
      setUserLevel(savedLevel);
      setLanguage(savedLanguage);
    } catch (error) {
      console.error('Failed to load preferences:', error);
    }

    // Listen for preference changes from PersonalizationPanel
    const handlePreferenceChange = (event) => {
      setUserLevel(event.detail.userLevel);
      setLanguage(event.detail.language);
    };

    window.addEventListener('preferencesChanged', handlePreferenceChange);
    return () => window.removeEventListener('preferencesChanged', handlePreferenceChange);
  }, []);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      role: 'user',
      content: inputValue,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch(`${API_URL}/api/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          max_results: 3,
          include_sources: true,
          user_level: userLevel,  // Personalization
          language: language      // Translation
        })
      });

      if (!response.ok) {
        throw new Error('Failed to get response from chatbot');
      }

      const data = await response.json();

      const assistantMessage = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources || [],
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      // Log error for debugging but don't expose details to user
      console.error('Chatbot error:', error);

      const errorMessage = {
        role: 'assistant',
        content: 'Sorry, I\'m having trouble connecting to the knowledge base right now. Please try again in a moment. If the issue persists, the backend service may not be available.',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className={styles.chatbotContainer}>
      {/* Personalization Settings Panel */}
      <PersonalizationPanel
        isOpen={isSettingsOpen}
        onClose={() => setIsSettingsOpen(false)}
      />

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <div className={styles.botIcon}>🤖</div>
              <div>
                <div className={styles.botName}>AI Robotics Tutor</div>
                <div className={styles.botStatus}>
                  {language === 'ur' ? '🇵🇰 Urdu' : '🇺🇸 English'} • {userLevel}
                </div>
              </div>
            </div>
            <div className={styles.headerButtons}>
              <button
                className={styles.settingsButton}
                onClick={() => setIsSettingsOpen(true)}
                aria-label="Open settings"
                title="Personalization Settings">
                ⚙️
              </button>
              <button
                className={styles.closeButton}
                onClick={toggleChat}
                aria-label="Close chat">
                ✕
              </button>
            </div>
          </div>

          <div className={styles.messagesContainer}>
            {messages.map((message, index) => (
              <div
                key={index}
                className={`${styles.message} ${
                  message.role === 'user' ? styles.userMessage : styles.assistantMessage
                }`}>
                <div className={styles.messageContent}>
                  {message.content}
                </div>
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <div className={styles.sourcesTitle}>📚 Sources:</div>
                    {message.sources.map((source, idx) => (
                      <div key={idx} className={styles.sourceItem}>
                        <strong>{source.chapter}</strong> / {source.section}
                        <div className={styles.similarityScore}>
                          Relevance: {(source.similarity_score * 100).toFixed(0)}%
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.loadingDots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputContainer}>
            <textarea
              ref={inputRef}
              className={styles.input}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your question here..."
              rows={1}
              disabled={isLoading}
            />
            <button
              className={styles.sendButton}
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isLoading}
              aria-label="Send message">
              {isLoading ? '⏳' : '➤'}
            </button>
          </div>
        </div>
      )}

      <button
        className={`${styles.chatToggle} ${isOpen ? styles.chatToggleOpen : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}>
        {isOpen ? '✕' : '💬'}
      </button>
    </div>
  );
};

export default ChatbotWidget;
