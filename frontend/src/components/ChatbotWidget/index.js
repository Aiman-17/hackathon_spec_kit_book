import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

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
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Backend API URL - defaults to Hugging Face Space in production
  const API_URL = process.env.REACT_APP_BACKEND_URL ||
                  (typeof window !== 'undefined' && window.location.hostname.includes('github.io')
                    ? 'https://your-username-ai-robotics-rag.hf.space'  // Update this after creating HF Space
                    : 'http://localhost:8000');

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
          include_sources: true
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
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <div className={styles.botIcon}>🤖</div>
              <div>
                <div className={styles.botName}>AI Robotics Tutor</div>
                <div className={styles.botStatus}>Online</div>
              </div>
            </div>
            <button
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat">
              ✕
            </button>
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
              placeholder="Ask about ROS 2, SLAM, robotics concepts..."
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
