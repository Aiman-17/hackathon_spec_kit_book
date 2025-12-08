/**
 * RAG Chatbot Widget Component
 *
 * Phase 5: Interactive chatbot widget for AI-Native Textbook
 * - Fixed position in bottom-right corner
 * - Connects to FastAPI backend for RAG queries
 * - Shows source citations from textbook
 */

import React, { useState, useRef, useEffect } from 'react';
import './styles.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: SourceCitation[];
  timestamp?: string;
}

interface SourceCitation {
  chapter: string;
  section: string;
  module: string;
  content_snippet: string;
  similarity_score: number;
}

// ============================================
// CONFIGURATION: Hugging Face Backend URL
// ============================================
const HF_BACKEND_URL = "https://huggingface.co/spaces/mksjai/ai-robotics-rag-backend";

// Backend API URL - automatically uses HF in production, localhost in development
const API_BASE_URL = process.env.REACT_APP_BACKEND_URL ||
  (process.env.NODE_ENV === 'production'
    ? HF_BACKEND_URL
    : 'http://localhost:8000');

// Check if running in production without backend configured
const IS_PRODUCTION_WITHOUT_BACKEND = process.env.NODE_ENV === 'production' &&
  HF_BACKEND_URL.includes('mksjai');

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      role: 'assistant',
      content: IS_PRODUCTION_WITHOUT_BACKEND
        ? 'Hi! The RAG chatbot feature is currently only available when running locally. To use this feature:\n\n1. Clone the repository\n2. Set up the backend (see README.md)\n3. Run locally with `npm start`\n\nFor now, you can explore the textbook content using the navigation menu!'
        : 'Hi! I\'m your AI tutor for Physical AI & Humanoid Robotics. Ask me anything about the textbook content!',
    }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

  const sendMessage = async () => {
    // Prevent sending in production without backend
    if (IS_PRODUCTION_WITHOUT_BACKEND) {
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'The chatbot is currently unavailable on the deployed site. Please run the project locally to use this feature. See the README for setup instructions.',
      }]);
      setInput('');
      return;
    }

    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      role: 'user',
      content: input,
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: input,
          max_results: 3,
          include_sources: true,
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to get response';
      setError(errorMessage);

      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please make sure the backend server is running and try again.',
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className="chat-widget-container">
      {/* Chat Button */}
      {!isOpen && (
        <button
          className="chat-toggle-button"
          onClick={toggleChat}
          aria-label="Open chat"
        >
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className="chat-window">
          {/* Header */}
          <div className="chat-header">
            <div className="chat-header-title">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
              </svg>
              <span>AI Tutor</span>
            </div>
            <button
              className="chat-close-button"
              onClick={toggleChat}
              aria-label="Close chat"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M18 6L6 18M6 6l12 12" />
              </svg>
            </button>
          </div>

          {/* Messages */}
          <div className="chat-messages">
            {messages.map((msg, idx) => (
              <div key={idx} className={`chat-message chat-message-${msg.role}`}>
                <div className="chat-message-content">
                  {msg.content}
                </div>

                {/* Source Citations */}
                {msg.sources && msg.sources.length > 0 && (
                  <div className="chat-sources">
                    <div className="chat-sources-title">Sources:</div>
                    {msg.sources.map((source, srcIdx) => (
                      <div key={srcIdx} className="chat-source-item">
                        <div className="chat-source-header">
                          {source.chapter} - {source.section}
                        </div>
                        <div className="chat-source-module">{source.module}</div>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className="chat-message chat-message-assistant">
                <div className="chat-loading">
                  <div className="chat-loading-dot"></div>
                  <div className="chat-loading-dot"></div>
                  <div className="chat-loading-dot"></div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Error Display */}
          {error && (
            <div className="chat-error">
              {error}
            </div>
          )}

          {/* Input */}
          <div className="chat-input-container">
            <input
              ref={inputRef}
              type="text"
              className="chat-input"
              placeholder={IS_PRODUCTION_WITHOUT_BACKEND
                ? "Chatbot unavailable - run locally to use"
                : "Ask a question about the textbook..."}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              disabled={isLoading}
            />
            <button
              className="chat-send-button"
              onClick={sendMessage}
              disabled={!input.trim() || isLoading}
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M22 2L11 13M22 2l-7 20-4-9-9-4 20-7z" />
              </svg>
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
