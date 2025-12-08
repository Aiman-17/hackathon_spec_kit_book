import React from 'react';
import ChatbotWidget from '../components/ChatbotWidget';

// Wraps the entire app to add global components like the chatbot
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}
