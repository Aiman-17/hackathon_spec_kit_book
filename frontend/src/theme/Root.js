import React from 'react';
import ChatbotWidget from '../components/ChatbotWidget';
import MobileNavSidebar from '../components/MobileNavSidebar';

// Wraps the entire app to add global components like the chatbot and mobile nav
export default function Root({children}) {
  return (
    <>
      <MobileNavSidebar />
      {children}
      <ChatbotWidget />
    </>
  );
}
