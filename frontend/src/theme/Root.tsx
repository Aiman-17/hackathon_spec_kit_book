/**
 * Root Component Wrapper
 *
 * This component wraps the entire Docusaurus application
 * and injects the ChatWidget and MobileNavSidebar on all pages.
 */

import React from 'react';
import ChatbotWidget from '../components/ChatbotWidget';
import MobileNavSidebar from '../components/MobileNavSidebar';

export default function Root({ children }) {
  return (
    <>
      <MobileNavSidebar />
      {children}
      <ChatbotWidget />
    </>
  );
}
