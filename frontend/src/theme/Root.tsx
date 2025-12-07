/**
 * Root Component Wrapper
 *
 * This component wraps the entire Docusaurus application
 * and injects the ChatWidget on all pages.
 */

import React from 'react';
import ChatWidget from '../components/ChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
