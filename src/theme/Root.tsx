// src/theme/Root.tsx

import React from 'react';
import Root from '@theme-original/Root';
import { SelectedTextProvider } from './Chatbot/SelectedTextProvider';
import Chatbot from './Chatbot/Chatbot';

export default function RootWrapper({children}) {
  return (
    <SelectedTextProvider>
      {/* The original Docusaurus root component that renders the page */}
      <Root>{children}</Root>

      {/* Render the chatbot on all pages */}
      <Chatbot />
    </SelectedTextProvider>
  );
}
