import React from 'react';
import MDXContent from '@theme/MDXContent';

export default function DocItemContent({ children }) {
  return (
    <div className="container margin-vert--lg">
      <div className="doc-content">
        <MDXContent>
          {children}
        </MDXContent>
      </div>
    </div>
  );
}