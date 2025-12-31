/**
 * Utility functions for content handling (personalization)
 */

/**
 * Fetches personalized content for a specific chapter and user profile
 */
export const fetchPersonalizedContent = async (chapterId, profile) => {
  try {
    const response = await fetch('/api/personalize', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        content: chapterId,
        profile: profile,
        userId: profile.userId
      }),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    return data.personalizedContent;
  } catch (error) {
    console.error('Error fetching personalized content:', error);
    throw error;
  }
};

/**
 * Gets the chapter content from its ID
 */
export const getChapterContent = async (chapterId) => {
  // In a real implementation, this would retrieve the actual chapter content
  // This is a placeholder implementation
  return `Content for chapter: ${chapterId}`;
};