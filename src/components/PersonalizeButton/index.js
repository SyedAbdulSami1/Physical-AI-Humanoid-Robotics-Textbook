import React, { useState, useEffect } from 'react';
import { useAuth } from '../../theme/Auth/Auth';

const PersonalizeButton = ({ chapterContent, onPersonalize }) => {
  const { user, isPending } = useAuth();
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [userProfile, setUserProfile] = useState(null);

  useEffect(() => {
    // Fetch user profile when user is available
    const fetchUserProfile = async () => {
      if (user && user.id) {
        try {
          const response = await fetch(`/api/user/profile/${user.id}`);
          if (response.ok) {
            const profileData = await response.json();
            setUserProfile(profileData);
          }
        } catch (error) {
          console.error('Error fetching user profile:', error);
        }
      }
    };

    if (!isPending && user) {
      fetchUserProfile();
    }
  }, [user, isPending]);

  const handlePersonalize = async () => {
    if (!user) {
      alert('Please log in to personalize content');
      return;
    }

    if (!userProfile) {
      alert('Please complete your profile first');
      return;
    }

    setIsPersonalizing(true);

    try {
      // Call backend API to personalize content based on user profile
      const response = await fetch('/api/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: chapterContent,
          profile: userProfile,
          userId: user.id
        }),
      });

      if (response.ok) {
        const { personalizedContent } = await response.json();
        onPersonalize(personalizedContent);
      } else {
        const errorData = await response.json();
        console.error('Failed to personalize content:', errorData);
        alert('Failed to personalize content: ' + (errorData.detail || 'Unknown error'));
      }
    } catch (error) {
      console.error('Error personalizing content:', error);
      alert('An error occurred while personalizing content');
    } finally {
      setIsPersonalizing(false);
    }
  };

  return (
    <button
      onClick={handlePersonalize}
      disabled={isPersonalizing || isPending || !user}
      className={`personalize-btn ${isPersonalizing ? 'loading' : ''} ${!user ? 'disabled' : ''}`}
    >
      {isPersonalizing ? 'Personalizing...' : 'Personalize Content'}
    </button>
  );
};

export default PersonalizeButton;