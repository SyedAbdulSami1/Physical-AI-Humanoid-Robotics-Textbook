import React, { useState, useEffect } from 'react';

const PersonalizeButton = ({ chapterSlug }) => {
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [backgroundLevel, setBackgroundLevel] = useState('');
  const [programmingLanguage, setProgrammingLanguage] = useState('');
  const [hardwareInterest, setHardwareInterest] = useState('');
  const [learningGoal, setLearningGoal] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [userBackground, setUserBackground] = useState('');

  // Check if user has already personalized
  useEffect(() => {
    const savedBackground = localStorage.getItem(`user-background-${chapterSlug}`);
    if (savedBackground) {
      setUserBackground(savedBackground);
      setIsPersonalized(true);
    }
  }, [chapterSlug]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);

    try {
      // In a real implementation, this would send data to the backend
      // Here we'll just store it locally as an example
      const questionnaireData = {
        background_level: backgroundLevel,
        programming_language: programmingLanguage,
        hardware_interest: hardwareInterest,
        learning_goal: learningGoal
      };

      // Store in local storage as an example
      localStorage.setItem(`user-background-${chapterSlug}`, JSON.stringify(questionnaireData));
      localStorage.setItem('user-questionnaire', JSON.stringify(questionnaireData));
      
      setUserBackground(JSON.stringify(questionnaireData));
      setIsPersonalized(true);
      setIsModalOpen(false);

      // In a real implementation, you would call an API endpoint here
      console.log('Questionnaire submitted:', questionnaireData);
    } catch (error) {
      console.error('Error submitting questionnaire:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handlePersonalize = () => {
    if (isPersonalized) {
      // Reset personalization
      localStorage.removeItem(`user-background-${chapterSlug}`);
      localStorage.removeItem('user-questionnaire');
      setIsPersonalized(false);
      setUserBackground('');
    } else {
      setIsModalOpen(true);
    }
  };

  return (
    <div className="personalize-section">
      <button 
        onClick={handlePersonalize}
        className={`personalize-button ${isPersonalized ? 'active' : ''}`}
      >
        {isPersonalized ? '✓ Personalized' : 'Personalize Content'}
      </button>

      {isModalOpen && (
        <div className="personalize-modal">
          <div className="modal-content">
            <h3>Tell us about your background</h3>
            <p>This helps us adapt the content to your level and interests.</p>
            
            <form onSubmit={handleSubmit}>
              <div className="form-group">
                <label htmlFor="backgroundLevel">Experience Level:</label>
                <select
                  id="backgroundLevel"
                  value={backgroundLevel}
                  onChange={(e) => setBackgroundLevel(e.target.value)}
                  required
                >
                  <option value="">Select your level</option>
                  <option value="novice">Novice (New to robotics/AI)</option>
                  <option value="intermediate">Intermediate (Some experience)</option>
                  <option value="advanced">Advanced (Experienced)</option>
                </select>
              </div>
              
              <div className="form-group">
                <label htmlFor="programmingLanguage">Preferred Programming Language:</label>
                <input
                  type="text"
                  id="programmingLanguage"
                  value={programmingLanguage}
                  onChange={(e) => setProgrammingLanguage(e.target.value)}
                  placeholder="e.g., Python, C++, etc."
                />
              </div>
              
              <div className="form-group">
                <label htmlFor="hardwareInterest">Hardware Interest:</label>
                <input
                  type="text"
                  id="hardwareInterest"
                  value={hardwareInterest}
                  onChange={(e) => setHardwareInterest(e.target.value)}
                  placeholder="e.g., humanoid robots, industrial robots, etc."
                />
              </div>
              
              <div className="form-group">
                <label htmlFor="learningGoal">Learning Goal:</label>
                <textarea
                  id="learningGoal"
                  value={learningGoal}
                  onChange={(e) => setLearningGoal(e.target.value)}
                  placeholder="What do you want to achieve?"
                  rows="3"
                />
              </div>
              
              <div className="form-actions">
                <button 
                  type="button" 
                  onClick={() => setIsModalOpen(false)}
                  className="cancel-button"
                >
                  Cancel
                </button>
                <button 
                  type="submit" 
                  disabled={isLoading}
                  className="submit-button"
                >
                  {isLoading ? 'Saving...' : 'Save Preferences'}
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
      
      {isPersonalized && (
        <div className="personalization-status">
          Content adapted to your preferences
        </div>
      )}
    </div>
  );
};

export default PersonalizeButton;