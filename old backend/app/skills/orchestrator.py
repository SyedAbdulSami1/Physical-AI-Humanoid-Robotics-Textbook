from typing import Dict, Any, List
import logging
from .personalization_skill import personalization_skill

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SkillOrchestrator:
    """
    Orchestrates the execution of various skills like personalization.
    """
    
    def __init__(self):
        self.skills = {
            'personalization': personalization_skill,
        }
    
    async def execute_skill(self, skill_name: str, *args, **kwargs) -> Any:
        """
        Execute a specific skill by name.
        
        Args:
            skill_name (str): Name of the skill to execute
            *args: Arguments to pass to the skill
            **kwargs: Keyword arguments to pass to the skill
            
        Returns:
            Any: Result of the skill execution
        """
        if skill_name not in self.skills:
            raise ValueError(f"Skill '{skill_name}' not found")
        
        skill = self.skills[skill_name]
        logger.info(f"Executing skill: {skill.name}")
        
        try:
            result = await skill.execute(*args, **kwargs)
            logger.info(f"Skill '{skill.name}' executed successfully")
            return result
        except Exception as e:
            logger.error(f"Error executing skill '{skill.name}': {str(e)}")
            raise
    
    async def execute_multiple_skills(self, skills_list: List[Dict[str, Any]]) -> List[Any]:
        """
        Execute multiple skills in sequence.
        
        Args:
            skills_list (List[Dict]): List of skill configurations with name and args
            
        Returns:
            List[Any]: Results of all skill executions
        """
        results = []
        
        for skill_config in skills_list:
            skill_name = skill_config.get('name')
            skill_args = skill_config.get('args', [])
            skill_kwargs = skill_config.get('kwargs', {})
            
            result = await self.execute_skill(skill_name, *skill_args, **skill_kwargs)
            results.append(result)
        
        return results

# Create a singleton instance
skill_orchestrator = SkillOrchestrator()