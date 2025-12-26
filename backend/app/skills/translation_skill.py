from typing import Optional
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class TranslationSkill:
    """
    A reusable skill for translating content to Urdu.
    """
    
    def __init__(self):
        self.name = "TranslationSkill"
        self.description = "A skill for translating content to Urdu"
    
    async def execute(self, text: str, target_language: str = "ur") -> str:
        """
        Execute the translation logic.
        
        Args:
            text (str): The text to translate
            target_language (str): The target language code (default: ur for Urdu)
            
        Returns:
            str: Translated text
        """
        try:
            logger.info(f"Executing {self.name} for text: {text[:50]}...")
            
            # Perform translation
            translated_text = await self._translate_text(text, target_language)
            
            logger.info(f"{self.name} executed successfully")
            return translated_text
            
        except Exception as e:
            logger.error(f"Error in {self.name}: {str(e)}")
            raise
    
    import httpx

# ... (keep existing logging setup)

class TranslationSkill:
    # ... (keep existing __init__ and execute methods)

    async def _translate_text(self, text: str, target_language: str) -> str:
        """
        Translates text to the specified language using LibreTranslate.
        """
        if target_language.lower() != "ur":
            return text

        libretranslate_url = "https://libretranslate.de/translate"
        
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    libretranslate_url,
                    json={
                        "q": text,
                        "source": "en",
                        "target": "ur",
                        "format": "text"
                    },
                    timeout=30.0  # 30-second timeout
                )
                response.raise_for_status()  # Raise an exception for bad status codes
                
                result = response.json()
                
                if "translatedText" in result:
                    return result["translatedText"]
                else:
                    logger.error(f"LibreTranslate response did not contain 'translatedText': {result}")
                    raise Exception("Translation failed: Invalid response from service.")

        except httpx.RequestError as e:
            logger.error(f"Error calling LibreTranslate API: {e}")
            raise Exception(f"Translation service is unavailable: {e}")
        except Exception as e:
            logger.error(f"An unexpected error occurred during translation: {e}")
            raise


# Create a singleton instance
translation_skill = TranslationSkill()