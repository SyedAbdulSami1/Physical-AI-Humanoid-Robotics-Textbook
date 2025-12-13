from typing import Optional
import logging
import asyncio
import aiohttp
import os
from pydantic import BaseModel

logger = logging.getLogger(__name__)

class TranslationRequest(BaseModel):
    text: str
    source_lang: str = "en"
    target_lang: str = "ur"  # Urdu

class TranslationService:
    """
    Service for translating content to Urdu using various APIs
    """
    
    def __init__(self):
        self.google_api_key = os.getenv("GOOGLE_TRANSLATE_API_KEY")
        self.deepl_api_key = os.getenv("DEEPL_API_KEY")
    
    async def translate_to_urdu(self, text: str, method: str = "fallback") -> Optional[str]:
        """
        Translate English text to Urdu using available services
        """
        if not text.strip():
            return text
        
        # Try different translation methods based on availability
        if method == "google" and self.google_api_key:
            return await self._translate_with_google(text)
        elif method == "deepl" and self.deepl_api_key:
            return await self._translate_with_deepl(text)
        else:
            # Try Google first, then DeepL, then a fallback approach
            if self.google_api_key:
                result = await self._translate_with_google(text)
                if result:
                    return result
            
            if self.deepl_api_key:
                result = await self._translate_with_deepl(text)
                if result:
                    return result
            
            # Fallback: return original text if no translation service is available
            return self._translate_with_fallback(text)
    
    async def _translate_with_google(self, text: str) -> Optional[str]:
        """
        Translate using Google Cloud Translation API
        """
        try:
            import google.cloud.translate_v2 as translate

            client = translate.Client(credentials=self.google_api_key)
            
            # Handle long texts by splitting into chunks
            if len(text) > 5000:
                chunks = self._split_text(text, 5000)
                translated_chunks = []
                
                for chunk in chunks:
                    result = client.translate(
                        chunk,
                        target_language='ur',
                        source_language='en'
                    )
                    translated_chunks.append(result['translatedText'])
                
                return ' '.join(translated_chunks)
            else:
                result = client.translate(
                    text,
                    target_language='ur',
                    source_language='en'
                )
                return result['translatedText']
                
        except Exception as e:
            logger.error(f"Google translation error: {str(e)}")
            return None
    
    async def _translate_with_deepl(self, text: str) -> Optional[str]:
        """
        Translate using DeepL API
        """
        try:
            url = "https://api-free.deepl.com/v2/translate"
            
            headers = {
                "Authorization": f"DeepL-Auth-Key {self.deepl_api_key}",
                "Content-Type": "application/x-www-form-urlencoded"
            }
            
            data = {
                "text": text,
                "target_lang": "UR"
            }
            
            async with aiohttp.ClientSession() as session:
                async with session.post(url, headers=headers, data=data) as response:
                    if response.status == 200:
                        json_response = await response.json()
                        translated_text = json_response['translations'][0]['text']
                        return translated_text
                    else:
                        logger.error(f"DeepL API error: {response.status}")
                        return None
        except Exception as e:
            logger.error(f"DeepL translation error: {str(e)}")
            return None
    
    def _translate_with_fallback(self, text: str) -> Optional[str]:
        """
        Fallback translation using a dictionary of common terms
        This is a very basic approach and should be replaced with a proper API in production
        """
        # For a real implementation, you would use a proper translation API
        # This is just a placeholder to show the structure
        logger.warning("Using fallback translation - please configure a proper translation API")
        return f"[URDU TRANSLATION NEEDED: {text[:50]}...]"  # Placeholder
    
    def _split_text(self, text: str, max_length: int) -> list:
        """
        Split text into chunks of max_length without breaking words
        """
        chunks = []
        current_chunk = ""
        
        for word in text.split():
            if len(current_chunk + " " + word) <= max_length:
                current_chunk += " " + word
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = word
        
        if current_chunk:
            chunks.append(current_chunk.strip())
        
        return chunks

class UrduTranslationService:
    """
    Main service for Urdu translation with loading states and fallbacks
    """
    
    def __init__(self):
        self.translation_service = TranslationService()
        self.cache = {}  # Simple cache to avoid repeated translations
    
    async def translate_chapter_content(self, content: str, use_cache: bool = True) -> str:
        """
        Translate a full chapter to Urdu
        """
        cache_key = f"urdu_{hash(content)}"
        
        if use_cache and cache_key in self.cache:
            return self.cache[cache_key]
        
        try:
            # For chapter translation, we might want to process it in chunks
            # to handle large texts and maintain quality
            translated = await self.translation_service.translate_to_urdu(content)
            
            if use_cache:
                self.cache[cache_key] = translated
            
            return translated or content  # Return original if translation failed
        except Exception as e:
            logger.error(f"Chapter translation error: {str(e)}")
            return f"Translation failed: {str(e)}"
    
    async def translate_text_snippet(self, text: str) -> str:
        """
        Translate a small text snippet to Urdu
        """
        try:
            return await self.translation_service.translate_to_urdu(text) or text
        except Exception as e:
            logger.error(f"Snippet translation error: {str(e)}")
            return f"Translation failed: {str(e)}"

# Create a singleton instance
urdu_translation_service = UrduTranslationService()