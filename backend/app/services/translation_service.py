import asyncio
from typing import Optional
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def translate_text_to_urdu(text: str, target_language: str = "ur") -> str:
    """
    Translates text to Urdu using a local model or API.
    This function now uses the TranslationSkill for actual translation.
    """
    try:
        # Log the translation request
        logger.info(f"Translating text to {target_language}: {text[:50]}...")

        # Use the translation skill
        from app.skills.orchestrator import skill_orchestrator
        translated_text = await skill_orchestrator.execute_skill(
            'translation',
            text,
            target_language
        )

        return translated_text

    except Exception as e:
        logger.error(f"Translation error: {str(e)}")
        raise Exception(f"Translation service error: {str(e)}")

# Alternative implementation using transformers (uncomment when needed)
"""
from transformers import pipeline, AutoTokenizer, AutoModelForSeq2SeqLM

async def translate_text_to_urdu(text: str, target_language: str = "ur") -> str:
    try:
        # Load the NLLB model for translation
        # This would require downloading the model first
        model_name = "facebook/nllb-200-distilled-600M"
        tokenizer = AutoTokenizer.from_pretrained(model_name)
        model = AutoModelForSeq2SeqLM.from_pretrained(model_name)

        # Create the translation pipeline
        translator = pipeline(
            "translation",
            model=model,
            tokenizer=tokenizer,
            src_lang="eng_Latn",  # Source language: English
            tgt_lang="urd_Arab"   # Target language: Urdu
        )

        # Perform the translation
        result = translator(text)
        return result[0]['translation_text']

    except Exception as e:
        logger.error(f"Translation error: {str(e)}")
        raise Exception(f"Translation service error: {str(e)}")
"""