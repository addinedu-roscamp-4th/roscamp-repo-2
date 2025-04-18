from typing import Type, Dict, Any, TypeVar, Optional
from sqlmodel import SQLModel, Session
from app.core.db_config import SessionLocal
from app.core.utils import update_model
import json
import logging

logger = logging.getLogger(__name__)

T = TypeVar('T', bound=SQLModel)

def process_model_update(
    model_class: Type[T], 
    data: Dict[str, Any], 
    special_handlers: Optional[Dict[str, callable]] = None
) -> T:
    """
    Generic service function to process partial updates for any model.
    Only ID is required, all other fields are optional.
    
    Args:
        model_class: SQLModel class to update
        data: Dictionary containing at least an ID and the fields to update
        special_handlers: Optional dict mapping field names to handler functions
        
    Returns:
        The updated or created model instance
    """
    # ID is required
    if "id" not in data:
        raise ValueError("ID field is required for model updates")
    
    model_id = data["id"]
    
    try:
        with SessionLocal() as session:
            instance = update_model(session, model_class, model_id, data, special_handlers)
            session.commit()
            # Create a copy of the model to return (after the session is closed)
            # to avoid detached instance issues
            result = {key: getattr(instance, key) for key in instance.__dict__ if not key.startswith('_')}
            logger.info(f"Successfully updated {model_class.__name__} with ID {model_id}")
            return result
    except Exception as e:
        logger.error(f"Failed to update {model_class.__name__} with ID {model_id}: {str(e)}")
        raise 