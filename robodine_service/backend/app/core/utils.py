from typing import Type, Dict, Any, Optional
from sqlmodel import SQLModel, Session
from datetime import datetime

def update_model(
    session: Session, 
    model_class: Type[SQLModel], 
    model_id: Any, 
    data: Dict[str, Any], 
    special_handlers: Optional[Dict[str, callable]] = None
) -> SQLModel:
    """
    Generic function to update a model with partial data.
    Only ID is required, all other fields are optional.
    
    Args:
        session: Database session
        model_class: The SQLModel class to update
        model_id: The ID of the model to update
        data: Dictionary containing fields to update
        special_handlers: Optional dict mapping field names to handler functions
        
    Returns:
        The updated or newly created model instance
    """
    special_handlers = special_handlers or {}
    
    # Try to get existing instance
    instance = session.get(model_class, model_id)
    
    # Create new instance if not found
    if not instance:
        # Create with ID only
        instance = model_class(id=model_id)
    
    # Update fields from data
    for key, value in data.items():
        if key != "id" and hasattr(instance, key):
            if key in special_handlers:
                # Use custom handler for special types
                setattr(instance, key, special_handlers[key](value))
            else:
                setattr(instance, key, value)
    
    # Add timestamp if model has the field
    if hasattr(instance, "timestamp") or hasattr(instance, "updated_at"):
        field_name = "timestamp" if hasattr(instance, "timestamp") else "updated_at"
        setattr(instance, field_name, datetime.now())
    
    session.add(instance)
    return instance 