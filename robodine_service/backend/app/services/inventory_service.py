from app.models.inventory import Inventory
from app.services.model_service import process_model_update
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)

def update_inventory(inventory_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """
    Update inventory with partial data. Only ID is required.
    
    Args:
        inventory_data: Dictionary containing at least an ID and the fields to update
        
    Returns:
        The updated or created inventory instance data
    """
    try:
        result = process_model_update(Inventory, inventory_data)
        return result
    except Exception as e:
        logger.error(f"Error updating inventory: {str(e)}")
        return None
