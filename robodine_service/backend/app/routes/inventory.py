from fastapi import APIRouter
from app.services.inventory_service import update_inventory  # 서비스 파일 호출
from pydantic import BaseModel
from typing import Optional
from app.models.inventory import Inventory
from app.core.db_config import SessionLocal

router = APIRouter()

class InventoryData(BaseModel):
    id: int  # Only ID is required
    item: Optional[str] = None
    count: Optional[int] = None
    status: Optional[str] = None

@router.post("/update_inventory")
def update_item_inventory(inventory_data: InventoryData):
    db = SessionLocal()
    
    # Check if inventory item already exists
    inventory = db.query(Inventory).filter(Inventory.id == inventory_data.id).first()
    
    if inventory:
        # Update only the fields that are provided
        for key, value in inventory_data.dict(exclude_unset=True).items():
            if value is not None and key != "id":
                setattr(inventory, key, value)
    else:
        # Create a new inventory item with the provided fields
        inventory = Inventory(**inventory_data.dict(exclude_unset=True))
    
    db.add(inventory)
    db.commit()
    db.close()
    
    return {"message": f"Inventory ID {inventory_data.id} updated successfully"}
