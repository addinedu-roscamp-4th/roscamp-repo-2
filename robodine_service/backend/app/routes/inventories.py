from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from pydantic import BaseModel
from typing import Optional, List
from app.models.inventory import Inventory
from app.models.inventory import MenuIngredient, MenuItem
from app.core.db_config import get_db
from sqlalchemy.orm import Session
from datetime import datetime
from app.models.enums import InventoryStatus, UserRole
from app.models.admin_settings import AdminSettings
from app.models import User
from app.routes.auth import get_current_user
from app.routes.events import log_info, log_warning, log_error

router = APIRouter()

class InventoryResponse(BaseModel):
    ingredient_id: int
    name: str
    count: int
    max_count: Optional[int] = None
    status: str

class InventoryCreateRequest(BaseModel):
    ingredient_id: int
    name: str
    count: int
    max_count: Optional[int] = None
    status: InventoryStatus

class MenuItemData(BaseModel):
    id: int
    name: Optional[str] = None
    price: Optional[float] = None
    prepare_time: Optional[int] = None

class MenuIgredientData(BaseModel):
    id: int
    name: Optional[str] = None
    menu_item_id: Optional[int] = None
    quantity_required: Optional[int] = None

# Get all inventory items
@router.get("", response_model=List[InventoryResponse])
def get_inventory(db: Session = Depends(get_db)):
    inventory_items = db.query(Inventory).all()
    return [
        InventoryResponse(
            ingredient_id=item.ingredient_id,
            name=item.name,
            count=item.count,
            max_count=item.max_count,
            status=item.status
        ) for item in inventory_items
    ]

# Create new inventory item
@router.post("", response_model=dict)
def create_inventory(
    inventory_data: dict,
    background_tasks: BackgroundTasks, 
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can create inventory
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to create inventory items"
        )
    
    # Check if the ingredient already exists
    existing_inventory = db.query(Inventory).filter(Inventory.name == inventory_data["name"]).first()
    if existing_inventory:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Inventory with name '{inventory_data['name']}' already exists"
        )
    
    # Create new inventory item
    max_count = inventory_data.get("max_count", 100)
    count = inventory_data.get("count", 0)
    
    # Determine status based on count and max_count
    status_value = InventoryStatus.IN_STOCK
    if count <= 0:
        status_value = InventoryStatus.OUT_OF_STOCK
    elif count < max_count * 0.2:  # Less than 20% of max
        status_value = InventoryStatus.LOW_STOCK
    
    new_inventory = Inventory(
        name=inventory_data["name"],
        count=count,
        max_count=max_count,
        status=status_value,
        updated_at=datetime.utcnow()
    )
    
    db.add(new_inventory)
    db.commit()
    db.refresh(new_inventory)
    
    # Log this action
    log_info(db, f"재고 항목 생성: {new_inventory.name}, 수량: {new_inventory.count}", background_tasks)
    
    return {
        "id": new_inventory.id,
        "status": "success",
        "message": "재고 항목이 생성되었습니다."
    }

# Update inventory item
@router.put("/{inventory_id}", response_model=dict)
def update_inventory(
    inventory_id: int,
    inventory_data: dict,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can update inventory
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to update inventory"
        )
    
    inventory = db.query(Inventory).filter(Inventory.id == inventory_id).first()
    if not inventory:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Inventory with ID {inventory_id} not found"
        )
    
    old_count = inventory.count
    
    # Update inventory fields
    if "name" in inventory_data:
        inventory.name = inventory_data["name"]
    
    if "count" in inventory_data:
        inventory.count = inventory_data["count"]
    
    if "max_count" in inventory_data:
        inventory.max_count = inventory_data["max_count"]
    
    # Recalculate status based on new values
    if inventory.count <= 0:
        inventory.status = InventoryStatus.OUT_OF_STOCK
        log_warning(db, f"재고 소진: {inventory.name} (ID: {inventory_id})", background_tasks)
    elif inventory.count < inventory.max_count * 0.2:  # Less than 20% of max
        inventory.status = InventoryStatus.LOW_STOCK
        log_warning(db, f"재고 부족: {inventory.name} (남은 수량: {inventory.count})", background_tasks)
    else:
        inventory.status = InventoryStatus.IN_STOCK
    
    inventory.updated_at = datetime.utcnow()
    
    db.add(inventory)
    db.commit()
    db.refresh(inventory)
    
    # Log inventory change
    if old_count != inventory.count:
        log_info(db, f"재고 수량 변경: {inventory.name}, {old_count} → {inventory.count}", background_tasks)
    
    return {
        "status": "success",
        "message": "재고가 업데이트되었습니다."
    }

# Delete inventory item
@router.delete("/{inventory_id}", response_model=dict)
def delete_inventory(inventory_id: int, db: Session = Depends(get_db)):
    # Find inventory item
    inventory = db.query(Inventory).filter(Inventory.id == inventory_id).first()
    if not inventory:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Inventory item with ID {inventory_id} not found"
        )
    
    # Get inventory name for log
    inventory_name = inventory.name
    
    # Delete inventory
    db.delete(inventory)
    db.commit()
    
    # Create a system log for this operation
    from app.models import SystemLog
    log = SystemLog(
        level="INFO",
        message=f"재고 항목 삭제: {inventory_name}",
        timestamp=datetime.utcnow()
    )
    db.add(log)
    db.commit()
    
    return {"message": "재고가 삭제되었습니다."}

# Below are legacy endpoints kept for backward compatibility
# 재고 로그
@router.post("/update_inventory")
def update_inventory_endpoint(
    data: InventoryCreateRequest,
    db: Session = Depends(get_db)
):
    # MenuIngredient 테이블에서 이름 조회
    menu = (
        db.query(MenuIngredient)
          .filter(MenuIngredient.id == data.ingredient_id)
          .first()
    )
    if not menu:
        raise HTTPException(status_code=404, detail="MenuIngredient not found")

    # Inventory 레코드 생성
    inv = Inventory(
        ingredient_id = data.ingredient_id,
        name          = data.name,
        count         = data.count,
        max_count     = data.max_count,
        status        = data.status,
    )
    db.add(inv)
    db.commit()
    db.refresh(inv)

    return {
        "message": "Inventory logged successfully",
        "inventory": inv,
    }

# 재료 항목 업데이트
@router.post("/update_menu_ingredient")
def update_menu_ingredient(ingredient_data: MenuIgredientData, db: Session = Depends(get_db)):
    # Check if ingredient already exists in the database
    ingredient = db.query(MenuIngredient).filter(MenuIngredient.id == ingredient_data.id).first()
    
    if ingredient:
        # Update only the fields that are provided
        for key, value in ingredient_data.dict(exclude_unset=True).items():
            if value is not None:
                setattr(ingredient, key, value)
    else:
        # Create a new ingredient with the provided fields
        ingredient = MenuIngredient(**ingredient_data.dict(exclude_unset=True))
    
    db.add(ingredient)
    db.commit()
    
    return {"message": "Ingredient data processed successfully!"}

# 메뉴 항목 업데이트
@router.post("/update_menu_item")
def update_menu_item(menu_item_data: MenuItemData, db: Session = Depends(get_db)):
    # Check if menu item already exists in the database
    menu_item = db.query(MenuItem).filter(MenuItem.id == menu_item_data.id).first()
    
    if menu_item:
        # Update only the fields that are provided
        for key, value in menu_item_data.dict(exclude_unset=True).items():
            if value is not None:
                setattr(menu_item, key, value)
    else:
        # Create a new menu item with the provided fields
        menu_item = MenuItem(**menu_item_data.dict(exclude_unset=True))
    
    db.add(menu_item)
    db.commit()
    
    return {"message": "Menu data processed successfully!"}