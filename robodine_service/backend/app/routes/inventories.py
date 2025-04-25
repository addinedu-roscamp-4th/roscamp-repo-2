from fastapi import APIRouter, Depends, HTTPException, Depends
from app.services.inventory_service import update_inventory  # 서비스 파일 호출
from pydantic import BaseModel
from typing import Optional
from app.models.inventory import Inventory
from app.models.inventory import MenuIngredient, MenuItem
from app.core.db_config import get_db, SessionLocal
from sqlalchemy.orm import Session
import datetime

router = APIRouter()

class MenuItemData(BaseModel):
    id: int
    name: Optional[str] = None
    price: Optional[float] = None
    prepare_time: Optional[int] = None

class InventoryData(BaseModel):
    ingredient_id: int
    count: Optional[int] = None
    status: Optional[str] = None

class MenuIgredientData(BaseModel):
    id: int
    name: Optional[str] = None
    menu_item_id: Optional[int] = None
    quantity_required: Optional[int] = None

# 재고 로그
@router.post("/update_inventory")
def update_inventory_endpoint(
    data: InventoryData,
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
        name          = menu.name,
        count         = data.count,
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
def update_robot_status(ingredient_data: MenuIgredientData):
    db: Session = SessionLocal()
    
    # Check if robot already exists in the database
    ingredient = db.query(MenuIngredient).filter(MenuIngredient.id == ingredient_data.id).first()
    
    if robot:
        # Update only the fields that are provided
        for key, value in ingredient_data.dict(exclude_unset=True).items():
            if value is not None:
                setattr(ingredient, key, value)
    else:
        # Create a new robot with the provided fields
        robot = ingredient(**ingredient_data.dict(exclude_unset=True))
    
    db.add(robot)
    db.commit()
    db.close()
    return {"message": "Ingredient data processed successfully!"}

# 재고 항목 업데이트
@router.post("/update_menu_item")
def update_menu_item(menu_item_data: MenuItemData):
    db: Session = SessionLocal()
    
    # Check if robot already exists in the database
    menu_item = db.query(MenuItem).filter(MenuItem.id == menu_item_data.id).first()
    
    if menu_item:
        # Update only the fields that are provided
        for key, value in menu_item_data.dict(exclude_unset=True).items():
            if value is not None:
                setattr(menu_item, key, value)
    else:
        # Create a new robot with the provided fields
        menu_item = MenuItem(**menu_item_data.dict(exclude_unset=True))
    
    db.add(menu_item)
    db.commit()
    db.close()
    return {"message": "Menu data processed successfully!"}