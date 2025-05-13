from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List, Optional
from schemas import MenuItem, MenuIngredient, MenuItemCreate, MenuIngredientCreate, MenuItemUpdate, MenuIngredientUpdate
from database import get_db
from models import MenuItem as MenuItemModel, MenuIngredient as MenuIngredientModel, Inventory
import asyncio
from routers.events import broadcast_entity_update

router = APIRouter()

# 메뉴 항목 생성
@router.post("/items", response_model=MenuItem)
async def create_menu_item(item: MenuItemCreate, db: Session = Depends(get_db)):
    """새로운 메뉴 항목을 생성합니다."""
    db_item = MenuItemModel(**item.dict())
    db.add(db_item)
    db.commit()
    db.refresh(db_item)
    
    # 생성 후 WebSocket으로 메뉴 데이터 갱신
    asyncio.create_task(broadcast_entity_update("menu", None))
    
    return db_item

# 메뉴 항목 수정
@router.put("/items/{item_id}", response_model=MenuItem)
async def update_menu_item(item_id: int, item: MenuItemUpdate, db: Session = Depends(get_db)):
    """기존 메뉴 항목을 수정합니다."""
    db_item = db.query(MenuItemModel).filter(MenuItemModel.id == item_id).first()
    if not db_item:
        raise HTTPException(status_code=404, detail="메뉴 항목을 찾을 수 없습니다")
    
    # 항목 필드 업데이트
    for key, value in item.dict().items():
        setattr(db_item, key, value)
    
    db.commit()
    db.refresh(db_item)
    
    # 수정 후 WebSocket으로 메뉴 데이터 갱신
    asyncio.create_task(broadcast_entity_update("menu", None))
    
    return db_item

# 메뉴 항목 삭제
@router.delete("/items/{item_id}")
async def delete_menu_item(item_id: int, db: Session = Depends(get_db)):
    """메뉴 항목을 삭제합니다."""
    db_item = db.query(MenuItemModel).filter(MenuItemModel.id == item_id).first()
    if not db_item:
        raise HTTPException(status_code=404, detail="메뉴 항목을 찾을 수 없습니다")
    
    # 연결된 재료 먼저 삭제
    db.query(MenuIngredientModel).filter(MenuIngredientModel.menu_item_id == item_id).delete()
    
    # 메뉴 항목 삭제
    db.delete(db_item)
    db.commit()
    
    # 삭제 후 WebSocket으로 메뉴 데이터 갱신
    asyncio.create_task(broadcast_entity_update("menu", None))
    
    return {"message": "메뉴 항목이 삭제되었습니다"}

# 메뉴 재료 생성
@router.post("/ingredients", response_model=MenuIngredient)
async def create_menu_ingredient(ingredient: MenuIngredientCreate, db: Session = Depends(get_db)):
    """새로운 메뉴 재료를 생성합니다."""
    db_ingredient = MenuIngredientModel(**ingredient.dict())
    db.add(db_ingredient)
    db.commit()
    db.refresh(db_ingredient)
    
    # 재고 항목도 생성 (없는 경우)
    existing_inventory = db.query(Inventory).filter(Inventory.name == db_ingredient.name).first()
    if not existing_inventory:
        new_inventory = Inventory(
            name=db_ingredient.name,
            count=0,
            max_count=100,  # 기본값
            status="OUT_OF_STOCK",
            ingredient_id=db_ingredient.id
        )
        db.add(new_inventory)
        db.commit()
    
    # 생성 후 WebSocket으로 메뉴 데이터 갱신
    asyncio.create_task(broadcast_entity_update("menu", None))
    # 재고 데이터도 갱신
    asyncio.create_task(broadcast_entity_update("inventory", None))
    
    return db_ingredient

# 메뉴 재료 수정
@router.put("/ingredients/{ingredient_id}", response_model=MenuIngredient)
async def update_menu_ingredient(ingredient_id: int, ingredient: MenuIngredientUpdate, db: Session = Depends(get_db)):
    """기존 메뉴 재료를 수정합니다."""
    db_ingredient = db.query(MenuIngredientModel).filter(MenuIngredientModel.id == ingredient_id).first()
    if not db_ingredient:
        raise HTTPException(status_code=404, detail="메뉴 재료를 찾을 수 없습니다")
    
    old_name = db_ingredient.name
    
    # 재료 필드 업데이트
    for key, value in ingredient.dict().items():
        setattr(db_ingredient, key, value)
    
    db.commit()
    db.refresh(db_ingredient)
    
    # 이름이 변경된 경우 연결된 재고 항목 이름도 업데이트
    if old_name != db_ingredient.name:
        inventory_item = db.query(Inventory).filter(Inventory.name == old_name).first()
        if inventory_item:
            inventory_item.name = db_ingredient.name
            inventory_item.ingredient_id = db_ingredient.id
            db.commit()
    
    # 수정 후 WebSocket으로 메뉴 데이터 갱신
    asyncio.create_task(broadcast_entity_update("menu", None))
    # 재고 데이터도 갱신
    asyncio.create_task(broadcast_entity_update("inventory", None))
    
    return db_ingredient

# 메뉴 재료 삭제
@router.delete("/ingredients/{ingredient_id}")
async def delete_menu_ingredient(ingredient_id: int, db: Session = Depends(get_db)):
    """메뉴 재료를 삭제합니다."""
    db_ingredient = db.query(MenuIngredientModel).filter(MenuIngredientModel.id == ingredient_id).first()
    if not db_ingredient:
        raise HTTPException(status_code=404, detail="메뉴 재료를 찾을 수 없습니다")
    
    # 재료 이름 저장 (재고 항목 찾기 위해)
    ingredient_name = db_ingredient.name
    
    # 메뉴 재료 삭제
    db.delete(db_ingredient)
    db.commit()
    
    # 삭제 후 WebSocket으로 메뉴 데이터 갱신
    asyncio.create_task(broadcast_entity_update("menu", None))
    
    # 연결된 재고 항목 삭제는 하지 않음 (재고는 그대로 유지)
    # 대신 연결 정보만 제거
    inventory_item = db.query(Inventory).filter(Inventory.name == ingredient_name).first()
    if inventory_item:
        inventory_item.ingredient_id = None
        db.commit()
        # 재고 데이터도 갱신
        asyncio.create_task(broadcast_entity_update("inventory", None))
    
    return {"message": "메뉴 재료가 삭제되었습니다"} 