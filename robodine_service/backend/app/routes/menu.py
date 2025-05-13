from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel

from app.core.db_config import get_db
from app.models import MenuItem, MenuIngredient, User
from app.models.enums import UserRole
from app.routes.auth import get_current_user

router = APIRouter()

# --- Menu Models ---
class MenuItemResponse(BaseModel):
    id: int
    name: str
    price: float
    prepare_time: int

class MenuItemDetailResponse(BaseModel):
    id: int
    name: str
    price: float
    prepare_time: int
    menu_ingredients: List[dict]

class MenuItemCreateRequest(BaseModel):
    name: str
    price: float
    prepare_time: int

class MenuItemUpdateRequest(BaseModel):
    name: Optional[str] = None
    price: Optional[float] = None
    prepare_time: Optional[int] = None

class IngredientResponse(BaseModel):
    id: int
    name: str
    menu_item_id: int
    quantity_required: int

class IngredientCreateRequest(BaseModel):
    name: str
    menu_item_id: int
    quantity_required: int

class IngredientUpdateRequest(BaseModel):
    name: Optional[str] = None
    quantity_required: Optional[int] = None

# --- Router Endpoints for Menu Items ---
@router.get("/items", response_model=List[MenuItemResponse])
def get_menu_items(db: Session = Depends(get_db)):
    menu_items = db.query(MenuItem).all()
    return [
        MenuItemResponse(
            id=item.id,
            name=item.name,
            price=item.price,
            prepare_time=item.prepare_time
        ) for item in menu_items
    ]

@router.get("/items/{item_id}", response_model=MenuItemDetailResponse)
def get_menu_item(item_id: int, db: Session = Depends(get_db)):
    menu_item = db.query(MenuItem).filter(MenuItem.id == item_id).first()
    if not menu_item:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Menu item with ID {item_id} not found"
        )
    
    # Get ingredients
    ingredients = db.query(MenuIngredient).filter(MenuIngredient.menu_item_id == item_id).all()
    
    return MenuItemDetailResponse(
        id=menu_item.id,
        name=menu_item.name,
        price=menu_item.price,
        prepare_time=menu_item.prepare_time,
        menu_ingredients=[
            {
                "ingredient_id": ingredient.id,
                "quantity_required": ingredient.quantity_required
            } for ingredient in ingredients
        ]
    )

@router.post("/items", response_model=dict)
def create_menu_item(
    item_data: MenuItemCreateRequest, 
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can create menu items
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to create menu items"
        )
    
    # Create new menu item
    new_item = MenuItem(
        name=item_data.name,
        price=item_data.price,
        prepare_time=item_data.prepare_time
    )
    
    db.add(new_item)
    db.commit()
    db.refresh(new_item)
    
    return {
        "id": new_item.id,
        "status": "success",
        "message": "메뉴 항목이 생성되었습니다."
    }

@router.put("/items/{item_id}", response_model=dict)
def update_menu_item(
    item_id: int,
    item_data: MenuItemUpdateRequest,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can update menu items
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to update menu items"
        )
    
    # Find menu item
    menu_item = db.query(MenuItem).filter(MenuItem.id == item_id).first()
    if not menu_item:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Menu item with ID {item_id} not found"
        )
    
    # Update fields
    if item_data.name:
        menu_item.name = item_data.name
    
    if item_data.price is not None:
        menu_item.price = item_data.price
    
    if item_data.prepare_time is not None:
        menu_item.prepare_time = item_data.prepare_time
    
    db.add(menu_item)
    db.commit()
    
    return {
        "status": "success",
        "message": "메뉴 항목이 수정되었습니다."
    }

@router.delete("/items/{item_id}", response_model=dict)
def delete_menu_item(
    item_id: int,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can delete menu items
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to delete menu items"
        )
    
    # Find menu item
    menu_item = db.query(MenuItem).filter(MenuItem.id == item_id).first()
    if not menu_item:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Menu item with ID {item_id} not found"
        )
    
    # Delete all associated ingredients first
    db.query(MenuIngredient).filter(MenuIngredient.menu_item_id == item_id).delete()
    
    # Delete menu item
    db.delete(menu_item)
    db.commit()
    
    return {
        "message": "메뉴 항목이 삭제되었습니다."
    }

# --- Router Endpoints for Ingredients ---
@router.get("/ingredients", response_model=List[IngredientResponse])
def get_ingredients(db: Session = Depends(get_db)):
    ingredients = db.query(MenuIngredient).all()
    return [
        IngredientResponse(
            id=ingredient.id,
            name=ingredient.name,
            menu_item_id=ingredient.menu_item_id,
            quantity_required=ingredient.quantity_required
        ) for ingredient in ingredients
    ]

@router.get("/ingredients/{ingredient_id}", response_model=IngredientResponse)
def get_ingredient(ingredient_id: int, db: Session = Depends(get_db)):
    ingredient = db.query(MenuIngredient).filter(MenuIngredient.id == ingredient_id).first()
    if not ingredient:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Ingredient with ID {ingredient_id} not found"
        )
    
    return IngredientResponse(
        id=ingredient.id,
        name=ingredient.name,
        menu_item_id=ingredient.menu_item_id,
        quantity_required=ingredient.quantity_required
    )

@router.post("/ingredients", response_model=dict)
def create_ingredient(
    ingredient_data: IngredientCreateRequest,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can create ingredients
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to create ingredients"
        )
    
    # Check if menu item exists
    menu_item = db.query(MenuItem).filter(MenuItem.id == ingredient_data.menu_item_id).first()
    if not menu_item:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Menu item with ID {ingredient_data.menu_item_id} not found"
        )
    
    # Create new ingredient
    new_ingredient = MenuIngredient(
        name=ingredient_data.name,
        menu_item_id=ingredient_data.menu_item_id,
        quantity_required=ingredient_data.quantity_required
    )
    
    db.add(new_ingredient)
    db.commit()
    db.refresh(new_ingredient)
    
    return {
        "id": new_ingredient.id,
        "status": "success",
        "message": "재료가 생성되었습니다."
    }

@router.put("/ingredients/{ingredient_id}", response_model=dict)
def update_ingredient(
    ingredient_id: int,
    ingredient_data: IngredientUpdateRequest,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can update ingredients
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to update ingredients"
        )
    
    # Find ingredient
    ingredient = db.query(MenuIngredient).filter(MenuIngredient.id == ingredient_id).first()
    if not ingredient:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Ingredient with ID {ingredient_id} not found"
        )
    
    # Update fields
    if ingredient_data.name:
        ingredient.name = ingredient_data.name
    
    if ingredient_data.quantity_required is not None:
        ingredient.quantity_required = ingredient_data.quantity_required
    
    db.add(ingredient)
    db.commit()
    
    return {
        "status": "success",
        "message": "재료가 수정되었습니다."
    }

@router.delete("/ingredients/{ingredient_id}", response_model=dict)
def delete_ingredient(
    ingredient_id: int,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    # Only admins can delete ingredients
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to delete ingredients"
        )
    
    # Find ingredient
    ingredient = db.query(MenuIngredient).filter(MenuIngredient.id == ingredient_id).first()
    if not ingredient:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Ingredient with ID {ingredient_id} not found"
        )
    
    # Delete ingredient
    db.delete(ingredient)
    db.commit()
    
    return {
        "message": "재료가 삭제되었습니다."
    } 