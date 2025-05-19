from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks, Form, UploadFile, File
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
from pydantic import BaseModel
import uuid

from app.core.db_config import get_db
from app.models import MenuItem, MenuIngredient, User, Inventory
from app.models.enums import UserRole, InventoryStatus
from app.routes.auth import get_current_user
from app.routes.websockets import manager
import asyncio

router = APIRouter()


# 로거 설정 및 저장
import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.FileHandler('inventory.log')
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)
# Create a directory for logs if it doesn't exist
import os
LOG_DIR = os.path.join(os.path.dirname(__file__), '..','..', '..', 'logs')
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, 'inventory.log')
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, 'w') as f:
        f.write("Inventory log file created.\n")
    f.write("Log entries will be appended here.\n")
    f.close()

# 메뉴 데이터 브로드캐스팅을 위한 유틸리티 함수
async def broadcast_menu_data(db: Session):
    """메뉴 데이터 변경을 WebSocket으로 브로드캐스팅합니다"""
    try:
        # 메뉴 및 재료 데이터 조회
        menu_items = db.query(MenuItem).all()
        menu_ingredients = db.query(MenuIngredient).all()
        
        # 디버깅 로그
        print(f"메뉴 브로드캐스팅: {len(menu_items)} 항목, {len(menu_ingredients)} 재료")
        
        # 직렬화
        serialized_items = [
            {
                "MenuItem.id": item.id, 
                "MenuItem.name": item.name, 
                "MenuItem.price": item.price, 
                "MenuItem.prepare_time": item.prepare_time,
                "MenuItem.image_url": item.image_url,
                "MenuItem.description": item.description,
            } 
            for item in menu_items
        ]
        
        serialized_ingredients = [
            {
                "MenuIngredient.id": ing.id, 
                "MenuIngredient.name": ing.name, 
                "MenuIngredient.menu_item_id": ing.menu_item_id, 
                "MenuIngredient.quantity_required": ing.quantity_required
            } 
            for ing in menu_ingredients
        ]
        
        # 브로드캐스트 메시지 구성
        message = {
            "type": "update",
            "topic": "menu", 
            "data": {
                "items": serialized_items,
                "ingredients": serialized_ingredients
            }
        }
        
        # 웹소켓을 통해 브로드캐스트
        await manager.broadcast(message, "menu")
        print(f"메뉴 데이터 브로드캐스트 완료")
    except Exception as e:
        print(f"메뉴 데이터 브로드캐스트 오류: {e}")


# --- Menu Models ---
class MenuItemResponse(BaseModel):
    id: int
    name: str
    price: float
    prepare_time: int
    image_url: Optional[str] = None
    description: Optional[str] = None

class MenuItemDetailResponse(BaseModel):
    id: int
    name: str
    price: float
    prepare_time: int
    menu_ingredients: List[dict]
    image_url: Optional[str] = None
    description: Optional[str] = None

class MenuItemCreateRequest(BaseModel):
    name: str
    price: float
    prepare_time: int
    image_url: Optional[str] = None
    description: Optional[str] = None

class MenuItemUpdateRequest(BaseModel):
    name: Optional[str] = None
    price: Optional[float] = None
    prepare_time: Optional[int] = None
    image_url: Optional[str] = None
    description: Optional[str] = None

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
            prepare_time=item.prepare_time,
            image_url=item.image_url,
            description=item.description
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
        image_url=menu_item.image_url,
        description=menu_item.description,
        menu_ingredients=[
            {
                "ingredient_id": ingredient.id,
                "quantity_required": ingredient.quantity_required
            } for ingredient in ingredients
        ]
    )

# 업로드된 파일을 저장할 디렉토리
UPLOAD_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'images', 'menu')
os.makedirs(UPLOAD_DIR, exist_ok=True)

@router.post("/items", response_model=dict)
async def create_menu_item(
    # form-data 로 받을 텍스트 필드들
    name: str = Form(...),
    price: float = Form(...),
    prepare_time: int = Form(...),
    description: str = Form(None),
    # 이미지 파일 (없어도 됨)
    image: UploadFile = File(None),
    image_url: str = Form(None),

    db: Session = Depends(get_db),
    current_user=Depends(get_current_user),
):
    # 권한 체크
    if current_user.role != UserRole.ADMIN:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="권한이 없습니다")

    image_url = None
    if image:
        # 1) 고유 파일명 생성
        ext = os.path.splitext(image.filename)[1]
        filename = f"{uuid.uuid4().hex}{ext}"
        filepath = os.path.join(UPLOAD_DIR, filename)

        # 2) 디스크에 저장
        with open(filepath, "wb") as f:
            content = await image.read()
            f.write(content)

        # 3) 정적 경로로 쓸 URL 생성 (백엔드가 /static 을 serve 한다고 가정)
        image_url = f"/images/menu/{filename}"

    # DB에 저장
    new_item = MenuItem(
        name=name,
        price=price,
        prepare_time=prepare_time,
        description=description,
        image_url=image_url,
    )
    db.add(new_item)
    db.commit()
    db.refresh(new_item)

    return {
        "status": "success",
        "message": "메뉴 항목이 추가되었습니다."
    }

@router.put("/items/{item_id}", response_model=dict)
async def update_menu_item(
    # form-data 로 받을 텍스트 필드들
    item_id: int,
    name: str = Form(...),
    price: float = Form(...),
    prepare_time: int = Form(...),
    description: str = Form(None),
    # 이미지 파일 (없어도 됨)
    image: UploadFile = File(None),
    image_url: str = Form(None),

    db: Session = Depends(get_db),
    current_user=Depends(get_current_user),
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
    
    image_url = None
    if image:
        # 1) 고유 파일명 생성
        ext = os.path.splitext(image.filename)[1]
        filename = f"{uuid.uuid4().hex}{ext}"
        filepath = os.path.join(UPLOAD_DIR, filename)

        # 2) 디스크에 저장
        with open(filepath, "wb") as f:
            content = await image.read()
            f.write(content)

        # 3) 정적 경로로 쓸 URL 생성 (백엔드가 /static 을 serve 한다고 가정)
        image_url = f"/images/menu/{filename}"
    
    # Update fields
    if name is not None:
        menu_item.name = name
    
    if price is not None:
        menu_item.price = price
    
    if prepare_time is not None:
        menu_item.prepare_time = prepare_time

    if image_url is not None:
        menu_item.image_url = image_url

    if description is not None:
        menu_item.description = description
    
    db.add(menu_item)
    db.commit()
    
    # 데이터 변경 후 웹소켓으로 브로드캐스트
    asyncio.create_task(broadcast_menu_data(db))
    
    return {
        "status": "success",
        "message": "메뉴 항목이 수정되었습니다."
    }

@router.delete("/items/{item_id}", response_model=dict)
async def delete_menu_item(
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
    
    # 데이터 변경 후 웹소켓으로 브로드캐스트
    asyncio.create_task(broadcast_menu_data(db))
    
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
async def create_ingredient(
    ingredient_data: IngredientCreateRequest,
    background_tasks: BackgroundTasks,
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
    
    # 재고 항목도 생성 ()
    new_inventory = Inventory(
        name=new_ingredient.name,
        count=0,
        max_count=10,  # 기본값
        status="OUT_OF_STOCK",
        ingredient_id=new_ingredient.id
    )
    db.add(new_inventory)
    db.commit()
    db.refresh(new_inventory)
        
    # 재고 데이터 변경 이벤트 발생
    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "inventory",
        None
    )
    
    # 메뉴 데이터 변경 후 웹소켓으로 브로드캐스트
    asyncio.create_task(broadcast_menu_data(db))
    
    return {
        "id": new_ingredient.id,
        "status": "success",
        "message": "재료가 생성되었습니다."
    }

@router.put("/ingredients/{ingredient_id}", response_model=dict)
async def update_ingredient(
    ingredient_id: int,
    ingredient_data: IngredientUpdateRequest,
    background_tasks: BackgroundTasks,
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
    
    # 원래 이름 저장
    original_name = ingredient.name
    
    # Update fields
    if ingredient_data.name is not None:
        ingredient.name = ingredient_data.name
    
    if ingredient_data.quantity_required is not None:
        ingredient.quantity_required = ingredient_data.quantity_required
    
    db.add(ingredient)
    db.commit()
    
    # 이름이 변경된 경우 재고 항목도 업데이트
    if original_name != ingredient.name and ingredient_data.name is not None:
        inventory_item = db.query(Inventory).filter(Inventory.name == original_name).first()
        if inventory_item:
            inventory_item.name = ingredient.name
            inventory_item.ingredient_id = ingredient.id
            db.add(inventory_item)
            db.commit()
            
    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "inventory",
        None
    )
    # 메뉴 데이터 변경 후 웹소켓으로 브로드캐스트
    asyncio.create_task(broadcast_menu_data(db))
    
    return {
        "status": "success",
        "message": "재료가 수정되었습니다."
    }

@router.delete("/ingredients/{ingredient_id}", response_model=dict)
async def delete_ingredient(
    ingredient_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):

    # Find ingredient
    ingredient = db.query(MenuIngredient).filter(MenuIngredient.id == ingredient_id).first()
    if not ingredient:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Ingredient with ID {ingredient_id} not found"
        )
    

    #같은 재고 항목이 있다면 삭제
    inventory_item = db.query(Inventory).filter(Inventory.ingredient_id == ingredient.id).first()
    # 재고 항목 삭제
    if inventory_item:
        db.delete(inventory_item)
        db.commit()

    from run import broadcast_entity_update
    # REST API 호출 시 웹소켓 브로드캐스트 트리거
    background_tasks.add_task(
        broadcast_entity_update,
        "inventory",
        None
    )
    # Delete ingredient
    db.delete(ingredient)
    db.commit()
    
    # 메뉴 데이터 변경 후 웹소켓으로 브로드캐스트
    asyncio.create_task(broadcast_menu_data(db))
    
    return {
        "message": "재료가 삭제되었습니다."
    } 