# app/models/inventory.py
# models/inventory.py
from typing import Optional, List
from datetime import datetime

from sqlmodel import SQLModel, Field, Relationship
from sqlalchemy import Column, Enum as SQLEnum, and_
from sqlalchemy.orm import foreign

from .enums import InventoryStatus, EntityType
from .pose6d import Pose6D

class Inventory(SQLModel, table=True):
    __tablename__ = "inventory"

    id: Optional[int] = Field(default=None, primary_key=True)  # int PK
    ingredient_id: Optional[int] = Field(default=None, foreign_key="menuingredient.id")
    name : Optional[str] = Field(default=None, foreign_key="menuingredient.name")
    count: Optional[int] = None
    status: Optional[InventoryStatus] = Field(
        sa_column=Column(SQLEnum(InventoryStatus, name="inventory_status"))
    )
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    menu_ingredients: List["MenuIngredient"] = Relationship(back_populates="inventory")




class MenuItem(SQLModel, table=True):
    __tablename__ = "menuitem"
    """
    메뉴 항목 관리 모델
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    name: Optional[str] = None
    price: Optional[float] = None
    prepare_time: Optional[int] = None
    
    menu_ingredients: List["MenuIngredient"] = Relationship(back_populates="menu_item")
    order_items: List["OrderItem"] = Relationship(back_populates="menu_item")

class MenuIngredient(SQLModel, table=True):
    __tablename__ = "menuingredient"
    """
    메뉴-재고 연결 관계 모델
    """
    id: Optional[int] = Field(default=None, primary_key=True)
    name: Optional[str] = Field(default=None, unique=True)
    menu_item_id: Optional[int] = Field(default=None, foreign_key="menuitem.id")
    quantity_required: Optional[int] = None
    
    menu_item: Optional[MenuItem] = Relationship(back_populates="menu_ingredients")
    inventory: Optional[Inventory] = Relationship(back_populates="menu_ingredients")
