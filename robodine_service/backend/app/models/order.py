from sqlmodel import SQLModel, Field, Relationship, Column
from datetime import datetime
from typing import Optional, List, TYPE_CHECKING
from sqlalchemy import Enum as SQLEnum
from .enums import OrderStatus
from .customer import Customer
from .robot import Robot

if TYPE_CHECKING:
    from .robot import Robot

class KioskTerminal(SQLModel, table=True):
    __tablename__ = "kioskterminal"
    id: Optional[int] = Field(default=None, primary_key=True)
    location: Optional[str] = None
    ip_address: Optional[str] = None
    orders: List["Order"] = Relationship(back_populates="kioskterminal")

class Order(SQLModel, table=True):
    __tablename__ = "order"
    id: Optional[int] = Field(default=None, primary_key=True)
    customer_id: Optional[int] = Field(default=None, foreign_key="customer.id")
    robot_id: Optional[int] = Field(default=None, foreign_key="robot.id")
    kiosk_id: Optional[int] = Field(default=None, foreign_key="kioskterminal.id")
    status: Optional[OrderStatus] = Field(sa_column=Column(SQLEnum(OrderStatus)))
    ordered_at: datetime = Field(default_factory=datetime.utcnow)
    served_at: Optional[datetime] = None
    
    customer: Optional[Customer] = Relationship(back_populates="order")
    robot: Optional[Robot] = Relationship(back_populates="order")
    kiosk: Optional[KioskTerminal] = Relationship(back_populates="order")
    items: List["OrderItem"] = Relationship(back_populates="order")

class OrderItem(SQLModel, table=True):
    __tablename__ = "orderitem"
    order_id: Optional[int] = Field(default=None, foreign_key="order.id", primary_key=True)
    menu_item_id: Optional[int] = Field(default=None, foreign_key="menuitem.id", primary_key=True)
    quantity: Optional[int] = None
    
    order: Optional[Order] = Relationship(back_populates="items")
    menu_item: Optional["MenuItem"] = Relationship(back_populates="order_items")
