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
    table_number: Optional[int] = Field(default=None)
    ip_address: Optional[str] = None

    orders: Optional["Order"] = Relationship(back_populates="kioskterminal")

class Order(SQLModel, table=True):
    __tablename__ = "order"  # 필요하다면 "orders" 로 변경 가능

    id: Optional[int] = Field(default=None, primary_key=True)
    customer_id: Optional[int] = Field(default=None, foreign_key="customer.id")
    robot_id:    Optional[int] = Field(default=None, foreign_key="robot.id")
    kiosk_id:    Optional[int] = Field(default=None, foreign_key="kioskterminal.id")
    status:      Optional[OrderStatus] = Field(sa_column=Column(SQLEnum(OrderStatus)))
    timestamp: Optional[datetime] = Field(default_factory=datetime.utcnow)
    served_at:   Optional[datetime] = None

    # 위에서 정의한 Customer.orders 와 짝을 이루도록 back_populates="orders"
    customer:  Optional[Customer]       = Relationship(back_populates="orders")
    robot:     Optional[Robot]          = Relationship(back_populates="orders")
    kioskterminal:     Optional[KioskTerminal]  = Relationship(back_populates="orders")

class OrderItem(SQLModel, table=True):
    __tablename__ = "orderitem"

    order_id:     Optional[int] = Field(default=None, foreign_key="order.id", primary_key=True)
    menu_item_id: Optional[int] = Field(default=None, foreign_key="menuitem.id", primary_key=True)
    quantity:     Optional[int]

    # MenuItem.order_items 과 짝을 이루도록 back_populates="order_items"
    menu_item: Optional["MenuItem"] = Relationship(back_populates="order_items")
