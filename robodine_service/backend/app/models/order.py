from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime
from .customer import Customer
from .robot import Robot
from typing import Optional

class Order(SQLModel, table=True):
    id: int = Field(primary_key=True)  # Only ID is required
    customer_id: Optional[int] = Field(default=None, foreign_key="customer.id")
    robot_id: Optional[str] = Field(default=None, foreign_key="robot.id")
    item: Optional[str] = None
    quantity: Optional[int] = None
    status: Optional[str] = None
    ordered_time: Optional[datetime] = None
    served_time: Optional[datetime] = None

    # Optional relationships
    customer: Optional[Customer] = Relationship(back_populates="orders", sa_relationship_kwargs={"lazy": "joined"})
    robot: Optional[Robot] = Relationship(back_populates="orders", sa_relationship_kwargs={"lazy": "joined"})
