from enum import Enum
from sqlalchemy import Enum as SQLEnum

# ── ENUM 정의 ────────────────────────────────────────────────────────────────
class EntityType(str, Enum):
    COOKBOT = "COOKBOT"
    ALBABOT = "ALBABOT"
    PINKY = "PINKY"
    GLOBAL = "GLOBAL"
    WORLD = "WORLD"
    INVENTORY = "INVENTORY"
    CHATBOT = "CHATBOT"

class RobotStatus(str, Enum):
    IDLE = "IDLE"
    COOKING = "COOKING"
    SERVING = "SERVING"
    CLEANING = "CLEANING"
    EMERGENCY = "EMERGENCY"
    MAINTENANCE = "MAINTENANCE"
    SECURITY = "SECURITY"

class CommandStatus(str, Enum):
    PENDING = "PENDING"
    SENT = "SENT"
    ACKED = "ACKED"
    EXECUTED = "EXECUTED"
    FAILED = "FAILED"

class InventoryStatus(str, Enum):
    IN_STOCK = "IN_STOCK"
    LOW_STOCK = "LOW_STOCK"
    OUT_OF_STOCK = "OUT_OF_STOCK"

class TableStatus(str, Enum):
    AVAILABLE = "AVAILABLE"
    OCCUPIED = "OCCUPIED"

class OrderStatus(str, Enum):
    PLACED = "PLACED"
    PREPARING = "PREPARING"
    SERVED = "SERVED"
    CANCELLED = "CANCELLED"

class EventType(str, Enum):
    WELCOME = "WELCOME"
    CALL = "CALL"
    BIRTHDAY = "BIRTHDAY"
    EMERGENCY = "EMERGENCY"
    CLEANING = "CLEANING"

class StreamSourceType(str, Enum):
    PINKY = "PINKY"
    COOKBOT = "COOKBOT"
    GLOBAL_CAM = "GLOBAL_CAM"

class StreamStatus(str, Enum):
    ACTIVE = "ACTIVE"
    INACTIVE = "INACTIVE"
    ERROR = "ERROR"

class UserRole(str, Enum):
    ADMIN = "ADMIN"
    EMPLOYEE = "EMPLOYEE"
    KIOSK = "KIOSK"

class NotificationStatus(str, Enum):
    PENDING = "PENDING"
    SENT = "SENT"
    FAILED = "FAILED" 